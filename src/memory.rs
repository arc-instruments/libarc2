use std::collections::VecDeque;

const CHUNK_SIZE: usize = 256; // 64 words per chunk
const MEM_SIZE: usize = 1024*1024*1024; // 1GiB of onboard memory
const FLAG_BASE: u32 = 0x7800_0000; // 2GiB - 128MiB
const DRAM_BASE: u32 = 0x0000_0000;


/// Error type for memory operations. The only two variants at this
/// stage are `ENOMEM` when the FPGA is out of memory and `EDFREE`
/// when an attempt was made to free the same memory segment.
#[derive(Debug)]
pub enum MemoryError {
    /// Out of memory
    ENOMEM,
    /// Double free
    EDFREE,
}

impl std::fmt::Display for MemoryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match *self {
            MemoryError::ENOMEM => write!(f, "Out of memory"),
            MemoryError::EDFREE => write!(f, "Double free")
        }
    }
}

impl std::convert::From<MemoryError> for &str {
    fn from(error: MemoryError) -> Self {
        match error {
            MemoryError::ENOMEM => "Out of memory",
            MemoryError::EDFREE => "Double free",
        }
    }
}

impl std::convert::From<MemoryError> for String {
    fn from(error: MemoryError) -> Self {
        let foo: &str = error.into();
        foo.to_string()
    }
}

impl std::error::Error for MemoryError {}


/// An active region in FPGA memory
///
/// A chunk represents a region in the FPGA DRAM that has been marked
/// as active, which means ArC2 has been instructed to put measurement
/// data there.
pub(crate) struct Chunk {
    _addr: u32,
    _valid: bool,
    _dummy: bool,
}

impl std::fmt::Debug for Chunk {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Chunk")
         .field("_addr", &format_args!("0x{:08x}", &self._addr))
         .field("_valid", &self._valid)
         .field("_dummy", &self._dummy)
         .finish()
    }
}

impl Chunk {

    /// Create a new Chunk at the specified address
    pub fn new(addr: u32) -> Self {
        Chunk { _addr: addr, _valid: true, _dummy: false }
    }

    /// Create a new dummy Chunk for synchronisation purposes
    pub(crate) fn dummy() -> Self {
        Chunk { _addr: 0x0, _valid: true, _dummy: true }
    }

    /// The memory associated with this chunk
    pub fn addr(&self) -> u32 {
        self._addr
    }

    fn offset(&self) -> u32 {
        self._addr / (CHUNK_SIZE as u32)
    }

    /// The associated flag address, raised when the operation
    /// associated with this address is finished
    pub fn flag_addr(&self) -> u32 {
        let offset = self.offset();
        return FLAG_BASE + 4*offset;
    }

    /// Returns `true` if the data is still active, as in not
    /// yet picked up by the consumer. If `false` the region
    /// represented by the chunk is available to reuse.
    fn is_valid(&self) -> bool {
        self._valid
    }

    pub(crate) fn is_dummy(&self) -> bool {
        self._dummy
    }

    /// Mark the memory region represented by this chunk as
    /// invalid. This is typically only called by the memory
    /// manager when freeing memory.
    fn invalidate(&mut self) {
        self._valid = false;
    }
}


/// FPGA DRAM memory manager and tracker
///
/// [`MemMan`] is used internally by `libarc2` to demarcate the
/// FPGA memory. The FPGA itself lacks any kind of memory
/// management capability so the consumer must keep track of the
/// memory spaces in the DRAM that are used for storing results.
/// [`MemMan`] essentially works as a crude memory allocator but
/// instead of allocating memory on the computer it does so on
/// the FPGA DRAM.
///
/// A memory region is represented by a [`Chunk`] which is
/// essentially an address plus a validation marker. This
/// address is typically required by operations that output
/// data, such as [`CurrentRead`][`crate::instructions::CurrentRead`]
/// or [`VoltageRead`][`crate::instructions::VoltageRead`]. In order
/// to accomodate these operations the memory manager issues a
/// [`Chunk`] of memory that is initially marked as valid (or active).
/// While the chunk is valid no data will be written to this address
/// until the chunk is freed (ie. marked as invalid). Then its
/// address will be put back into the available address pool. It is
/// responsibility of the caller to free the chunks whose data have
/// been picked up otherwise the FPGA will run, eventually, out of
/// memory.
pub(crate) struct MemMan {
    // Highest available address
    top: u32,
    // Total blocks available to allocate
    total_blocks: usize,
    // Remaining blocks available to allocate
    free_blocks: usize,
    // Reclaimed addresses; these are expended first
    // before `top` is increased
    stack: VecDeque<u32>
}

impl MemMan {

    /// Initialise a new memory manager with the default
    /// memory layout.
    pub fn new() -> Self {

        MemMan {
            top: DRAM_BASE,
            total_blocks: MEM_SIZE / CHUNK_SIZE,
            free_blocks: MEM_SIZE / CHUNK_SIZE,
            stack: VecDeque::with_capacity(2048)
        }

    }

    /// Find out which one is the next available address. Returns [`None`]
    /// if all available address space has been exhausted. This function
    /// does _not_ allocate any memory.
    fn next(&self) -> Option<u32> {
        if self.free_blocks == 0 {
            return None;
        }

        if self.stack.len() > 0 {
            return Some(*self.stack.front().unwrap());
        } else {
            return Some(self.top);
        }
    }

    /// Allocate a new chunk if memory is available. There is
    /// no guarantee that this chunk will be sequential compared
    /// to a previous allocation as the allocator will try to
    /// reuse memory as much as possible.
    pub fn alloc_chunk(&mut self) -> Result<Chunk, MemoryError> {
        if self.free_blocks == 0 {
            return Err(MemoryError::ENOMEM);
        }

        let chunk: Chunk;

        // Check if we can reuse any of the old addresses, if not
        // increase the top address to accomodate for a new one
        if self.stack.len() > 0 {
            chunk = Chunk::new(self.stack.pop_front().unwrap());
        } else {
            chunk = Chunk::new(self.top);
            self.top += CHUNK_SIZE as u32;
        }

        // in any case decrease the number of available blocks
        self.free_blocks -= 1;

        Ok(chunk)
    }

    /// Release the address represented by a [`Chunk`] back into the
    /// available memory pool.
    pub fn free_chunk(&mut self, chunk: &mut Chunk) -> Result<(), MemoryError> {

        if chunk.is_dummy() {
            chunk.invalidate();
            return Ok(())
        }

        if !chunk.is_valid() {
            return Err(MemoryError::EDFREE);
        }

        self.stack.push_back(chunk.addr());
        chunk.invalidate();
        self.free_blocks += 1;

        Ok(())
    }

}


#[cfg(test)]
mod tests {

    use assert_matches::assert_matches;
    use super::{MemMan, Chunk, MemoryError, CHUNK_SIZE, DRAM_BASE};

    #[test]
    fn memman_test_alloc() {

        let mut manager = MemMan::new();
        assert_eq!(manager.free_blocks, manager.total_blocks);

        let mut vec: Vec<Chunk> = Vec::new();

        for i in 0..10 {
            let chunk = manager.alloc_chunk().unwrap();
            assert_eq!(chunk.addr(), DRAM_BASE + i*CHUNK_SIZE as u32);
            vec.push(chunk);
        }

        assert_eq!(manager.free_blocks, manager.total_blocks - vec.len());

    }

    #[test]
    fn memman_test_overalloc() {
        let mut manager = MemMan::new();

        let mut vec: Vec<Chunk> = Vec::new();


        let mut success = false;

        // An error should be raised when trying to overallocate
        // more than 1024*1024*1024 / 256 addresses
        for i in 0..u32::MAX {
            match manager.alloc_chunk() {
                Ok(c) => vec.push(c),
                Err(_) => { success = true; break; }
            }
        }

        assert_eq!(success, true);
    }

    #[test]
    fn memman_test_no_double_free() {
        let mut manager = MemMan::new();

        let mut chunk = manager.alloc_chunk().unwrap();

        // First free is fine
        assert_matches!(manager.free_chunk(&mut chunk), Ok(()));

        assert_eq!(manager.free_blocks, manager.total_blocks);

        // Second free throws double free error
        assert_matches!(manager.free_chunk(&mut chunk), Err(MemoryError::EDFREE));

    }

    #[test]
    fn memman_test_reclaim() {
        let mut manager = MemMan::new();
        let chunk0 = manager.alloc_chunk().unwrap();
        let mut chunk1 = manager.alloc_chunk().unwrap();
        let chunk2 = manager.alloc_chunk().unwrap();

        assert_eq!(chunk0.addr(), DRAM_BASE);
        assert_eq!(chunk1.addr(), DRAM_BASE + CHUNK_SIZE as u32);
        assert_eq!(chunk2.addr(), DRAM_BASE + 2u32*CHUNK_SIZE as u32);

        assert_eq!(manager.next(), Some(DRAM_BASE + 3u32*CHUNK_SIZE as u32));

        manager.free_chunk(&mut chunk1);

        // chunk1's address should be back in the pool
        assert_eq!(manager.next(), Some(DRAM_BASE + CHUNK_SIZE as u32));


    }

}
