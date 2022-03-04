use std::{time, thread};
use std::collections::HashSet;
use std::sync::{RwLock, Arc, Mutex, atomic};
use std::sync::mpsc::{channel, Sender, Receiver, TryRecvError};
use beastlink as bl;
use ndarray::{Array, Ix1, Ix2};
use thiserror::Error;

use crate::instructions::*;
use crate::registers::{ChannelState, ChanMask, IOMask};
use crate::registers::{ChannelConf, PulseAttrs, ClusterMask};
use crate::registers::consts::HSCLUSTERMAP;
use crate::memory::{MemMan, Chunk, MemoryError};

const EFM03_VID: u16 = 0x10f8;
const EFM03_PID: u16 = 0xc583;
const BASEADDR: u32 = 0x80000000;
const FIFOBUSYADDR: u32 = 0x80020000;
const WRITEDELAY: time::Duration = time::Duration::from_nanos(2_500_000);
const BLFLAGS_W: bl::Flags = bl::Flags::ConstAddress;
const BLFLAGS_R: bl::Flags = bl::Flags::NoFlags;
const INBUF: usize = 64*std::mem::size_of::<u32>();
const VALUEAVAILFLAG: u32 = 0xcafebabe;
const INSTRCAP: usize = 768*9*std::mem::size_of::<u32>();

// We are caching common instructions
lazy_static! {
    static ref UPDATE_DAC: UpdateDAC = {
        let mut instr = UpdateDAC::new();
        instr.compile();
        instr
    };

    static ref RESET_DAC: ResetDAC = {
        let mut instr = ResetDAC::new();
        instr.compile();
        instr
    };

    static ref SET_3V3_LOGIC: SetDAC = {
        let mut instr = SetDAC::new_3v3_logic();
        instr.compile();
        instr
    };

    static ref CHAN_FLOAT_ALL: UpdateChannel = {
        let chanconf = ChannelConf::new_with_state(ChannelState::Open);
        let mut instr = UpdateChannel::from_regs_default_source(&chanconf);

        instr.compile();
        instr
    };

    static ref CHAN_ARB_ALL: UpdateChannel = {
        let chanconf = ChannelConf::new_with_state(ChannelState::VoltArb);
        let mut instr = UpdateChannel::from_regs_default_source(&chanconf);

        instr.compile();
        instr
    };

    static ref PREP_AMP_ALL: AmpPrep = {
        let mut mask = ChanMask::new();
        mask.set_enabled_all(true);
        let mut instr = AmpPrep::new(&mask);

        instr.compile();
        instr
    };

    static ref ZERO_HS_TIMINGS: HSConfig = {
        let mut instr = HSConfig::new([0u32; 8]);
        instr.compile();
        instr
    };

    static ref ALL_CHANS: Vec<usize> = {
        (0usize..64).collect()
    };

    static ref ALL_WORDS: Vec<usize> = {
        let mut channels: Vec<usize> = Vec::with_capacity(32);
        channels.append(&mut (16usize..32).collect::<Vec<usize>>());
        channels.append(&mut (48usize..64).collect::<Vec<usize>>());

        channels
    };

    static ref ALL_WORDS_SET: HashSet<usize> = {
        let mut set: HashSet<usize> = HashSet::with_capacity(32);

        for i in 16usize..32 {
            set.insert(i);
        }

        for i in 48usize..64 {
            set.insert(i);
        }

        set
    };

    static ref ALL_BITS: Vec<usize> = {
        let mut channels: Vec<usize> = Vec::with_capacity(32);
        channels.append(&mut ( 0usize..16).collect::<Vec<usize>>());
        channels.append(&mut (32usize..48).collect::<Vec<usize>>());

        channels
    };

    static ref ALL_BITS_SET: HashSet<usize> = {
        let mut set: HashSet<usize> = HashSet::with_capacity(32);

        for i in  0usize..16 {
            set.insert(i);
        }

        for i in 32usize..48 {
            set.insert(i);
        }

        set
    };
}

#[derive(Error, Debug)]
pub enum ArC2Error {
    /// Low level FPGA communication error
    #[error("FPGA Error: {0}")]
    FPGAError(#[from] bl::BLError),
    /// Memory management error
    #[error("Memory error: {0}")]
    MemoryError(#[from] MemoryError),
    /// Invalid Device ID or device not existing
    #[error("Invalid ArC2 device id: {0}")]
    InvalidID(i32),
    /// Invalid ramp parameters
    #[error("Ramp operation error: Vstart {0}, Vstop {1}, Vstep {2}")]
    RampOperationError(f32, f32, f32),
    /// Output buffer access error
    #[error("Cannot store address {0} to output buffer")]
    OutputBufferError(i64)
}

impl std::convert::From<std::sync::mpsc::SendError<Option<Chunk>>> for ArC2Error {
    fn from(error: std::sync::mpsc::SendError<Option<Chunk>>) -> Self {
        let chunk = error.0;
        match chunk {
            Some(c) => ArC2Error::OutputBufferError(c.addr() as i64),
            None => ArC2Error::OutputBufferError(-1)
        }
    }
}


/// Current status of the TIA feedback loop
#[derive(Clone, Debug)]
enum TIAState {
    Closed,
    Open
}


/// Order of read/pulse operation when reading all crosspoints
///
/// This enum signifies how a _{read,pulse} all_ operation should be
/// done with respect to the geometry of a full 32×32 array. `Columns`
/// will bias rows (commonly referred to as *bitlines*) whereas
/// `Rows` will bias columns (commonly referred to as *wordlines*).
#[derive(Clone)]
pub enum BiasOrder {
    Columns,
    Rows
}

/// Daughterboard mode of operation
///
/// This enum is used to communicate how devices are controlled when a
/// daughterboard is connected. In the case of the 32NAA daughterboard
/// when devices are inserted directly into the PLCC socket, control mode
/// should be `Internal`. If device connections are broken out from the
/// header then control mode should be `Header`. This affects the way the
/// I/O logic is configured internally.
#[derive(Clone)]
pub enum ControlMode {
    Header,
    Internal
}

/// Read-out voltage configuration for pulse trains
///
/// This enum controls _how_ read-outs should be performed during a ramp
/// operation. If `Bias` is selected then read-outs will be done at the
/// existing bias (this would be common for an I-V like ramp). If `Arb`
/// is selected then read-outs will be performed at the user specified
/// voltage. If `Never` is selected no read-outs will be performed (which
/// also implies [`ReadAfter::Never`].
#[derive(Clone)]
pub enum ReadAt {
    Bias,
    Arb(f32),
    Never
}

impl ReadAt {
    /// Unwrap the inner Arbitrary voltage. This will panic if `self` is
    /// not of type [`ReadAt::Arb`]
    pub fn arb(self) -> f32 {
        if let ReadAt::Arb(v) = self { v } else { panic!("Not Arb variant") }
    }

    fn is_never(&self) -> bool {
        match self {
            ReadAt::Never => true,
            _ => false
        }
    }
}

/// Read-out frequency configuration for pulse trains
///
/// This enum controls _when_ read-outs should be performed during a ramp
/// operation. If `Pulse` is selected the read-outs will be done after
/// each pulse. If `Ramp` is selected a single read-out will be performed
/// at the end of the pulse train. `Block` will perform a single read after
/// all repetitions of a pulse at a specific voltage have elapsed. Obviously
/// if singular pulse ramps are used (one pulse per voltage), `Block` and
/// `Pulse` behave identically. If `Never` is used then no read-outs
/// will be performed (which also implies [`ReadAt::Never`]).
#[derive(Clone)]
pub enum ReadAfter {
    Pulse,
    Block,
    Ramp,
    Never
}

impl ReadAfter {

    fn is_never(&self) -> bool {
        match self {
            ReadAfter::Never => true,
            _ => false
        }
    }

    fn is_at_ramp(&self) -> bool {
        match self {
            ReadAfter::Ramp => true,
            _ => false
        }
    }
}

/// Read-out mode for bulk memory reads
///
/// This is primarily used with [`Instrument::pick_one`] to read a block
/// of memory values from the internal long process buffer. When `Words`
/// is selected only the 32 values that correspond to configured word channels
/// will be retrieved in ascending channel order. When `Bits` is selected
/// the 32 values that correspond to the configured bit channels will be
/// retrieved, again in ascending channel order. When `All` is selected all
/// raw values in ascending channel number are retrieved.
#[derive(Clone)]
pub enum DataMode {
    Words,
    Bits,
    All
}


/// ArC2 entry level object
///
/// `Instrument` implements the frontend for the ArC2. Its role is essentially
/// to process instructions and read back results. ArC2 processes instructions
/// either in _immediate_ mode or _retained_ mode. In the first case instructions
/// are written to the tool as soon they are issued. In retained mode
/// instructions will be gathered into a buffer which **must** be flushed
/// explicitly for them to have any effect.
#[derive(Clone)]
pub struct Instrument {

    // Handle to underlying device
    efm: Arc<Mutex<bl::Device>>,

    // We need reference counting here so that we can use the
    // chunked iterator on `execute`. Since this is a mutating
    // iteration over the buffer we need to have an exclusive lock
    // over the buffer. The same could be done with Rc<RefCell<>>
    // but unfortunately these do not implement Sync and the python
    // bindings will fail to compile. Hence the use of Arc/Mutex
    instr_buffer: Option<Arc<RwLock<Vec<u8>>>>,

    // Memory management
    memman: Arc<RwLock<MemMan>>,

    // Long operation handling
    _sender: Sender<Option<Chunk>>,
    _receiver: Arc<Mutex<Receiver<Option<Chunk>>>>,
    // A thread has been spawned
    _op_running: Arc<atomic::AtomicBool>,

    _tia_state: TIAState,
}

/// Find available device IDs.
///
/// This function will enumerate all available boards and return an array
/// with the IDs of all found devices. This can be passed on to
/// [`Instrument::open()`] to connect to a specific device.
pub fn find_ids() -> Result<Vec<i32>, ArC2Error> {
    /*let max_id = match bl::enumerate(EFM03_VID, EFM03_PID) {
        Ok(v) => Ok(v),
        Err(err) => Err(format!("Could not enumerate devices: {}", err))
    }?;*/
    let max_id = bl::enumerate(EFM03_VID, EFM03_PID)?;

    if max_id >= 0 {
        Ok((0..max_id).collect())
    } else {
        // max_id -1 usually indicates an error...
        Err(ArC2Error::InvalidID(max_id))
    }
}


/// Convert a raw ADC value to current reading
fn _adc_to_current(val: u32) -> f32 {

    // first two bytes is the range
    let range = ((val >> 24) & 0xFF) as u8;

    // get the rest of the bytes
    let mut bytes = val.to_be_bytes();

    // replace the first 2 bytes (where the range was)
    // with 0 to simplify things
    bytes[0] = 0;

    // and reconstruct the value into an i32 (which is
    // what the ADC is outputting)
    let uval = i32::from_be_bytes(bytes);

    let val: f32;
    let res: f32;
    let temp: f32;

    // If value is > 2e17 it means that ADC output has
    // overflowed, so fix that. We know the ADC has only
    // 18-bit precision so overflows cannot happen.
    if uval > 2i32.pow(17) {
        val = (uval - 2i32.pow(18)) as f32;
    } else {
        val = uval as f32;
    }

    // Calculation formula for different ranges
    if range == 0x82 || range == 0x84 || range == 0x88 {
        temp = 20.48 * (val / 2.0f32.powf(18.0));
    } else {
        temp = 10.24 * (val / 2.0f32.powf(18.0));
    }

    if range == 0x82 || range == 0x90 {
        res = temp / 830.0;
    } else if range == 0x84 || range == 0xa0 {
        res = temp / 110.0e3;
    } else if range == 0x88 || range == 0xc0 {
        res = temp / 15.0e6;
    } else {
        res = f32::NAN;
    }

    res

}


impl Instrument {

    /// Create a new Instrument with a known ID.  Use [`find_ids`]
    /// to discover devices. Set `retained_mode` to `true` to defer
    /// writing instructions until an [`Instrument::execute()`]
    /// has been called explicitly.
    pub fn open(id: i32, retained_mode: bool) -> Result<Instrument, ArC2Error> {

        if !find_ids()?.contains(&id) {
            return Err(ArC2Error::InvalidID(id));
        }

        let buffer: Option<Arc<RwLock<Vec<u8>>>>;

        // If in retained mode preallocate space for 10×768 instructions.
        // If not the instruction buffer is not used.
        if retained_mode {
            buffer = Some(Arc::new(RwLock::new(Vec::with_capacity(10usize*INSTRCAP))));
        } else {
            buffer = None;
        }

        let (sender, receiver) = channel::<Option<Chunk>>();

        match bl::Device::open(id) {
            Ok(d) => Ok(Instrument {
                efm: Arc::new(Mutex::new(d)),
                instr_buffer: buffer,
                memman: Arc::new(RwLock::new(MemMan::new())),
                _sender: sender,
                _receiver: Arc::new(Mutex::new(receiver)),
                _op_running: Arc::new(atomic::AtomicBool::new(false)),
                _tia_state: TIAState::Open,

            }),
            Err(err) => Err(ArC2Error::FPGAError(err))
        }
    }

    /// Load an FPGA bitstream from a file.
    pub fn load_firmware(&self, path: &str) -> Result<(), ArC2Error> {
        let _efm = self.efm.clone();
        let efm = _efm.lock().unwrap();
        /*match efm.program_from_file(&path) {
            Ok(()) => { Ok(()) },
            Err(err) => { Err(format!("Could not program FPGA: {}", err)) }
        }*/
        efm.program_from_file(&path).map_err(|e| ArC2Error::FPGAError(e) )
    }

    /// Open a new Instrument with a specified id and bitstream.
    /// This essentially combines [`Instrument::open()`] and
    /// [`Instrument::load_firmware()`].
    pub fn open_with_fw(id: i32, path: &str, retained_mode: bool) -> Result<Instrument, ArC2Error> {
        let mut instr = Instrument::open(id, retained_mode)?;
        instr.load_firmware(&path)?;
        instr._amp_prep()?;
        instr.process(&*RESET_DAC)?;
        instr.process(&*SET_3V3_LOGIC)?;
        instr.process(&*UPDATE_DAC)?;
        instr.add_delay(30_000u128)?;
        instr.execute()?;

        Ok(instr)
    }

    /// Process a compiled instruction
    pub fn process<T: Instruction>(&mut self, instr: &T) -> Result<(), ArC2Error> {

        instrdbg!(instr);

        // convert the instruction into raw bytes
        let mut bytes = instr.to_bytevec();

        if let Some(buff) = &mut self.instr_buffer.clone() {
            buff.write().unwrap().extend(bytes);
            return Ok(());
        }

        // Otherwise write directly to ArC2 (immediate)
        let _efm = self.efm.clone();
        let efm = _efm.lock().unwrap();
        #[cfg(not(feature="dummy_writes"))]
        match efm.write_block(BASEADDR, &mut bytes, BLFLAGS_W) {

            Ok(()) => {
                thread::sleep(WRITEDELAY);
            },
            Err(err) => return Err(ArC2Error::FPGAError(err))
        }

        #[cfg(feature="dummy_writes")]
        eprintln!("DW: {:?}", bytes);

        Ok(())
    }

    /// Zero an FPGA address chunk
    #[cfg(feature="zero_before_write")]
    fn _zero_chunk(&mut self, chunk: &Chunk) -> Result<(), ArC2Error> {
        let mut zerobuf: [u8; INBUF] = [0u8; INBUF];
        let addr = chunk.addr();

        eprintln!("Trying to zero chunk");
        match self.efm.write_block(addr, &mut zerobuf, BLFLAGS_W) {
            Ok(()) => {

                #[cfg(feature="debug_packets")]
                eprintln!("ZERO: {:08x} → {:08x}", addr, (addr as usize)+INBUF-1);

                Ok(())
            },
            Err(err) => Err(ArC2Error::FPGAError(err))
        }
    }

    /// Write all retained instructions to ArC2. Has no effect if
    /// `retained_mode` is `false` since all instructions are
    /// executed as they are issued.
    pub fn execute(&mut self) -> Result<&mut Self, ArC2Error> {

        match self.instr_buffer.clone() {
            Some(ref mut buf) => {
                #[cfg(not(feature="dummy_writes"))] {
                    // lock the instruction buffer
                    let mut actual_buf = buf.write().unwrap();

                    // if buffer is smaller than the instruction cap no
                    // chunking is practically needed (or to put it
                    // otherwise: there is only one chunk) so there is
                    // no need to explicitly wait for the FPGA to consume
                    // the FIFO before writing the next one.
                    // If that's the case we can quickly break out of the
                    // loop.
                    let quick_break = actual_buf.len() < INSTRCAP;

                    let _efm = self.efm.clone();
                    let efm = _efm.lock().unwrap();

                    // split buffer in chunks
                    for chunk in actual_buf.chunks_mut(INSTRCAP) {
                        // write the chunk to the FPGA
                        match efm.write_block(BASEADDR, chunk, BLFLAGS_W) {
                            Ok(()) => {

                                #[cfg(feature="debug_packets")] {
                                    eprintln!("FLUSH");
                                }

                                if quick_break {
                                    break
                                }

                                // wait until the FPGA instr. buffer is expended
                                // otherwise new instructions might overwrite older
                                // ones
                                self.__wait_no_lock(&efm);
                            },
                            Err(err) => { return Err(ArC2Error::FPGAError(err)); }
                        }
                    }
                    thread::sleep(WRITEDELAY);
                    // empty the buffer
                    actual_buf.clear();
                    Ok(self)
                }
                #[cfg(feature="dummy_writes")] {
                    eprintln!("DW: {:?}", buf);
                    buf.lock().unwrap().clear();
                    Ok(self)
                }
            },
            None => Ok(self)
        }

    }

    /// Compiles and process an instruction
    pub fn compile_process<T: Instruction>(&mut self, instr: &mut T) -> Result<(), ArC2Error> {

        self.process(instr.compile())

    }

    /// Clears everything in the output buffer. Use this to discard any
    /// unwanted results and free up the memory. If you want to retrieve
    /// values use [`Instrument::pick_one()`] which will free up memory
    /// as values are retrieved.
    pub fn free_output_buffer(&self) -> Result<(), ArC2Error> {
        let _receiver = self._receiver.clone();
        let receiver = _receiver.lock().unwrap();

        // iterate through everything in the receiver and free
        // the address chunks
        for item in receiver.iter() {
            match item {
                Some(mut chunk) => {
                    let _memman = self.memman.clone();
                    let mut memman = _memman.write().unwrap();
                    memman.free_chunk(&mut chunk)?
                    // memman is released here
                },
                None => {}
            }
        }

        Ok(())

    }

    /// Allocate a memory area
    fn make_chunk(&self) -> Result<Chunk, ArC2Error> {
        let _memman = self.memman.clone();
        let mut memman = _memman.write().unwrap();

        /*match memman.alloc_chunk() {
            Ok(chunk) => Ok(chunk),
            Err(err) => Err(err.to_string())
        }*/
        memman.alloc_chunk().map_err(|e| ArC2Error::MemoryError(e))
    }

    /// Read a chunk's contents in a word, bit or full mode
    fn read_chunk(&self, chunk: &mut Chunk, mode: &DataMode) -> Result<Vec<f32>, ArC2Error> {

        let ret: Vec<f32> = match mode {
            DataMode::Words => self.word_currents_from_address(chunk.addr())?,
            DataMode::Bits => self.bit_currents_from_address(chunk.addr())?,
            DataMode::All => {
                self.currents_from_address(chunk.addr(), &ALL_CHANS)?
            }
        };

        let _memman = self.memman.clone();
        let mut memman = _memman.write().unwrap();
        memman.free_chunk(chunk)?;

        /*
         * TODO: Zero offset flag when this is supported by the FPGA
         */
        #[cfg(feature="flag_addresses")] {
            let _efm = self.efm.clone();
            let efm = _efm.lock().unwrap();
            // clear flag
            efm.write_block(chunk.flag_addr(), &mut [0x0, 0x0, 0x0, 0x0], BLFLAGS_W)?;
        }

        Ok(ret)
    }


    /// Add a delay to the FIFO command buffer
    pub fn add_delay(&mut self, nanos: u128) -> Result<&mut Self, ArC2Error> {

        // If a small delay is requested process it immediately, it will
        // be over before the computer can blink!
        if nanos <= 500_000u128 {
            self.process(Delay::from_nanos(nanos).compile())?;
        } else {
            // In any other case split the delay into two delays. A large
            // delay that holds the bulk of the waiting and a zero delay
            // that always corresponds to a minimum delay. That way if the
            // delay instruction is the last in the instruction buffer then
            // it will be terminated on a small delay. This is necessary
            // because if a large delay is the last command in the buffer
            // the FIFO will erroneously look empty *while* ArC2 is busy
            // delaying.
            self.process(Delay::from_nanos(nanos-Delay::MIN_NS).compile())?;
            self.process(Delay::from_nanos(0u128).compile())?;
        }
        Ok(self)
    }

    /// Read raw data from block memory
    fn read_raw(&self, addr: u32) -> Result<Vec<u8>, ArC2Error> {
        let _efm = self.efm.clone();
        let efm = _efm.lock().unwrap();

        match efm.read_block(addr, INBUF as i32, BLFLAGS_R) {
            Ok(buf) => { pktdbg!(buf); Ok(buf) },
            Err(err) => Err(ArC2Error::FPGAError(err))
        }

    }

    /// Reset all DACs on the tool. This will execute existing buffers.
    pub fn reset_dacs(&mut self) -> Result<&mut Self, ArC2Error> {
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(30_000u128)?;
        self.execute()
    }

    /// Disconnect all channels
    pub fn float_all(&mut self) -> Result<&mut Self, ArC2Error> {
        self.process(&*CHAN_FLOAT_ALL)?;
        self._tia_state = TIAState::Open;
        Ok(self)
    }

    /// Ground all channels reverting them to VoltArb
    pub fn ground_all(&mut self) -> Result<&mut Self, ArC2Error> {
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(30_000u128)?;
        self._amp_prep()?;
        self.process(&*CHAN_ARB_ALL)?;

        Ok(self)
    }

    /// Modify previously configured channels by switching them to ground. Use an
    /// empty channel list to release. This will clear any other ground/floating
    /// instructions, for instance one issued with [`Instrument::connect_to_ac_gnd()`].
    pub fn connect_to_gnd(&mut self, channels: &[usize]) -> Result<&mut Self, ArC2Error> {

        let mut chanmask = ChanMask::new();
        for c in channels {
            chanmask.set_enabled(*c, true);
        }

        let mut instr = ModifyChannel::from_masks(&chanmask, &ChanMask::none(),
            &ChanMask::none());
        self.process(instr.compile())?;

        Ok(self)

    }

    /// Modify previously configured channels by switching them to a capacitor backed ground.
    /// Use an empty channel list to release. This will clear any other ground/floating
    /// instructions, for instance one issued with [`Instrument::connect_to_gnd()`].
    pub fn connect_to_ac_gnd(&mut self, channels: &[usize]) -> Result<&mut Self, ArC2Error> {

        let mut chanmask = ChanMask::new();
        for c in channels {
            chanmask.set_enabled(*c, true);
        }

        let mut instr = ModifyChannel::from_masks(&ChanMask::none(), &chanmask,
            &ChanMask::none());
        self.process(instr.compile())?;

        Ok(self)

    }

    /// Prepare the DACs for transition to VoltArb or CurrentRead
    fn _amp_prep(&mut self) -> Result<&mut Self, ArC2Error> {
        match self._tia_state {
            TIAState::Open => {
                self._tia_state = TIAState::Closed;
                // set DACs to 0V and connect to GND
                self.ground_all_fast()?;
                self.connect_to_gnd(&ALL_CHANS)?;
                self.process(&*PREP_AMP_ALL)?;
                self.add_delay(100_000u128)?;
                // and disconnect GND before moving on
                self.connect_to_gnd(&[])
            },
            TIAState::Closed => {
                Ok(self)
            }
        }
    }

    /// Set all DACs to ground maintaining current channel state
    pub fn ground_all_fast(&mut self) -> Result<&mut Self, ArC2Error> {
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(30_000u128)?;

        Ok(self)
    }

    /// Set global 3.3 V logic level
    pub fn set_3v3_logic(&mut self) -> Result<&mut Self, ArC2Error> {
        self.process(&*SET_3V3_LOGIC)?;
        self.load_dacs()
    }

    /// Update the DAC configuration
    pub fn load_dacs(&mut self) -> Result<&mut Self, ArC2Error> {
        self.process(&*UPDATE_DAC)?;
        Ok(self)
    }

    /// Retrieve all channel currents from specific address segment. This function will
    /// always return a 64-element vector. Non-selected channels will be replaced by
    /// f32::NAN. The function will panic if a channel number exceeding the highest
    /// channel (presently 63) is provided or if an invalid base address is selected.
    /// Base address must be a multiple of 256.
    pub fn currents_from_address(&self, addr: u32, chans: &[usize]) -> Result<Vec<f32>, ArC2Error> {

        if addr % 256 != 0 {
            panic!("Attempted to read currents from invalid base address");
        }

        let data = self.read_raw(addr)?;
        let mut result: Vec<f32> = vec![f32::NAN; 64];

        // Assemble the number from the 4 neighbouring bytes
        for chan in chans {
            let val: u32 = u32::from_le_bytes([data[4*chan+0], data[4*chan+1],
                data[4*chan+2], data[4*chan+3]]);

            if chan % 2 == 0 {
                result[*chan] = _adc_to_current(val);
            } else {
                result[*chan] = -1.0*_adc_to_current(val);
            }
        }

        Ok(result)

    }

    /// Retrieve all wordline currents from specific address segment. This function will
    /// always return a 32-element vector. The function will panic if an invalid base
    /// address is provided. Base address must be a multiple of 256.
    pub fn word_currents_from_address(&self, addr: u32) -> Result<Vec<f32>, ArC2Error> {

        if addr % 256 != 0 {
            panic!("Attempted to read currents from invalid base address");
        }

        let data = self.read_raw(addr)?;
        let mut result: Vec<f32> = Vec::with_capacity(32);

        for chan in &*ALL_WORDS {
            let val: u32 = u32::from_le_bytes([data[4*chan+0], data[4*chan+1],
                data[4*chan+2], data[4*chan+3]]);

            if chan % 2 == 0 {
                result.push(_adc_to_current(val));
            } else {
                result.push(-1.0*_adc_to_current(val));
            }
        }

        Ok(result)
    }

    /// Retrieve all bitline currents from specific address segment. This function will
    /// always return a 32-element vector.
    pub fn bit_currents_from_address(&self, addr: u32) -> Result<Vec<f32>, ArC2Error> {

        if addr % 256 != 0 {
            panic!("Attempted to read currents from invalid base address");
        }

        let data = self.read_raw(addr)?;
        let mut result: Vec<f32> = Vec::with_capacity(32);

        for chan in &*ALL_BITS {
            let val: u32 = u32::from_le_bytes([data[4*chan+0], data[4*chan+1],
                data[4*chan+2], data[4*chan+3]]);

            if chan % 2 == 0 {
                result.push(_adc_to_current(val));
            } else {
                result.push(-1.0*_adc_to_current(val));
            }
        }

        Ok(result)
    }

    /// Common read functionality with one low channel and several high channels.
    /// This function is guaranteed never to flush the output.
    fn _read_slice_inner(&mut self, low: usize, highs: &[usize], vread: u16)
        -> Result<Chunk, ArC2Error> {

        let zero: u16 = vidx!(0.0);

        // generate a list of dac settings, only one channel in this case
        let setdacs = SetDAC::from_channels(&[(low as u16, vread, vread)], (zero, zero), false);

        self._amp_prep()?;
        // process them with the appropriate delay
        for mut instr in setdacs {
            self.process(instr.compile())?;
        }
        self.process(&*UPDATE_DAC)?;
        self.add_delay(30_000u128)?;

        // set all channels to Arbitrary Voltage
        let mut channelconf =
            UpdateChannel::from_regs_global_state(ChannelState::VoltArb);
        self.process(channelconf.compile())?;

        // Prepare the ADC mask for the readout
        let mut adcmask = ChanMask::new();

        for chan in highs {
            adcmask.set_enabled(*chan, true);
        }

        let chunk = self.make_chunk().unwrap();

        #[cfg(feature="zero_before_write")]
        match self._zero_chunk(&chunk) {
            Ok(()) => {},
            Err(err) => { eprintln!("Zeroing chunk at {} failed: {}", chunk.addr(), err) }
        };

        let mut currentread = CurrentRead::new(&adcmask, chunk.addr(),
            chunk.flag_addr(), VALUEAVAILFLAG);
        self.process(currentread.compile())?;
        self.add_delay(1_000u128)?;

        // The read operation has been assembled, return back the readout address.
        Ok(chunk)
    }

    /// Perform a current read between the specified channels. A voltage
    /// of `-vread` will be applied to the `low` channel and current will
    /// be read from the `high` channel.
    pub fn read_one(&mut self, low: usize, high: usize, vread: f32) -> Result<f32, ArC2Error> {

        // Reset DAC configuration
        self.reset_dacs()?;

        // Initiate a read operation, get the address of the data to be...
        let mut chunk = self._read_slice_inner(low, &[high], vidx!(-vread))?;

        // ... and finally withdraw voltage from the biasing channels
        self.ground_all_fast()?.execute()?;

        // Read the requested chunk...
        let res = self.read_chunk(&mut chunk, &DataMode::All)?;

        // ... and return only the requested channel
        Ok(res[high])
    }

    /// Read all the values which have `chan` as the low potential channel
    ///
    /// If `chan` is between 0 and 15 or 32 and 47 (inclusive) this will correspond
    /// to a row read at `vread` in a standard 32×32 array, otherwise it's a column read.
    pub fn read_slice(&mut self, chan: usize, vread: f32) -> Result<Vec<f32>, ArC2Error> {

        // Reset DAC configuration
        self.reset_dacs()?;

        let res: Vec<f32>;
        let mut chunk: Chunk;

        // Channels 0..16 and 32..48 correspond to rows
        // so we want to read from all the columns in the
        // row
        if (chan < 16) || ((32 <= chan) && (chan < 48)) {
            // Initiate a read operation get the address of the data to be...
            chunk = self._read_slice_inner(chan, &*ALL_WORDS, vidx!(-vread))?;
            // ... and finally withdraw voltage from the biasing channels
            self.ground_all_fast()?.execute()?;
            res = self.read_chunk(&mut chunk, &DataMode::Words)?;
        // Or the other way around. Channels 16..32 and
        // 48..64 correspond to columns so we want to read
        // from all the rows in the column.
        } else {
            // Initiate a read operation get the address of the data to be...
            chunk = self._read_slice_inner(chan, &*ALL_BITS, vidx!(-vread))?;
            // ... and finally withdraw voltage from the biasing channels
            self.ground_all_fast()?.execute()?;
            res = self.read_chunk(&mut chunk, &DataMode::Bits)?;
        }

        Ok(res)
    }

    /// Read the specified high channels that have `chan` as the low potential channel
    ///
    /// If `chan` is between 0 and 15 or 32 and 47 (inclusive) this will correspond
    /// to a row read at `vread` in a standard 32×32 array, otherwise it's a column read.
    /// This is equivalent to read_slice but replaces channels not contained in the
    /// mask with `f32::NAN`.
    pub fn read_slice_masked(&mut self, chan: usize, mask: &[usize], vread: f32) -> Result<Vec<f32>, ArC2Error> {

        // Reset DAC configuration
        self.reset_dacs()?;

        // We are using both the set and the vector to maintain
        // numeric order of the channels. The set is only used
        // for checking whether a particular channel is contained
        // in the returning set of channels or not.
        let all_channels: &Vec<usize>;
        let all_channels_set: &HashSet<usize>;

        // Channels 0..16 and 32..48 correspond to rows
        // so we want to read from all the columns in the
        // row
        if (chan < 16) || ((32 <= chan) && (chan < 48)) {
            all_channels = &*ALL_WORDS;
            all_channels_set = &*ALL_WORDS_SET;
        // Or the other way around. Channels 16..32 and
        // 48..64 correspond to columns so we want to read
        // from all the rows in the column.
        } else {
            all_channels = &*ALL_BITS;
            all_channels_set = &*ALL_BITS_SET;
        }

        // Initiate a read operation get the address of the data to be...
        let mut chunk = self._read_slice_inner(chan, &mask, vidx!(-vread))?;

        // ... and finally withdraw voltage from the biasing channels
        self.ground_all_fast()?.execute()?;

        // Make an array to hold all the values of row/column
        let mut res: Vec<f32> = Vec::with_capacity(32);

        // Read the raw chunk of data
        let data = self.read_chunk(&mut chunk, &DataMode::All)?;

        // Convert adc values to current
        for chan in all_channels {

            if all_channels_set.contains(&chan) {
                res.push(data[*chan]);
            } else {
                res.push(f32::NAN);
            }
        }

        Ok(res)
    }

    /// Same as [`Instrument::read_slice_masked`] but returning an [`Array`][`ndarray::Array`]
    /// for numpy compatibility
    pub fn read_slice_masked_as_ndarray(&mut self, chan: usize, mask: &[usize], vread: f32) ->
        Result<Array<f32, Ix1>, ArC2Error> {

        let data = self.read_slice_masked(chan, mask, vread)?;
        Ok(Array::from(data))

    }

    /// Same as [`Instrument::read_slice`] but returning an [`Array`][`ndarray::Array`]
    /// for numpy compatibility
    pub fn read_slice_as_ndarray(&mut self, chan: usize, vread: f32) -> Result<Array<f32, Ix1>, ArC2Error> {

        let data = self.read_slice(chan, vread)?;

        Ok(Array::from(data))
    }

    /// Read all the available crosspoints at the specified voltage
    ///
    /// This function will read all available crosspoints on the array. This can be done
    /// either by high biasing the rows ([`BiasOrder::Rows`]) or columns ([`BiasOrder::Columns`]).
    /// The result is stored in linear vector (and not a 2D matrix) in blocks of 32 values
    /// in channel order. When order is `Columns` the low potential channels are `[16..32)`
    /// and `[48..64)` (wordlines). When order is `Rows` the low potential channels are
    /// `[0..16)` and `[32..48)` (bitlines). Function [`Instrument::read_slice()`] is
    /// applied for every one of the selected channels.
    pub fn read_all(&mut self, vread: f32, order: BiasOrder) -> Result<Vec<f32>, ArC2Error> {

        let mut results = Vec::with_capacity(32*32);

        let bias_channels = match order {
            BiasOrder::Rows => &*ALL_WORDS,
            BiasOrder::Columns => &*ALL_BITS
        };

        for chan in bias_channels {
            results.append(&mut self.read_slice(*chan, vread)?);
        }

        Ok(results)

    }

    /// Same as [`Instrument::read_all()`] but represented as a 2D [`Array`][`ndarray::Array`].
    pub fn read_all_as_ndarray(&mut self, vread: f32, order: BiasOrder) -> Result<Array<f32, Ix2>, ArC2Error> {

        let data = self.read_all(vread, order)?;

        // we know the size is correct so we can unwrap safely here
        Ok(Array::from_shape_vec((32, 32), data).unwrap())
    }

    /// Check if the FPGA FIFO buffer is busy. The FPGA's FIFO buffer is regarded as busy
    /// if it's not empty, which means all the instructions have not yet been consumed.
    /// This is crucial for operations that include long delays. The output buffer should
    /// _not_ be consumed before the FIFO has. Use [`Instrument::wait`] to wait until the
    /// buffer has been emptied.
    pub fn busy(&self) -> bool {

        // Check FIFO empty flag; on error assume it's busy; you shouldn't be reading
        // the output anyway.

        let _efm = self.efm.clone();
        let efm = _efm.lock().unwrap();
        let response = match efm.read_block(FIFOBUSYADDR, 1i32, BLFLAGS_R) {
            Ok(buf) => { pktdbg!(buf); buf[0] },
            Err(_) => { eprintln!("Error reading FIFO busy"); 0u8 }
        };

        // If value is 0x01 the FIFO is empty (and therefore NOT busy)
        response != 1u8
    }

    // Variant of busy for internal use when a lock to a bl::Device has already
    // been acquired
    fn __busy_no_lock(&self, efm: &bl::Device) -> bool {
        let response = match efm.read_block(FIFOBUSYADDR, 1i32, BLFLAGS_R) {
            Ok(buf) => { pktdbg!(buf); buf[0] },
            Err(_) => { eprintln!("Error reading FIFO busy"); 0u8 }
        };

        // If value is 0x01 the FIFO is empty (and therefore NOT busy)
        response != 1u8
    }


    /// Block until the instrument has executed its command buffer.
    pub fn wait(&self) {

        let mut counter: u64 = 0;
        let mut exponent: u32 = 0;

        while self.busy() {

            // limit maximum polling to 100 ms
            if exponent < 5 {
                if counter == 9 {
                    counter = 0;
                    exponent += 1;
                } else {
                    counter += 1;
                }
            }

            let wait = std::time::Duration::from_nanos(10u64.pow(exponent) * 1000u64);
            std::thread::sleep(wait);
        }
    }

    /// Wait until the value specified by the chunk is populated. This will
    /// be typically done at the end of a read procedure. The function will
    /// return once this is done and it is using a similar polling procedure
    /// as [`Instrument::wait()`].
    #[cfg(feature="flag_addresses")]
    fn wait_for_flag(&mut self, chunk: &Chunk) -> Result<(), ArC2Error> {

        let mut counter: u64 = 0;
        let mut exponent: u32 = 0;

        loop {
            if self.value_available(&chunk)? {
                return Ok(())
            }

            // limit maximum polling to 100 ms
            if exponent < 5 {
                if counter == 9 {
                    counter = 0;
                    exponent += 1;
                } else {
                    counter += 1;
                }
            }

            let wait = std::time::Duration::from_nanos(10u64.pow(exponent) * 1000u64);
            std::thread::sleep(wait);
        }

    }

    // Variant of wait for internal use when a lock to a bl::Device has already
    // been acquired
    fn __wait_no_lock(&self, efm: &bl::Device) {

        let mut counter: u64 = 0;
        let mut exponent: u32 = 0;

        while self.__busy_no_lock(efm) {

            // limit maximum polling to 100 ms
            if exponent < 5 {
                if counter == 9 {
                    counter = 0;
                    exponent += 1;
                } else {
                    counter += 1;
                }
            }

            let wait = std::time::Duration::from_nanos(10u64.pow(exponent) * 1000u64);
            std::thread::sleep(wait);
        }
    }

    /// Apply a pulse between the specified channels. A voltage of `-voltage/2` will be
    /// applied to the `low` channel `voltage/2` to the `high` channel. When `preset_state`
    /// is true the state of high speed drivers will be initialised before the actual pulsing
    /// sequence begins.
    pub fn pulse_one(&mut self, low: usize, high: usize, voltage: f32, nanos: u128, preset_state: bool)
        -> Result<&mut Self, ArC2Error> {

        // use the high speed driver for all pulses faster than 500 ms
        if nanos < 500_000_000u128 {
            self.pulse_one_fast(low, high, voltage, nanos, preset_state)?;
        } else {
            self.pulse_one_slow(low, high, voltage, nanos)?;
        }

        Ok(self)
    }

    /// Apply a pulse to all channels with `chan` as the low potential channel.
    ///
    /// If `chan` is between 0 and 15 or 32 and 47 (inclusive) this will correspond
    /// to a row pulse, otherwise it's a column pulse. When `preset_state`
    /// is true the state of high speed drivers will be initialised before the actual pulsing
    /// sequence begins.
    pub fn pulse_slice(&mut self, chan: usize, voltage: f32, nanos: u128, preset_state: bool) -> Result<&mut Self, ArC2Error> {

        // use the high speed driver for all pulses faster than 500 ms
        if nanos < 500_000_000u128 {
            self.pulse_slice_fast(chan, voltage, nanos, None, preset_state)?;
        } else {
            self.pulse_slice_slow(chan, voltage, nanos, None)?;
        }

        Ok(self)
    }

    /// Apply a pulse to all channels with `chan` as the low potential channel.
    ///
    /// If `chan` is between 0 and 15 or 32 and 47 (inclusive) this will correspond
    /// to a row pulse, otherwise it's a column pulse. When `preset_state`
    /// is true the state of high speed drivers will be initialised before the actual pulsing
    /// sequence begins.
    pub fn pulse_slice_masked(&mut self, chan: usize, mask: &[usize], voltage: f32, nanos: u128, preset_state: bool) -> Result<&mut Self, ArC2Error> {

        // use the high speed driver for all pulses faster than 500 ms
        if nanos < 500_000_000u128 {
            self.pulse_slice_fast(chan, voltage, nanos, Some(mask), preset_state)?;
        } else {
            self.pulse_slice_slow(chan, voltage, nanos, Some(mask))?;
        }

        Ok(self)


    }

    /// Setup the biasing channels for multi-channel pulsing. This function will setup
    /// the output DACs for pulsing of a single crosspoint. The `config` argument
    /// holds a list of bias pairs in the form of `(low ch, high ch, voltage)`.
    /// This function will set the high channel to `voltage/2` and the low channel
    /// to `-voltage/2` if `differential` is `true` otherwise it will apply `-voltage`
    /// to the low channel and 0.0 to the high. If the `high_speed` argument is true
    /// then the DACs will be setup for high-speed pulsing as required
    /// by the High Speed drivers. No delays are introduced here as this will be
    /// handled either by a standard Delay instruction or a High Speed timer.
    fn _setup_dacs_for_pulsing(&mut self, config: &[(usize, usize, f32)], high_speed: bool, differential: bool)
        -> Result<(), ArC2Error> {

        // (idx, low, high); as required by SetDAC::from_channels().
        let mut channels: Vec<(u16, u16, u16)> = Vec::with_capacity(config.len()*2usize);

        for conf in config {
            let (low_ch, high_ch, voltage) = (conf.0, conf.1, conf.2);

            if high_speed {
                if voltage > 0.0 {
                    if differential {
                        channels.push((high_ch as u16, vidx!(0.0), vidx!(voltage/2.0)));
                        channels.push((low_ch as u16, vidx!(-voltage/2.0), vidx!(0.0)));
                    } else {
                        channels.push((high_ch as u16, vidx!(0.0), vidx!(0.0)));
                        channels.push((low_ch as u16, vidx!(-voltage), vidx!(0.0)));
                    }
                } else {
                    if differential {
                        channels.push((high_ch as u16, vidx!(voltage/2.0), vidx!(0.0)));
                        channels.push((low_ch as u16, vidx!(0.0), vidx!(-voltage/2.0)));
                    } else {
                        channels.push((high_ch as u16, vidx!(0.0), vidx!(0.0)));
                        channels.push((low_ch as u16, vidx!(0.0), vidx!(-voltage)));
                    }
                }
            } else {
                if differential {
                    channels.push((high_ch as u16, vidx!(voltage/2.0), vidx!(voltage/2.0)));
                    channels.push((low_ch as u16, vidx!(-voltage/2.0), vidx!(-voltage/2.0)));
                } else {
                    channels.push((high_ch as u16, vidx!(0.0), vidx!(0.0)));
                    channels.push((low_ch as u16, vidx!(-voltage), vidx!(-voltage)));
                }
            }

        }

        let instrs = SetDAC::from_channels(&channels, (vidx!(0.0), vidx!(0.0)), false);
        for mut i in instrs {
            self.process(i.compile())?;
        }

        self.process(&*UPDATE_DAC)

    }

    /// Pulse a crosspoint using conventional biasing and delaying. This function
    /// will *NOT* automatically flush output.
    fn pulse_one_slow(&mut self, low: usize, high: usize, voltage: f32, nanos: u128)
        -> Result<&mut Self, ArC2Error> {

        // set high and low channels as HS drivers
        let mut bias_conf = ChannelConf::new();
        bias_conf.set(high, ChannelState::VoltArb);
        bias_conf.set(low, ChannelState::VoltArb);

        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);
        self.process(conf.compile())?;

        // setup a non-high speed differential pulsing scheme
        self._setup_dacs_for_pulsing(&[(low, high, voltage)], false, true)?;
        self.add_delay(nanos+30_000u128)?;

        Ok(self)

    }

    /// Pulse a crosspoint using the high speed drivers. This function will *NOT*
    /// automatically flush output.
    fn pulse_one_fast(&mut self, low: usize, high: usize, voltage: f32, nanos: u128, preset_state: bool)
        -> Result<&mut Self, ArC2Error> {

        // set high and low channels as HS drivers
        let mut bias_conf = ChannelConf::new();
        bias_conf.set(high, ChannelState::HiSpeed);
        bias_conf.set(low, ChannelState::HiSpeed);

        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);

        let mut timings: [u32; 8] = [0u32; 8];
        // Obviously these will clip if nanos > u32::MAX
        // but we shouldn't get there because the high speed
        // path is triggered for < 500e6 nanoseconds which
        // will always fit in a u32
        let cl_low = low/8;
        let cl_high = high/8;
        timings[cl_low] = nanos as u32;
        timings[cl_high] = nanos as u32;

        let mut hsconf = HSConfig::new(timings);

        let hs_clusters = ClusterMask::new_from_cluster_idx(&[cl_low as u8, cl_high as u8]);
        let inv_clusters = if voltage > 0.0 {
            ClusterMask::new_from_cluster_idx(&[cl_low as u8])
        } else if voltage < 0.0 {
            ClusterMask::new_from_cluster_idx(&[cl_high as u8])
        } else {
            ClusterMask::NONE
        };
        let cncl_clusters = ClusterMask::NONE;

        let pulse_attrs = PulseAttrs::new_with_params(hs_clusters,
            inv_clusters, cncl_clusters);

        let mut pulse_base = HSPulse::new_from_attrs(&pulse_attrs);
        let pulse = pulse_base.compile();

        if preset_state {
            self.process(&*ZERO_HS_TIMINGS)?;
            self.process(pulse)?;
        }

        // set channels as HS
        self.process(conf.compile())?;
        self._tia_state = TIAState::Open;
        // setup a high-speed differential pulsing scheme
        self._setup_dacs_for_pulsing(&[(low, high, voltage)], true, true)?;
        self.add_delay(30_000u128)?;
        // HS configuration
        self.process(hsconf.compile())?;
        // HS Pulse
        self.process(pulse)?;
        self.add_delay(nanos)?;

        Ok(self)

    }

    fn pulse_slice_slow(&mut self, chan: usize, voltage: f32, nanos: u128, mask: Option<&[usize]>) ->
        Result<&mut Self, ArC2Error> {

        let mut bias_conf = ChannelConf::new();

        // Check if there are specific channels requested (`mask`). If not
        // just used all words/bits depending on the low channel number.
        let bias_channels = match mask {
            Some(m) => m,
            None => if (chan < 16) || ((32 <= chan) && (chan < 48)) {
                &*ALL_WORDS
            } else {
                &*ALL_BITS
            }
        };

        let mut channel_pairs: Vec<(usize, usize, f32)> = Vec::with_capacity(32);

        // Set all the channels involved in this operation to VoltArb
        // first the low
        bias_conf.set(chan, ChannelState::VoltArb);
        for c in bias_channels {
            // and all the highs
            bias_conf.set(*c, ChannelState::VoltArb);
            // and also add the pair to the channel_pairs vec above
            channel_pairs.push((chan, *c, voltage));
        }

        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);
        self._amp_prep()?;
        self.process(conf.compile())?;

        // setup a non-high speed differential pulsing scheme
        self._setup_dacs_for_pulsing(&channel_pairs, false, true)?;
        self.add_delay(nanos+30_000u128)?;

        Ok(self)
    }

    fn pulse_slice_fast(&mut self, chan: usize, voltage: f32, nanos: u128, mask: Option<&[usize]>, preset_state:bool) ->
        Result<&mut Self, ArC2Error> {

        let mut bias_conf = ChannelConf::new();

        // Check if there are specific channels requested (`mask`). If not
        // just used all words/bits depending on the low channel number.
        let bias_channels = match mask {
            Some(m) => m,
            None => if (chan < 16) || ((32 <= chan) && (chan < 48)) {
                &*ALL_WORDS
            } else {
                &*ALL_BITS
            }
        };

        // (low/high) pulsing pairs
        let mut channel_pairs: Vec<(usize, usize, f32)> = Vec::with_capacity(32);

        // timings for the high speed clusters
        let mut timings: [u32; 8] = [0u32; 8];

        // high speed cluster mask
        let mut hs_clusters = ClusterMask::NONE;
        // inversion cluster mask
        let mut inv_clusters = ClusterMask::NONE;
        // cancel cluster mask
        let cncl_clusters = ClusterMask::NONE;

        // Set all the channels involved in this operation to High Speed
        // first the low
        bias_conf.set(chan, ChannelState::HiSpeed);
        // Set the timing of the cluster where the low channel belongs to
        timings[chan/8] = nanos as u32;
        // and also mark it as high speed
        hs_clusters |= HSCLUSTERMAP[chan/8];

        // check where to set the inversion bit, if voltage > 0
        // this should be done on the low channel (here) otherwise
        // it will have to be done for the high channels (in the
        // for loop below).
        let inv_on_high = if voltage > 0.0 {
            inv_clusters |= HSCLUSTERMAP[chan/8];
            false
        } else { true };

        for c in bias_channels {
            // and all the highs
            bias_conf.set(*c, ChannelState::HiSpeed);
            // add the (low, high) pair to the vec
            channel_pairs.push((chan, *c, voltage));
            // IMPORTANT! This assumes differential pulsing. If
            // not then nanos should be set to 0 and
            // `_setup_dacs_for_pulsing` should be called with
            // differential set to false.
            timings[*c/8] = nanos as u32;
            // and mark the cluster as high speed
            hs_clusters |= HSCLUSTERMAP[*c/8];

            // in negative voltages the inversion bit must be
            // set on the high channels
            if inv_on_high {
                inv_clusters |= HSCLUSTERMAP[*c/8];
            }
        }

        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);

        let mut hsconf = HSConfig::new(timings);

        let pulse_attrs = PulseAttrs::new_with_params(hs_clusters,
            inv_clusters, cncl_clusters);

        let mut pulse_base = HSPulse::new_from_attrs(&pulse_attrs);
        let pulse = pulse_base.compile();

        if preset_state {
            self.process(&*ZERO_HS_TIMINGS)?;
            self.process(pulse)?;
        }

        self.process(conf.compile())?;
        self._tia_state = TIAState::Open;
        // setup a high speed differential pulsing scheme
        // WARNING! If a non-differential pulse (last argument is `false`)
        // is used instead `timings` for high channels above should be
        // set to 0 ns.
        self._setup_dacs_for_pulsing(&channel_pairs, true, true)?;
        self.add_delay(30_000u128)?;

        self.process(hsconf.compile())?;
        self.process(pulse)?;
        self.add_delay(nanos)?;

        Ok(self)
    }

    /// Pulse all crosspoints at the specified voltage and pulse width
    ///
    /// This function will pulse available crosspoints on the array. This can be done
    /// either by high biasing the rows ([`BiasOrder::Rows`]) or columns ([`BiasOrder::Columns`]).
    pub fn pulse_all(&mut self, voltage: f32, nanos: u128, order: BiasOrder, preset_state: bool) -> Result<&mut Self, ArC2Error> {

        let bias_channels = match order {
            BiasOrder::Rows => &*ALL_WORDS,
            BiasOrder::Columns => &*ALL_BITS
        };

        if nanos < 500_000_000u128 {
            for chan in bias_channels {
               self.pulse_slice_fast(*chan, voltage, nanos, None, preset_state)?;
            }

        } else {
            for chan in bias_channels {
               self.pulse_slice_slow(*chan, voltage, nanos, None)?;
            }
        }

        Ok(self)

    }

    /// Pulse and immediately read a crosspoint. Semantics and arguments follow the same
    /// conventions as [`Instrument::read_one`] and [`Instrument::pulse_one'].
    pub fn pulseread_one(&mut self, low: usize, high: usize, vpulse: f32, nanos: u128,
        vread: f32) -> Result<f32, ArC2Error> {

        if nanos < 500_000_000u128 {
            // For fast pulse+read we need to use the fast variant of grounding which
            // essentially removes the bias without reverting to VoltArb. This is
            // immediately followed by the generic inner read method (which will
            // revert the channels to VoltArb eventually). Then the crosspoints
            // are grounded to finalise the operation.
            //
            // Presently there is no way to do pulse + read without reverting to
            // VoltArb and waiting for the ~120 us that comes with the AMP PRP
            // operation. This is a hardware limitation for the current revision
            // of the board. So pulse trains of pulse+read operations will never
            // be truly accurate as there will be a gap between the application
            // of the pulse and the readout operation. The only true way to do
            // pulse trains is by forgoing the intermediate reads (or read after
            // a batch of pulses). The way pulseread is structured is optimised
            // with this work case in mind.
            let mut chunk = self.pulse_one_fast(low, high, vpulse, nanos, true)?
                                .ground_all_fast()?
                                ._read_slice_inner(low, &[high], vidx!(-vread))?;
            self.ground_all_fast()? // we are already in VoltArb no need to AmpPrep again
                .execute()?;

            let data = self.read_chunk(&mut chunk, &DataMode::All)?;
            Ok(data[high])

        } else {
            // For the slow path the delays included are 3 order of magnitude
            // smaller than the indented pulse width so we're just doing a
            // regular pulse followed by a regular ground and a regular read.
            // At worst we'll lose 120 us in > 500 ms pulse; peanuts!
            self.pulse_one_slow(low, high, vpulse, nanos)?
                .ground_all()?;
            self.read_one(low, high, vread)
        }

    }

    /// Pulse and read a slice. Semantics and arguments follow the same conventions as
    /// [`Instrument::read_slice`] and [`Instrument::pulse_slice`];
    pub fn pulseread_slice(&mut self, chan: usize, vpulse: f32, nanos: u128, vread: f32) -> Result<Vec<f32>, ArC2Error> {

        let mode: DataMode;
        let channels: &Vec<usize>;

        if (chan < 16) || ((32 <= chan) && (chan < 48)) {
            channels = &*ALL_WORDS;
            mode = DataMode::Words;
        } else {
            channels = &*ALL_BITS;
            mode = DataMode::Bits;
        }

        if nanos < 500_000_000u128 {
            let mut chunk = self.pulse_slice_fast(chan, vpulse, nanos, None, true)?
                                .ground_all_fast()?
                                ._read_slice_inner(chan, channels, vidx!(-vread))?;
            self.ground_all_fast()?
                .execute()?;
            let res = self.read_chunk(&mut chunk, &mode);

            res
        } else {
            self.pulse_slice_slow(chan, vpulse, nanos, None)?
                .ground_all_fast()?;
            self.read_slice(chan, vread)
        }

    }
    /// Pulse and read a slice but returning an [`Array`][`ndarray::Array`] for
    /// numpy compatibility
    pub fn pulseread_slice_as_ndarray(&mut self, chan: usize, vpulse: f32,
        nanos: u128, vread: f32) -> Result<Array<f32, Ix1>, ArC2Error> {
        let data = self.pulseread_slice(chan, vpulse, nanos, vread)?;
        Ok(Array::from(data))
    }

    /// Pulse and immediately read all crosspoints. Semantics and arguments follow the same
    /// conventions as [`Instrument::read_all`] and [`Instrument::pulse_all'].
    pub fn pulseread_all(&mut self, vpulse: f32, nanos: u128, vread: f32, order: BiasOrder) -> Result<Vec<f32>, ArC2Error> {

        let mode: DataMode;
        let bias_channels: &Vec<usize>;
        let read_channels: &Vec<usize>;
        let mut result = Vec::with_capacity(32*32);

        match order {
            BiasOrder::Rows => {
                bias_channels = &*ALL_WORDS;
                read_channels = &*ALL_BITS;
                mode = DataMode::Bits;
            },
            BiasOrder::Columns => {
                bias_channels = &*ALL_BITS;
                read_channels = &*ALL_WORDS;
                mode = DataMode::Words;
            },
        };

        if nanos < 500_000_000u128 {
            let mut chunks: Vec<Chunk> = Vec::with_capacity(32);

            for chan in bias_channels {
                let chunk = self.pulse_slice_fast(*chan, vpulse, nanos, None, true)?
                                .ground_all_fast()?
                                ._read_slice_inner(*chan, read_channels, vidx!(-vread))?;
                self.ground_all_fast()?
                    .execute()?;

                chunks.push(chunk);

            }

            for mut chunk in chunks {
                let mut data = self.read_chunk(&mut chunk, &mode)?;
                result.append(&mut data);
            }

        } else {
            for chan in bias_channels {
                self.pulse_slice_slow(*chan, vpulse, nanos, None)?
                    .ground_all_fast()?;
                result.append(&mut self.read_slice(*chan, vread)?);
            }
        }

        Ok(result)

    }

    /// Pulse and read all crosspoints but returning an [`Array`][`ndarray::Array`]
    /// for numpy compatibility
    pub fn pulseread_all_as_ndarray(&mut self, vpulse: f32, nanos: u128,
        vread: f32, order: BiasOrder) -> Result<Array<f32, Ix2>, ArC2Error> {

        let data = self.pulseread_all(vpulse, nanos, vread, order)?;
        Ok(Array::from_shape_vec((32, 32), data).unwrap())
    }

    /// Pulse and read the specified high channels that have `chan` as the low potential channel.
    /// Semantics and arguments follow the same conventions as [`Instrument::read_slice_masked`]
    /// and [`Instrument::pulse_slice_masked`].
    pub fn pulseread_slice_masked(&mut self, chan: usize, mask: &[usize], vpulse: f32,
        nanos: u128, vread: f32) -> Result<Vec<f32>, ArC2Error> {

        let all_channels: &Vec<usize>;
        let all_channels_set: &HashSet<usize>;
        let mut res: Vec<f32>;

        if (chan < 16) || ((32 <= chan) && (chan < 48)) {
            all_channels = &*ALL_WORDS;
            all_channels_set = &*ALL_WORDS_SET;
        } else {
            all_channels = &*ALL_BITS;
            all_channels_set = &*ALL_BITS_SET;
        }

        if nanos < 500_000_000u128 {
            let mut chunk = self.pulse_slice_fast(chan, vpulse, nanos, Some(mask), true)?
                                .ground_all_fast()?
                                ._read_slice_inner(chan, &mask, vidx!(-vread))?;
            self.ground_all_fast()?
                .execute()?;

            res = Vec::with_capacity(32);
            let data = self.read_chunk(&mut chunk, &DataMode::All)?;

            for chan in all_channels {
                if all_channels_set.contains(&chan) {
                    res.push(data[*chan]);
                } else {
                    res.push(f32::NAN);
                }
            }

        } else {
            self.pulse_slice_slow(chan, vpulse, nanos, Some(mask))?
                .ground_all_fast()?;
            res = self.read_slice_masked(chan, mask, vread)?;
        }

        Ok(res)

    }

    /// Pulse and read the specified high channels that have `chan` as the low potential channel,
    /// returning an ndarray for numpy compatibility
    pub fn pulseread_slice_masked_as_ndarray(&mut self, chan: usize, mask: &[usize], vpulse: f32,
        nanos: u128, vread: f32) -> Result<Array<f32, Ix1>, ArC2Error> {
        let data = self.pulseread_slice_masked(chan, mask, vpulse, nanos, vread)?;
        Ok(Array::from(data))
    }

    /// Set control mode for daughterboards
    ///
    /// This can be set either to internal or header-controlled. For the 32NAA daughterboard
    /// the first scenario routes channels to the internal PLCC socket and the second to
    /// the on-board headers.
    pub fn set_control_mode(&mut self, mode: ControlMode) -> Result<&mut Self, ArC2Error> {

        let mut iologic = match mode {
            ControlMode::Internal => UpdateLogic::new(false, true),
            ControlMode::Header => UpdateLogic::new(true, true)
        };

        self.process(iologic.compile())?;

        Ok(self)
    }

    /// Configures the digital I/Os to the specified levels. The I/Os can only be
    /// configured as outputs for now, so the bitmask essentially switches the
    /// digital outputs as low/high.
    pub fn set_logic(&mut self, mask: &IOMask, enable: bool) -> Result<&mut Self, ArC2Error> {

        let mut instr = UpdateLogic::with_regs_output(&mask, enable);
        self.process(instr.compile())?;

        Ok(self)
    }

    pub fn generate_ramp(&mut self,
        low: usize, high: usize,
        vstart: f32, vstep: f32, vstop: f32,
        pw_nanos: u128, inter_nanos: u128,
        num_pulses: usize,
        read_at: ReadAt, read_after: ReadAfter) ->
        Result<&mut Self, ArC2Error> {


        // helper function for reads
        fn __do_read(slf: &mut Instrument, low: usize, high: usize, read_at: &ReadAt,
            bias_voltage: f32) -> Result<Chunk, ArC2Error> {

            let chunk = match read_at {
                ReadAt::Bias => {
                    slf._amp_prep()?
                       ._read_slice_inner(low, &[high], vidx!(-bias_voltage))?
                },
                ReadAt::Arb(arbv) => {
                    slf._amp_prep()?
                       ._read_slice_inner(low, &[high], vidx!(-arbv))?
                },
                // if ReadAt or ReadAfter is never no reads will ever be performed
                ReadAt::Never => {
                    unreachable!()
                }
            };

            Ok(chunk)
        }

        // helper function for interpulse waits
        fn __inter_wait(slf: &mut Instrument, nanos: u128) -> Result<(), ArC2Error> {

            // if there's no interpulse just exit quickly
            if nanos > 0u128 {
                slf.add_delay(nanos)?;
            }

            Ok(())
        }

        // Check if voltage ramp is described correctly
        if vstop < vstart && vstep >= 0.0 {
            return Err(ArC2Error::RampOperationError(vstart, vstop, vstep));
        }

        // make the voltage list
        let voltages = Array::range(vstart, vstop, vstep);

        let sender = self._sender.clone();

        for v in &voltages {

            // if num pulses is 0 then no pulsing will be done, only reads
            if num_pulses == 0 {
                // but only if we're actually reading at pulse. Obviously
                // using anything other than ReadAfter::Pulse when num_pulses is 0
                // makes no sense, but for consistency's sake we make sure that
                // the convention is followed even in this unusual scenario.
                match read_after {
                    ReadAfter::Pulse => {
                        let chunk = __do_read(self, low, high, &read_at, *v)?;
                        match sender.send(Some(chunk)) {
                            Ok(()) => {},
                            Err(err) => { return Err(ArC2Error::from(err)); }
                        }
                    },
                    _ => { eprintln!("RMP: read-only ramp without ReadAfter::Pulse!!"); }
                };

                continue;
            }

            for pidx in 0..num_pulses {

                if pw_nanos < 500_000_000u128 {
                    self.pulse_one_fast(low, high, *v, pw_nanos, true)?
                        .ground_all_fast()?;
                } else {
                    self.pulse_one_slow(low, high, *v, pw_nanos)?
                        .ground_all_fast()?;
                }

                // No reads are required; interpulse wait and loop to
                // the next pulse
                if read_after.is_never() || read_at.is_never() {
                    __inter_wait(self, inter_nanos)?;
                    continue;
                }

                match read_after {
                    // If reading after every pulse just follow through
                    ReadAfter::Pulse => { },
                    ReadAfter::Block => {
                        if pidx < num_pulses - 1 {
                            // not at the end of the block
                            continue;
                        }
                    },
                    // not at the end of the ramp yet, see below
                    // inter wait and move to next pulse
                    ReadAfter::Ramp => {
                        __inter_wait(self, inter_nanos)?;
                        continue;
                    },
                    // already covered
                    ReadAfter::Never => { unreachable!(); }
                };

                let chunk = __do_read(self, low, high, &read_at, *v)?;
                match sender.send(Some(chunk)) {
                    Ok(()) => {},
                    Err(err) => { return Err(ArC2Error::from(err)); }
                }

                // if there's interpulse remove bias and wait
                __inter_wait(self, inter_nanos)?;
            }
        }

        // if we are doing read after ramp, do it here as the ramp is finished now
        if read_after.is_at_ramp() {
            // take the last value of the voltage list
            let voltage = voltages.as_slice().unwrap().last().unwrap();
            let chunk = __do_read(self, low, high, &read_at, *voltage)?;
            match sender.send(Some(chunk)) {
                Ok(()) => {},
                Err(err) => { return Err(ArC2Error::from(err)); }
            }
        };

        Ok(self)
    }

    /// Read one block of values from the internal buffer
    pub fn pick_one(&mut self, mode: DataMode) -> Result<Option<Vec<f32>>, ArC2Error> {
        let _receiver = self._receiver.clone();
        let receiver = _receiver.lock().unwrap();

        let chunk_opt: Option<Chunk>;

        loop {
            match receiver.try_recv() {
                Ok(x) => {
                    chunk_opt = x;
                    break
                },
                Err(TryRecvError::Empty) => {
                    // The ordering of the OR clause here is important!
                    // If there's nothing on the receiver and an operation is
                    // running then there's no point checking if the device is
                    // busy by querying the memory. It most likely is!
                    if self._op_running.load(atomic::Ordering::Relaxed) || self.busy() {
                        continue
                    } else {
                        chunk_opt = None;
                        break;
                    }
                },
                Err(TryRecvError::Disconnected) => { chunk_opt = None; break; }
            }
        }

        if chunk_opt.is_some() {
            let mut chunk = chunk_opt.unwrap();

            #[cfg(feature="flag_addresses")]
            self.wait_for_flag(&chunk);

            match self.read_chunk(&mut chunk, &mode) {
                Ok(v) => {
                    return Ok(Some(v));

                },
                Err(e) => Err(e)
            }
        } else {
            Ok(None)
        }
    }

    /// Check if the value of the chunk is actually available
    #[cfg(feature="flag_addresses")]
    fn value_available(&mut self, chunk: &Chunk) -> Result<bool, ArC2Error> {
        let _efm = self.efm.clone();
        let efm = _efm.lock().unwrap();

        let data = efm.read_register(chunk.flag_addr())?;

        Ok(data == VALUEAVAILFLAG)

    }

}


impl Drop for Instrument {
    fn drop(&mut self) {
        if Arc::strong_count(&self.efm) == 1 {
            let _efm = &*self.efm;
            let efm = _efm.lock().unwrap();
            efm.close().unwrap();
        }
    }
}

