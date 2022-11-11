use std::{time, thread};
use std::collections::HashSet;
use std::sync::{RwLock, Arc, Mutex, atomic};
use std::sync::mpsc::{channel, Sender, Receiver, TryRecvError};

#[cfg(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64"))]
use beastlink as bl;

use thiserror::Error;
use spin_sleep;

use crate::instructions::*;
use crate::registers::{ChannelState, ChanMask, IOMask};
use crate::registers::{ChannelConf, PulseAttrs, ClusterMask};
use crate::registers::{IOEnable, AuxDACFn};
use crate::registers::consts::HSCLUSTERMAP;
use crate::memory::{MemMan, Chunk, MemoryError};

const EFM03_VID: u16 = 0x10f8;
const EFM03_PID: u16 = 0xc583;
const BASEADDR: u32 = 0x80000000;
const FIFOBUSYADDR: u32 = 0x80020000;
const WRITEDELAY: time::Duration = time::Duration::from_nanos(2_500_000);

#[cfg(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64"))]
const BLFLAGS_W: bl::Flags = bl::Flags::ConstAddress;
#[cfg(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64"))]
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
    #[cfg(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64"))]
    #[error("FPGA Error: {0}")]
    FPGAError(#[from] bl::BLError),
    /// Memory management error
    #[error("Memory error: {0}")]
    MemoryError(#[from] MemoryError),
    #[error("Instruction Error: {0}")]
    InstructionError(#[from] InstructionError),
    /// Invalid Device ID or device not existing
    #[error("Invalid ArC2 device id: {0}")]
    InvalidID(i32),
    /// Invalid ramp parameters
    #[error("Ramp operation error: Vstart {0}, Vstop {1}, Vstep {2}")]
    RampOperationError(f32, f32, f32),
    /// Output buffer access error
    #[error("Cannot store address {0} to output buffer")]
    OutputBufferError(i64),
    /// Invalid HS timing combination
    #[error("HS Channel {0} has no timing associated on cluster {1}")]
    HSClusterTimingError(usize, usize),
    /// Invalid HS Duration
    #[error("HS Channel pulse width {0} too long")]
    /// Attempting to change a previously set polarity bit
    HSDurationError(usize),
    #[error("Attempting to toggle a previously set HS polarity bit")]
    HSPolarityBitError(),
    /// Unsupported platform
    #[error("Hardware functionality unavailable on this platform")]
    PlatformUnsupported(),
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


/// Type of data to retrieve from a memory read
#[derive(Clone)]
pub enum ReadType {
    Current,
    Voltage
}


/// Exit condition for long running processes
#[derive(Clone)]
pub enum WaitFor {
    Time(std::time::Duration),
    Iterations(usize)
}

impl WaitFor {
    /// Unwrap the inner [`Duration`][`std::time::Duration`]. This will
    /// panic if used with on an improper variant.
    pub fn time(self) -> std::time::Duration {
        if let WaitFor::Time(t) = self { t } else { panic!("No Time variant") }
    }

    /// Unwrap the inner number of [`WaitFor::Pulses`][`std::time::Duration`].
    /// This will panic if used with on an improper variant.
    pub fn iterations(self) -> usize {
        if let WaitFor::Iterations(i) = self { i } else { panic!("No Iterations variant") }
    }
}


/// ArC2 entry level object
///
/// `Instrument` implements the frontend for the ArC2. Its role is essentially
/// to process instructions and read back results. ArC2 processes instructions
/// either in _immediate_ mode or _retained_ mode. In the first case instructions
/// are written to the tool as soon they are issued. In retained mode
/// instructions will be gathered into a buffer which **must** be flushed
/// explicitly for them to have any effect.
#[cfg(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64"))]
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

    _tia_state: ChanMask
}

/// Find available device IDs.
///
/// This function will enumerate all available boards and return an array
/// with the IDs of all found devices. This can be passed on to
/// [`Instrument::open()`] to connect to a specific device.
#[cfg(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64"))]
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

#[cfg(not(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64")))]
pub fn find_ids() -> Result<Vec<i32>, ArC2Error> {
    Err(ArC2Error::PlatformUnsupported())
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

/// Convert a raw ADC value to a voltage reading
fn _adc_to_voltage(val: u32) -> f32 {

    let mut bytes = val.to_be_bytes();

    let range = bytes[0];

    if range == 0x01 {
        // channels hasn't been used
        return f32::NAN;
    }

    // replace the first 8 bytes with 0 to simplify things
    bytes[0] = 0;

    // and reconstruct the ADC output
    let uval = i32::from_be_bytes(bytes);

    let val: f32;
    let res: f32;

    // same as in _adc_to_current check if the ADC has
    // overflowed and subtract 2^{18}
    if uval > 2i32.pow(17) {
        val = (uval - 2i32.pow(18)) as f32;
    } else {
        val = uval as f32;
    }

    if range == 0x81 || range == 0x82 || range == 0x84 || range == 0x88 {
        20.48 * (val / 2.0f32.powf(18.0))
    } else if range == 0x90 || range == 0xa0 || range == 0xc0 {
        10.24 * (val / 2.0f32.powf(18.0))
    } else {
        // unknown range
        f32::NAN
    }


}


#[cfg(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64"))]
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
                _tia_state: ChanMask::all(),

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
        instr._amp_prep(None)?;
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
                    spin_sleep::sleep(WRITEDELAY);
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

    #[cfg(not(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64")))]
    pub fn execute(&mut self) -> Result<&mut Self, ArC2Error> {
        Err(ArC2Error::PlatformUnsupported())
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
    fn read_chunk(&self, chunk: &mut Chunk, mode: &DataMode, rtype: &ReadType) -> Result<Vec<f32>, ArC2Error> {



        let ret: Vec<f32> = match rtype {
            ReadType::Current => {
                match mode {
                    DataMode::Words => self.word_currents_from_address(chunk.addr())?,
                    DataMode::Bits => self.bit_currents_from_address(chunk.addr())?,
                    DataMode::All => {
                        self.currents_from_address(chunk.addr(), &ALL_CHANS)?
                    }
                }
            },
            ReadType::Voltage => {
                match mode {
                    DataMode::Words => self.word_voltages_from_address(chunk.addr())?,
                    DataMode::Bits => self.bit_voltages_from_address(chunk.addr())?,
                    DataMode::All => {
                        self.voltages_from_address(chunk.addr(), &ALL_CHANS)?
                    }
                }
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
        self.connect_to_gnd(&[])?;
        self.process(&*CHAN_FLOAT_ALL)?;
        self._tia_state = ChanMask::all();

        Ok(self)
    }

    /// Ground all channels reverting them to VoltArb
    pub fn ground_all(&mut self) -> Result<&mut Self, ArC2Error> {
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(30_000u128)?;
        self._amp_prep(None)?;
        self.process(&*CHAN_ARB_ALL)?;

        Ok(self)
    }

    /// Ground specified channels reverting them to VoltArb
    pub fn ground_slice(&mut self, chans: &[usize]) -> Result<&mut Self, ArC2Error> {

        let input: Vec<(u16, u16, u16)> = chans.iter()
                                               .map(|c| (*c as u16, vidx!(0.0), vidx!(0.0)))
                                               .collect();

        let (mut upch, setdacs) = SetDAC::from_channels(&input, None,
            &ChannelState::VoltArb, &ChannelState::Maintain)?;

        for mut s in setdacs {
            self.process(s.compile())?;
        }

        self.add_delay(30_000u128)?;
        self._amp_prep(Some(chans))?;
        // MOD CH should come after amp prep
        self.process(upch.compile())?;

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
    fn _amp_prep(&mut self, chans: Option<&[usize]>) -> Result<&mut Self, ArC2Error> {

            let common = &self._tia_state & &(if chans.is_some() {
                ChanMask::from_channels(chans.unwrap())
            } else { ChanMask::all() });

            // Only prepare channels that are common in chans
            // and self._tia_state
            let mut amp_prep = AmpPrep::new(&common);

            // this sets the channels in common as 0
            // essentially this means
            // self._tia_state = self._tia_state ^ common
            // was -> self._tia_state = TIAState::Closed;
            self._tia_state = &self._tia_state ^ &common;
            // set DACs for common channels to 0V and connect to GND
            // was -> self.ground_all_fast()?;
            self.ground_slice_fast(&common.channels())?;
            // was -> self.connect_to_gnd(&ALL_CHANS)?;
            self.connect_to_gnd(&common.channels())?;

            // AMP PRP
            self.process(amp_prep.compile())?;
            self.add_delay(100_000u128)?;
            // and disconnect GND before moving on
            self.connect_to_gnd(&[])

    }

    /// Set all DACs to ground maintaining current channel state
    pub fn ground_all_fast(&mut self) -> Result<&mut Self, ArC2Error> {
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(30_000u128)?;

        Ok(self)
    }

    /// Ground specified channels maintaining current channel state
    pub fn ground_slice_fast(&mut self, chans: &[usize]) -> Result<&mut Self, ArC2Error> {

        let input: Vec<(u16, u16, u16)> = chans.iter()
                                               .map(|c| (*c as u16, vidx!(0.0), vidx!(0.0)))
                                               .collect();
        // we don't care about the update channel instructions,
        // just setting the DACs to proper voltages, the `Maintain`s here
        // are not really needed either as they are only associated with
        // the UP CH instruction we are ignoring
        let (_, setdacs) = SetDAC::from_channels(&input, None,
            &ChannelState::Maintain, &ChannelState::Maintain)?;

        for mut s in setdacs {
            self.process(s.compile())?;
        }

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

    /// Retrieve all channel voltages from specific address segment. This function will
    /// always return a 64-element vector. Non-selected channels will be replaced by
    /// f32::NAN. The function will panic if a channel number exceeding the highest
    /// channel (presently 63) is provided or if an invalid base address is selected.
    /// Base address must be a multiple of 256.
    pub fn voltages_from_address(&self, addr: u32, chans: &[usize]) -> Result<Vec<f32>, ArC2Error> {

        if addr % 256 != 0 {
            panic!("Attempted to read voltages from invalid base address");
        }

        let data = self.read_raw(addr)?;
        let mut result: Vec<f32> = vec![f32::NAN; 64];

        // Assemble the number from the 4 neighbouring bytes
        for chan in chans {
            let val: u32 = u32::from_le_bytes([data[4*chan+0], data[4*chan+1],
                data[4*chan+2], data[4*chan+3]]);

            if chan % 2 == 0 {
                result[*chan] = _adc_to_voltage(val);
            } else {
                result[*chan] = -1.0*_adc_to_voltage(val);
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

    /// Retrieve all wordline voltages from specific address segment. This function will
    /// always return a 32-element vector. The function will panic if an invalid base
    /// address is provided. Base address must be a multiple of 256.
    pub fn word_voltages_from_address(&self, addr: u32) -> Result<Vec<f32>, ArC2Error> {

        if addr % 256 != 0 {
            panic!("Attempted to read voltages from invalid base address");
        }

        let data = self.read_raw(addr)?;
        let mut result: Vec<f32> = Vec::with_capacity(32);

        for chan in &*ALL_WORDS {
            let val: u32 = u32::from_le_bytes([data[4*chan+0], data[4*chan+1],
                data[4*chan+2], data[4*chan+3]]);

            if chan % 2 == 0 {
                result.push(_adc_to_voltage(val));
            } else {
                result.push(-1.0*_adc_to_voltage(val));
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

    /// Retrieve all bitline voltages from specific address segment. This function will
    /// always return a 32-element vector.
    pub fn bit_voltages_from_address(&self, addr: u32) -> Result<Vec<f32>, ArC2Error> {

        if addr % 256 != 0 {
            panic!("Attempted to read voltages from invalid base address");
        }

        let data = self.read_raw(addr)?;
        let mut result: Vec<f32> = Vec::with_capacity(32);

        for chan in &*ALL_BITS {
            let val: u32 = u32::from_le_bytes([data[4*chan+0], data[4*chan+1],
                data[4*chan+2], data[4*chan+3]]);

            if chan % 2 == 0 {
                result.push(_adc_to_voltage(val));
            } else {
                result.push(-1.0*_adc_to_voltage(val));
            }
        }

        Ok(result)
    }


    /// Set the specified channels as Open effectively disconnecting them from the DACs.
    /// Typically this is done when chaining into a new channel configuration. For instance
    /// a common biasing scenario is to bias specific channels at a specific voltage
    /// then float some (or all) of the unselected channels. This would translate into the
    /// following code
    ///
    /// ```no_run
    /// use libarc2::{Instrument};
    /// # use libarc2::ArC2Error;
    ///
    /// # fn main() -> Result<(), ArC2Error> {
    ///
    /// let mut arc2 = Instrument::open_with_fw(0, "fw.bin", true).unwrap();
    ///
    /// // This will float all channels then bias only the selected ones, in this case
    /// // channels 7 and 19 at 2.0 and 1.5 volts respectively
    ///
    /// arc2.connect_to_gnd(&[])? // clear all grounds
    ///     .open_channels(&(0..64).collect::<Vec<usize>>())? // set all channels to open;
    ///                                                       // effectively floating them
    ///     .config_channels(&[(7, 2.0), (19, 1.5)], None)?;  // bias selected channels
    /// # Ok(())
    /// # }
    ///
    /// ```
    pub fn open_channels(&mut self, input: &[usize]) -> Result<&mut Self, ArC2Error> {

        let mut chanconf = ChannelConf::new();

        for ch in input {
            chanconf.set(*ch, ChannelState::Open);
        }

        let mut upch = UpdateChannel::from_regs_default_source(&chanconf);
        self.process(upch.compile())?;

        Ok(self)
    }

    /// Configure channels at specified voltages. Argument `voltages` expects an
    /// array of tuples following the format `(channel, voltage)`.
    /// If `base` is not `None` the channels *not* included in `voltages` will be
    /// set at the specified voltage, otherwise they will remain at whatever
    /// state they were previously configured.
    /// For instance to set channels 7, 8 and 9 at 1.0, 1.5 and 2.0 V respectively
    /// you would do the following
    ///
    /// ```no_run
    /// use libarc2::{Instrument};
    /// # use libarc2::ArC2Error;
    ///
    /// # fn main() -> Result<(), ArC2Error> {
    /// let mut arc2 = Instrument::open_with_fw(0, "fw.bin", true).unwrap();
    ///
    /// let input: Vec<(u16, f32)> = vec![(7, 1.0), (8, 1.5), (9, 2.0)];
    /// // Setup channels 7, 8 and 9, set others to 0.0 V
    /// arc2.config_channels(&input, Some(0.0))?;
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// A common scenario is to float unselected channels while biasing the
    /// selected ones.
    ///
    /// ```no_run
    /// use libarc2::{Instrument};
    /// # use libarc2::ArC2Error;
    ///
    /// # fn main() -> Result<(), ArC2Error> {
    /// let mut arc2 = Instrument::open_with_fw(0, "fw.bin", true).unwrap();
    /// let all_chans = (0..64).collect::<Vec<usize>>();
    ///
    /// let input: Vec<(u16, f32)> = vec![(7, 1.0), (8, 1.5), (9, 2.0)];
    ///
    /// arc2.connect_to_gnd(&[])?
    ///     .open_channels(&all_chans)?
    ///     .config_channels(&input, None)?;
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// If you want the unselected channels to be set at 400 mV, you
    /// would do instead
    ///
    /// ```no_run
    /// use libarc2::{Instrument};
    /// # use libarc2::ArC2Error;
    ///
    /// # fn main() -> Result<(), ArC2Error> {
    /// let mut arc2 = Instrument::open_with_fw(0, "fw.bin", true).unwrap();
    /// let all_chans = (0..64).collect::<Vec<usize>>();
    ///
    /// let input: Vec<(u16, f32)> = vec![(7, 1.0), (8, 1.5), (9, 2.0)];
    ///
    /// arc2.connect_to_gnd(&[])?
    ///     .open_channels(&all_chans)?
    ///     .config_channels(&input, Some(0.4))?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn config_channels(&mut self, voltages: &[(u16, f32)], base: Option<f32>)
        -> Result<&mut Self, ArC2Error> {

        // If `base` is Some(...) convert it into a suitable pair of u16s
        let base_voltage = base.and_then(|x| Some((vidx!(x), vidx!(x))));

        let mut input: Vec<(u16, u16, u16)> = Vec::with_capacity(voltages.len());

        for item in voltages {
            // item.0 is the channel number
            input.push((item.0, vidx!(item.1), vidx!(item.1)));
            self._tia_state.set_enabled(item.0 as usize, true);
        }


        let unselected_state = if base_voltage.is_some() {
            ChannelState::VoltArb
        } else {
            ChannelState::Maintain
        };

        let (mut upch, instrs) = SetDAC::from_channels(&input, base_voltage,
                &ChannelState::VoltArb, &unselected_state)?;

        // AMP PRP the channels involved; If base voltage is supplied then *all* channels
        // will have to be AMP PRPed as they will be set to VoltArb initially
        if base.is_none() {
            self._amp_prep(Some(&input.iter().map(|c| (c.0 as usize)).collect::<Vec<_>>()))?;
        } else {
            self._amp_prep(None)?;
        }

        self.process(upch.compile())?;
        // this has now moved into the first loop see up
        //self._tia_state = TIAState::Open(ChanMask::all());

        for mut i in instrs {
            self.process(i.compile())?;
        }

        self.process(&*UPDATE_DAC)?;
        self.add_delay(30_000u128)?;

        Ok(self)
    }

    /// Configure the ArC2 auxiliary DACs. The AUX DACs manage signals
    /// required by the peripheral ArC2 circuitry as well as the
    /// arbitrary power supplies for DUTs. The sole argument is a
    /// list of tuples containing the specified DAC function and its
    /// corresponding voltage. See [`AuxDACFn`][`crate::registers::AuxDACFn`]
    /// for the available DAC functions that can be passed as arguments.
    pub fn config_aux_channels(&mut self, voltages: &[(AuxDACFn, f32)]) ->
        Result<&mut Self, ArC2Error> {

        let instrs = SetDAC::from_channels_aux(&voltages)?;

        for mut i in instrs {
            self.process(i.compile())?
        }

        self.process(&*UPDATE_DAC)?;
        self.add_delay(30_000u128)?;

        Ok(self)

    }

    /// Common read functionality with one low channel and several high channels.
    /// This function is guaranteed never to flush the output.
    fn _read_slice_inner(&mut self, low: usize, highs: &[usize], vread: u16)
        -> Result<Chunk, ArC2Error> {

        let zero: u16 = vidx!(0.0);

        // generate a list of dac settings, only one channel in this case
        let (mut upch, setdacs) = SetDAC::from_channels(&[(low as u16, vread, vread)],
            Some((zero, zero)), &ChannelState::VoltArb, &ChannelState::Open)?;

        // Is this necessary here?
        // Yes it is necessary, as the UP CH following this will transition
        // channels to VoltArb so they (both low and high channels) *must be*
        // AMP PRPed to allow for the transition to happen without transients.
        let mut chans_to_prep = highs.to_vec();
        chans_to_prep.push(low);
        // This actually needs to AMP PRP all channels as a base voltage is applied
        // above so all channels will be set to VoltArb
        //self._amp_prep(Some(&chans_to_prep))?;
        self._amp_prep(None)?;
        self.process(upch.compile())?;
        // was -> self._tia_state = TIAState::Open(ChanMask::all());
        self._tia_state.set_enabled(low, true);
        self._tia_state.set_channels_enabled(highs, true);
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

        let chunk = self.make_chunk()?;

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

    /// Do an open current measurement along the specified channels. No channel setup is
    /// done before the actual current measurement. If required this should be done with
    /// [`Instrument::config_channels`]. Setting `ground` to `true` will ground the channels
    /// after the measurement has gone through.
    pub fn read_slice_open(&mut self, highs: &[usize], ground: bool) -> Result<Vec<f32>, ArC2Error> {

        self._amp_prep(Some(&highs))?;

        let mut channelconf = UpdateChannel::from_regs_global_state(ChannelState::VoltArb);
        self.process(channelconf.compile())?;

        let mut adcmask = ChanMask::new();

        for chan in highs {
            adcmask.set_enabled(*chan, true);
        }

        let mut chunk = self.make_chunk()?;

        #[cfg(feature="zero_before_write")]
        match self._zero_chunk(&chunk) {
            Ok(()) => {},
            Err(err) => { eprintln!("Zeroing chunk at {} failed: {}", chunk.addr(), err) }
        };

        let mut currentread = CurrentRead::new(&adcmask, chunk.addr(),
            chunk.flag_addr(), VALUEAVAILFLAG);
        self.process(currentread.compile())?;
        self.add_delay(1_000u128)?;

        if ground {
            self.ground_slice_fast(&highs)?.execute()?;
        } else {
            self.execute()?;
        }

        self.wait();

        let res = self.read_chunk(&mut chunk, &DataMode::All, &ReadType::Current)?;

        Ok(res)
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
        self.wait();

        // Read the requested chunk...
        let res = self.read_chunk(&mut chunk, &DataMode::All, &ReadType::Current)?;

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
            self.wait();
            res = self.read_chunk(&mut chunk, &DataMode::Words, &ReadType::Current)?;
        // Or the other way around. Channels 16..32 and
        // 48..64 correspond to columns so we want to read
        // from all the rows in the column.
        } else {
            // Initiate a read operation get the address of the data to be...
            chunk = self._read_slice_inner(chan, &*ALL_BITS, vidx!(-vread))?;
            // ... and finally withdraw voltage from the biasing channels
            self.ground_all_fast()?.execute()?;
            self.wait();
            res = self.read_chunk(&mut chunk, &DataMode::Bits, &ReadType::Current)?;
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
        self.wait();

        // Make an array to hold all the values of row/column
        let mut res: Vec<f32> = Vec::with_capacity(32);

        // Read the raw chunk of data
        let data = self.read_chunk(&mut chunk, &DataMode::All, &ReadType::Current)?;

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


    /// Do a voltage read on all the selected channels
    ///
    /// This function will read the voltage on all specified channels and return their
    /// values in ascending channel order. Set `avg` to `true` to enable averaging
    /// but that will increase readout time from 10 to 320 μs
    pub fn vread_channels(&mut self, uchans: &[usize], avg: bool) -> Result<Vec<f32>, ArC2Error> {

        // first sort the channels
        let mut chans = uchans.to_vec();
        chans.sort();

        // Create a new mask and populate it with the specified channels
        let mut mask = ChanMask::none();
        for c in &chans {
            mask.set_enabled(*c, true);
        }

        let mut chunk = self.make_chunk()?;

        let mut results: Vec<f32> = Vec::with_capacity(chans.len());

        let mut voltageread = VoltageRead::new(&mask, avg, chunk.addr(),
            chunk.flag_addr(), VALUEAVAILFLAG);
        self.process(voltageread.compile())?;

        if avg {
            self.add_delay(320_000)?;
        } else {
            self.add_delay(10_000)?;
        }

        self.execute()?;
        self.wait();

        let res = self.read_chunk(&mut chunk, &DataMode::All, &ReadType::Voltage)?;

        for c in &chans {
            results.push(res[*c]);
        }

        Ok(results)

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
    pub fn pulse_one(&mut self, low: usize, high: usize, voltage: f32, nanos: u128)
        -> Result<&mut Self, ArC2Error> {

        // use the high speed driver for all pulses faster than 500 ms
        if nanos < 500_000_000u128 {
            self.pulse_one_fast(low, high, voltage, nanos)?;
        } else {
            self.pulse_one_slow(low, high, voltage, nanos)?;
        }

        Ok(self)
    }

    /// Apply a pulse to all channels with `chan` as the low potential channel.
    ///
    /// If `chan` is between 0 and 15 or 32 and 47 (inclusive) this will correspond
    /// to a row pulse, otherwise it's a column pulse.
    pub fn pulse_slice(&mut self, chan: usize, voltage: f32, nanos: u128) -> Result<&mut Self, ArC2Error> {

        // use the high speed driver for all pulses faster than 500 ms
        if nanos < 500_000_000u128 {
            self.pulse_slice_fast(chan, voltage, nanos, None)?;
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
    pub fn pulse_slice_masked(&mut self, chan: usize, mask: &[usize], voltage: f32, nanos: u128) -> Result<&mut Self, ArC2Error> {

        // use the high speed driver for all pulses faster than 500 ms
        if nanos < 500_000_000u128 {
            self.pulse_slice_fast(chan, voltage, nanos, Some(mask))?;
        } else {
            self.pulse_slice_slow(chan, voltage, nanos, Some(mask))?;
        }

        Ok(self)


    }

    /// Setup the biasing channels for multi-channel 2 terminal pulsing.  The `config` argument
    /// holds a list of bias pairs in the form of `(low ch, high ch, voltage)`.  This function will
    /// set the high channel to `voltage/2` and the low channel to `-voltage/2` if `differential`
    /// is `true` otherwise it will apply `-voltage` to the low channel and 0.0 to the high. If the
    /// `high_speed` argument is true then the DACs will be setup for high-speed pulsing as
    /// required by the High Speed drivers. No delays are introduced here as this will be handled
    /// either by a standard Delay instruction or a High Speed timer.
    fn _setup_dacs_2t_pulsing(&mut self, config: &[(usize, usize, f32)], high_speed: bool, differential: bool)
        -> Result<(), ArC2Error> {

        // (idx, low, high); as required by SetDAC::from_channels().
        let mut channels: Vec<(u16, u16, u16)> = Vec::with_capacity(config.len()*2usize);

        for conf in config {
            let (low_ch, high_ch, voltage) = (conf.0, conf.1, conf.2);

            if high_speed {

                // HiSpeed Channels should be marked as TIA::Open
                self._tia_state.set_channels_enabled(&[low_ch as usize, high_ch as usize], true);

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

        let (mut upch, instrs) = if high_speed {
            SetDAC::from_channels(&channels, Some((vidx!(0.0), vidx!(0.0))),
                &ChannelState::HiSpeed, &ChannelState::Open)?
        } else {
            SetDAC::from_channels(&channels, Some((vidx!(0.0), vidx!(0.0))),
                &ChannelState::VoltArb, &ChannelState::Open)?
        };
        self.process(upch.compile())?;

        // was -> self._tia_state = TIAState::Open(ChanMask::all());
        // this has now been replaced with selective channel opening during
        // the main iteration; see above!

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

        self._amp_prep(Some(&[low, high]))?;
        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);
        self.process(conf.compile())?;

        // setup a non-high speed differential pulsing scheme
        self._setup_dacs_2t_pulsing(&[(low, high, voltage)], false, true)?;
        self.add_delay(nanos+30_000u128)?;

        Ok(self)

    }

    /// Pulse a crosspoint using the high speed drivers. This function will *NOT*
    /// automatically flush output.
    fn pulse_one_fast(&mut self, low: usize, high: usize, voltage: f32, nanos: u128)
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

        // preload the HS drivers
        self.process(&*ZERO_HS_TIMINGS)?;
        self.process(pulse)?;

        // set channels as HS
        self.process(conf.compile())?;
        // was -> self._tia_state = TIAState::Open(ChanMask::all());
        self._tia_state.set_channels_enabled(&[low, high], true);
        // setup a high-speed differential pulsing scheme
        self._setup_dacs_2t_pulsing(&[(low, high, voltage)], true, true)?;
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
        let mut chans_to_prep: Vec<usize> = Vec::with_capacity(33); // all highs + the low

        // Set all the channels involved in this operation to VoltArb
        // first the low
        bias_conf.set(chan, ChannelState::VoltArb);
        // channels to AMP PRP
        chans_to_prep.push(chan);
        for c in bias_channels {
            // and all the highs
            bias_conf.set(*c, ChannelState::VoltArb);
            // and also add the pair to the channel_pairs vec above
            channel_pairs.push((chan, *c, voltage));
            chans_to_prep.push(*c);
        }

        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);
        //self._amp_prep(Some(&chans_to_prep))?;
        // this also needs to AMP PRP all channels because in slow operation
        // SetDAC::from_channels will set a voltage for the unselected
        // channels (the "base" voltage)
        self._amp_prep(None)?;
        self.process(conf.compile())?;

        // setup a non-high speed differential pulsing scheme
        self._setup_dacs_2t_pulsing(&channel_pairs, false, true)?;
        self.add_delay(nanos+30_000u128)?;

        Ok(self)
    }

    fn pulse_slice_fast(&mut self, chan: usize, voltage: f32, nanos: u128, mask: Option<&[usize]>) ->
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
            // `_setup_dacs_2t_pulsing` should be called with
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

        // Preload the HS drivers
        self.process(&*ZERO_HS_TIMINGS)?;
        self.process(pulse)?;

        self.process(conf.compile())?;
        // this is redundant -> self._tia_state = TIAState::Open(ChanMask::all());
        // setup a high speed differential pulsing scheme
        // WARNING! If a non-differential pulse (last argument is `false`)
        // is used instead `timings` for high channels above should be
        // set to 0 ns.
        self._setup_dacs_2t_pulsing(&channel_pairs, true, true)?;
        self.add_delay(30_000u128)?;

        self.process(hsconf.compile())?;
        self.process(pulse)?;
        self.add_delay(nanos)?;

        Ok(self)
    }

    /// Apply a sub-500 ms pulse to all specified channels
    ///
    /// This differs from [`Instrument::pulse_slice`] as it does not expect a low potential channel
    /// as the "receiving" end.  When `preset_state` is true the state of high speed drivers will
    /// be initialised before the actual pulsing sequence begins. Format of the `chans` slice
    /// is [(chan number, pulse voltage, normal voltage), ...] and `cl_nanos` contains the
    /// optional timings (pulse widths) per cluster. If cluster timing is `None` then
    /// channels of this cluster won't be pulsed at all. Be aware that the function will throw
    /// an error if a channel is included in the `chans` slice but the channel's cluster timing
    /// is set to `None`. Same restriction as other pulsing functions apply here as well, which
    /// means that 0 ns is valid timing, but anything between 1 ns and 40 ns will be clamped to
    /// 40 ns. No restriction for timings between 41 ns and 5×10⁸ ns.
    ///
    /// Please note that this function uses the high speed drivers of ArC2. If you want similar
    /// functionality with pulse widths > 500 ms you can do so using a sequence of
    /// [`config_channels`][`Instrument::config_channels`] and
    /// [`add_delay`][`Instrument::add_delay`] functions.
    pub fn pulse_slice_fast_open(&mut self, chans: &[(usize, f32, f32)],
        cl_nanos: &[Option<u128>; 8], preset_state: bool) -> Result<&mut Self, ArC2Error> {

        let mut bias_conf = ChannelConf::new();
        let mut timings: [u32; 8] = [0; 8];

        let mut hs_clusters = ClusterMask::NONE;
        let mut inv_clusters = ClusterMask::NONE;
        let mut cncl_clusters = ClusterMask::NONE;

        // Polarity bit tracking; initially no polarity bits are configured.
        // During the loop below the polarity bits per cluster will be
        // asserted (Some(true)) or deasserted (Some(false)).
        let mut polarity_track: Vec<Option<bool>> = vec![None; 8];

        // DAC configuration for active pulsing channels
        let mut input_active: Vec<(u16, u16, u16)> = Vec::with_capacity(chans.len());

        // this loop checks if the timings provided are consistent and
        // collects the channel voltages as a list of arguments for the
        // SetDAC::from_channels function.
        for (chan, active_v, normal_v) in chans {

            self._tia_state.set_enabled(*chan, true);

            // ensure that a timing has been provided in the corresponding cluster
            // index in `cl_nanos`. Reminder: cluster index is always floor(chan/8).
            // For instance if `chans` contains (7, 2.0, 0.0) a channel in cluster 0
            // (floor(7/8)) must have a timing configured. That essentially means that
            // `cl_nanos[0]` MUST NOT BE `None`.
            match cl_nanos[*chan/8] {
                // there is a timing configured; good!
                Some(t) => {

                    let active_v_raw = vidx!(active_v);
                    let normal_v_raw = vidx!(normal_v);

                    // Determine and set cluster timing
                    if t > 500_000_000u128 {
                        return Err(ArC2Error::HSDurationError(*chan));
                    } else {
                        // this cast is safe as 5×10⁸ will always fit in a u32
                        // and it will never reach this point otherwise
                        timings[*chan/8] = t as u32;
                    }
                    // toggle the cluster corresponding to channel as high-speed
                    hs_clusters |= HSCLUSTERMAP[chan/8];
                    // and mark the channel as high speed as well
                    bias_conf.set(*chan, ChannelState::HiSpeed);

                    // add the channel DAC-/DAC+ values to the list of arguments
                    // for the SetDAC::from_channels function.
                    // max channel is 63 (+aux) so casts to u16 here are safe
                    if active_v_raw <= normal_v_raw {
                        input_active.push((*chan as u16, active_v_raw, normal_v_raw ));
                    } else {
                        input_active.push((*chan as u16, normal_v_raw, active_v_raw ));
                    }

                    // Determine the polarity bit, that essentially checks if
                    // we are doing low → high → low transition (pol=0) or
                    // high → low → high transition (pol=1)
                    let polbithigh = active_v_raw <= normal_v_raw;

                    // Check if a polarity bit has been previously set
                    // for the cluster on which the channel is
                    match polarity_track[*chan/8] {
                        // If yes, check if it is the same
                        Some(b) => {
                            // If it's not we have two different polarities on the
                            // same DAC cluster which is an error
                            if b != polbithigh {
                                return Err(ArC2Error::HSPolarityBitError())
                            }
                        },
                        // If no polarity bit has been set for this cluster,
                        // toggle the corresponding cluster bit and track it
                        None => {
                            if polbithigh {
                                inv_clusters |= HSCLUSTERMAP[*chan/8];
                            }
                            polarity_track[*chan/8] = Some(polbithigh);
                        }
                    };

                    if !preset_state {
                        cncl_clusters |= HSCLUSTERMAP[*chan/8];
                    }

                },
                // a channel has been selected in the `chans` argument but no timing
                // has been provided for the corresponding cluster index in `cl_nanos`;
                // that's an error
                None => { return Err(ArC2Error::HSClusterTimingError(*chan, *chan/8)) }
            };
        }


        // Ready the UP CH and HS CNF instructions for the actual pulse
        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);
        let mut hsconf = HSConfig::new(timings);

        // PREP PULSE ATTRS (HS driver preloading)
        let prep_pulse_attrs = PulseAttrs::new_with_params(hs_clusters, inv_clusters,
            cncl_clusters);

        // ACTUAL PULSE ATTRS (cancel is always NONE here)
        let pulse_attrs = PulseAttrs::new_with_params(hs_clusters, inv_clusters,
            ClusterMask::NONE);

        let mut prep_pulse_base = HSPulse::new_from_attrs(&prep_pulse_attrs);
        let mut pulse_base = HSPulse::new_from_attrs(&pulse_attrs);
        let prep_pulse = prep_pulse_base.compile();
        let pulse = pulse_base.compile();

        // Preload the HS drivers HS CNF + HS PLS with 0 ns
        self.process(&*ZERO_HS_TIMINGS)?;
        self.process(prep_pulse)?;

        // set pulse channel configuration for the actual pulse
        self.process(conf.compile())?;
        // this was probably redundant -> self._tia_state = TIAState::Open(ChanMask::all());

        let (mut upch, instr_active) = SetDAC::from_channels(&input_active,
            Some((vidx!(0.0), vidx!(0.0))), &ChannelState::HiSpeed,
            &ChannelState::Open)?;

        self.process(upch.compile())?;
        // this is now done in the main processing loop; see above
        // self._tia_state = TIAState::Open(ChanMask::all());

        // SetDAC for the actual pulse
        for mut i in instr_active {
            self.process(i.compile())?;
        }

        // UP DAC
        self.process(&*UPDATE_DAC)?;
        self.add_delay(30_000u128)?;

        // HS CONF (actual)
        self.process(hsconf.compile())?;
        // HS PLS (actual)
        self.process(pulse)?;

        Ok(self)
    }

    /// Pulse all crosspoints at the specified voltage and pulse width
    ///
    /// This function will pulse available crosspoints on the array. This can be done
    /// either by high biasing the rows ([`BiasOrder::Rows`]) or columns ([`BiasOrder::Columns`]).
    pub fn pulse_all(&mut self, voltage: f32, nanos: u128, order: BiasOrder) -> Result<&mut Self, ArC2Error> {

        let bias_channels = match order {
            BiasOrder::Rows => &*ALL_WORDS,
            BiasOrder::Columns => &*ALL_BITS
        };

        if nanos < 500_000_000u128 {
            for chan in bias_channels {
               self.pulse_slice_fast(*chan, voltage, nanos, None)?;
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
            let mut chunk = self.pulse_one_fast(low, high, vpulse, nanos)?
                                .ground_all_fast()?
                                ._read_slice_inner(low, &[high], vidx!(-vread))?;
            self.ground_all_fast()? // we are already in VoltArb no need to AmpPrep again
                .execute()?;
            self.wait();

            let data = self.read_chunk(&mut chunk, &DataMode::All, &ReadType::Current)?;
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
            let mut chunk = self.pulse_slice_fast(chan, vpulse, nanos, None)?
                                .ground_all_fast()?
                                ._read_slice_inner(chan, channels, vidx!(-vread))?;
            self.ground_all_fast()?
                .execute()?;
            self.wait();
            let res = self.read_chunk(&mut chunk, &mode, &ReadType::Current);

            res
        } else {
            self.pulse_slice_slow(chan, vpulse, nanos, None)?
                .ground_all_fast()?;
            self.read_slice(chan, vread)
        }

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
                let chunk = self.pulse_slice_fast(*chan, vpulse, nanos, None)?
                                .ground_all_fast()?
                                ._read_slice_inner(*chan, read_channels, vidx!(-vread))?;
                self.ground_all_fast()?
                    .execute()?;
                self.wait();

                chunks.push(chunk);

            }

            for mut chunk in chunks {
                let mut data = self.read_chunk(&mut chunk, &mode, &ReadType::Current)?;
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
            let mut chunk = self.pulse_slice_fast(chan, vpulse, nanos, Some(mask))?
                                .ground_all_fast()?
                                ._read_slice_inner(chan, &mask, vidx!(-vread))?;
            self.ground_all_fast()?
                .execute()?;
            self.wait();

            res = Vec::with_capacity(32);
            let data = self.read_chunk(&mut chunk, &DataMode::All, &ReadType::Current)?;

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

    /// Set control mode for daughterboards
    ///
    /// This can be set either to internal or header-controlled. For the 32NAA daughterboard
    /// the first scenario routes channels to the internal PLCC socket and the second to
    /// the on-board headers.
    pub fn set_control_mode(&mut self, mode: ControlMode) -> Result<&mut Self, ArC2Error> {

        let mut iologic = match mode {
            ControlMode::Internal => UpdateLogic::new(false, true),
            ControlMode::Header =>  {
                let mut mask = IOMask::new();
                mask.set_enabled(0, true);

                let mut en = IOEnable::new();
                en.set_en(true);

                UpdateLogic::with_regs(&mask, &en)
            }
        };

        self.process(iologic.compile())?;

        Ok(self)
    }

    /// Configures the digital I/Os to the specified levels. The I/Os can only be
    /// configured as outputs for now, so the bitmask essentially switches the
    /// digital outputs as low/high.
    pub fn set_logic(&mut self, mask: &IOMask) -> Result<&mut Self, ArC2Error> {

        let mut instr = UpdateLogic::with_mask(&mask);
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
                    slf._amp_prep(Some(&[low, high]))?
                       ._read_slice_inner(low, &[high], vidx!(-bias_voltage))?
                },
                ReadAt::Arb(arbv) => {
                    slf._amp_prep(Some(&[low, high]))?
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

        // determine the number of steps
        let steps = f32::ceil((vstop - vstart)/vstep) as usize;

        let sender = self._sender.clone();

        for idx in 0..steps {

            let v = vstart + vstep*(idx as f32);

            // if num pulses is 0 then no pulsing will be done, only reads
            if num_pulses == 0 {
                // but only if we're actually reading at pulse. Obviously
                // using anything other than ReadAfter::Pulse when num_pulses is 0
                // makes no sense, but for consistency's sake we make sure that
                // the convention is followed even in this unusual scenario.
                match read_after {
                    ReadAfter::Pulse => {
                        let chunk = __do_read(self, low, high, &read_at, v)?;
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
                    self.pulse_one_fast(low, high, v, pw_nanos)?
                        .ground_all_fast()?;
                } else {
                    self.pulse_one_slow(low, high, v, pw_nanos)?
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

                let chunk = __do_read(self, low, high, &read_at, v)?;
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
            // take the last value of the voltage list, lists are always [start, stop)
            // so substract 1 from steps to get the last value
            let voltage = vstart + vstep*((steps-1) as f32);
            let chunk = __do_read(self, low, high, &read_at, voltage)?;
            match sender.send(Some(chunk)) {
                Ok(()) => {},
                Err(err) => { return Err(ArC2Error::from(err)); }
            }
        };

        Ok(self)
    }

    /// Perform a retention-like read train operation on a single cross-point at specific
    /// interpulse intervals.
    ///
    /// You can optionally pre-load the cross-point at a voltage but be aware that if it's
    /// different from `vread` an additional instruction will be generated after **every** read
    /// operation to set the voltage back at `preload` voltage so it's not recommended unless you
    /// know what you are doing. There are two common scenarios for `read_train`: (a) if you are
    /// trying to do a chronoamperometry measurement (bias a cross-point and measure the current at
    /// specific intervals) then the correct call is `read_train(low, high, vread, interpulse,
    /// Some(vread), WaitFor.Time(…)/WaitFor.Iterations(…))`; (b) if you are trying to do a
    /// retention-like measurement (read an otherwise grounded cross-point at specific intervals)
    /// then the correct call is `read_train(low, high, vread, interpulse, None,
    /// WaitFor.Time(…)/WaitFor.Iterations(…))`.
    // XXX: This needs better in-thread error handling
    pub fn read_train(&mut self, low: usize, high: usize, vread: f32, interpulse: u128,
        preload: Option<f32>, condition: WaitFor)
        -> Result<(), ArC2Error> {

        self._op_running.store(true, atomic::Ordering::Relaxed);
        self.reset_dacs()?;

        let now = time::Instant::now();
        let mut iter = 0;

        let mut slf = self.clone();
        let cond = condition.clone();

        std::thread::spawn(move || {
            let sender = slf._sender.clone();

            // if preloading, set the bias voltage, this will get
            // executed *before* the first execute, implicitly called by
            // the first _read_slice_inner of the loop
            match preload {
                Some(v) => {
                    let input: Vec<(u16, f32)> = vec![(low as u16, -v)];
                    slf.config_channels(&input, None).unwrap();
                }
                _ => {}
            };

            loop {
                let chunk = slf._read_slice_inner(low, &[high], vidx!(-vread)).unwrap();
                // if crosspoint is preloaded do not ground the lines, instead put it
                // back to the preloading voltage (if different from vread)
                match preload {
                    Some(v) => {
                        // unless preload and vread are similar (difference is less
                        // than 5 mV) put the correct bias back on the line
                        if (vread == v) || (vread - v).abs() > 5e-3 {
                            let input: Vec<(u16, f32)> = vec![(low as u16, -v)];
                            slf.config_channels(&input, None).unwrap()
                               .execute().unwrap();
                        }
                    }
                    None => { slf.ground_all_fast().unwrap(); }
                };
                if interpulse > 0u128 {
                    slf.add_delay(interpulse).unwrap();
                }
                slf.execute().unwrap();
                slf.wait();
                sender.send(Some(chunk)).unwrap();
                iter += 1;

                match cond {
                    WaitFor::Time(d) => {
                        if now.elapsed() >= d {
                            // ensure crosspoint is grounded if pre-loaded
                            match preload {
                                None => { slf.ground_all_fast().unwrap(); }
                                _ => {}
                            };
                            break;
                        }
                    },
                    WaitFor::Iterations(i) => {
                        if iter >= i {
                            // ensure crosspoint is grounded if pre-loaded
                            match preload {
                                None => { slf.ground_all_fast().unwrap(); }
                                _ => {}
                            };
                            break;
                        }
                    }
                }
            }
            slf._op_running.store(false, atomic::Ordering::Relaxed);
        });

        Ok(())

    }

    /// Read one block of values from the internal buffer
    pub fn pick_one(&mut self, mode: DataMode, rtype: ReadType) -> Result<Option<Vec<f32>>, ArC2Error> {
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

            match self.read_chunk(&mut chunk, &mode, &rtype) {
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


#[cfg(all(any(target_os = "windows", target_os = "linux"), target_arch = "x86_64"))]
impl Drop for Instrument {
    fn drop(&mut self) {
        if Arc::strong_count(&self.efm) == 1 {
            let _efm = &*self.efm;
            let efm = _efm.lock().unwrap();
            efm.close().unwrap();
        }
    }
}

