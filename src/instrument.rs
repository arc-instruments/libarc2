use std::{time, thread};
use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use beastlink as bl;
use ndarray::{Array, Ix1, Ix2};

use crate::instructions::*;
use crate::registers::{ChannelState, ChanMask};
use crate::registers::{ChannelConf, PulseAttrs, ClusterMask};
use crate::registers::consts::HSCLUSTERMAP;
use crate::memory::{MemMan, Chunk};

const EFM03_VID: u16 = 0x10f8;
const EFM03_PID: u16 = 0xc583;
const BASEADDR: u32 = 0x80000000;
const FIFOBUSYADDR: u32 = 0x80020000;
const WRITEDELAY: time::Duration = time::Duration::from_nanos(2_500_000);
const BLFLAGS_W: bl::Flags = bl::Flags::ConstAddress;
const BLFLAGS_R: bl::Flags = bl::Flags::NoFlags;
const INBUF: usize = 64*std::mem::size_of::<u32>();

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

/// ArC2 entry level object
///
/// `Instrument` implements the frontend for the ArC2. Its role is essentially
/// to process instructions and read back results. ArC2 processes instructions
/// either in _immediate_ mode or _retained_ mode. In the first case instructions
/// are written to the tool as soon they are issued. In retained mode
/// instructions will be gathered into a buffer which **must** be flushed
/// explicitly for them to have any effect.
pub struct Instrument {
    efm: bl::Device,
    instr_buffer: Option<Vec<u8>>,
    memman: Arc<Mutex<MemMan>>,
}

/// Find available device IDs.
///
/// This function will enumerate all available boards and return an array
/// with the IDs of all found devices. This can be passed on to
/// [`Instrument::open()`] to connect to a specific device.
pub fn find_ids() -> Result<Vec<i32>, String> {
    let max_id = match bl::enumerate(EFM03_VID, EFM03_PID) {
        Ok(v) => Ok(v),
        Err(err) => Err(format!("Could not enumerate devices: {}", err))
    }?;

    if max_id >= 0 {
        Ok((0..max_id).collect())
    } else {
        // max_id -1 usually indicates an error...
        Err(format!("Invalid max id: {}", max_id))
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
    pub fn open(id: i32, retained_mode: bool) -> Result<Instrument, String> {

        if !find_ids()?.contains(&id) {
            return Err(format!("No such device id: {}", id));
        }

        let buffer: Option<Vec<u8>>;

        // If in retained mode preallocate space for 512 instructions.
        // If not the instruction buffer is not used.
        if retained_mode {
            buffer = Some(Vec::with_capacity(512*9*std::mem::size_of::<u32>()));
        } else {
            buffer = None;
        }

        match bl::Device::open(id) {
            Ok(d) => Ok(Instrument {
                efm: d,
                instr_buffer: buffer,
                memman: Arc::new(Mutex::new(MemMan::new()))
            }),
            Err(err) => Err(format!("Could not open device: {}", err))
        }
    }

    /// Load an FPGA bitstream from a file.
    pub fn load_firmware(&self, path: &str) -> Result<(), String> {
        match self.efm.program_from_file(&path) {
            Ok(()) => { Ok(()) },
            Err(err) => { Err(format!("Could not program FPGA: {}", err)) }
        }
    }

    /// Open a new Instrument with a specified id and bitstream.
    /// This essentially combines [`Instrument::open()`] and
    /// [`Instrument::load_firmware()`].
    pub fn open_with_fw(id: i32, path: &str, retained_mode: bool) -> Result<Instrument, String> {
        let mut instr = Instrument::open(id, retained_mode)?;
        instr.load_firmware(&path)?;
        instr.process(&*RESET_DAC)?;
        instr.process(&*SET_3V3_LOGIC)?;
        instr.process(&*UPDATE_DAC)?;
        instr.add_delay(20_000u128)?;
        instr.execute()?;

        Ok(instr)
    }

    /// Process a compiled instruction
    pub fn process<T: Instruction>(&mut self, instr: &T) -> Result<(), String> {

        instrdbg!(instr);

        // convert the instruction into raw bytes
        let mut bytes = instr.to_bytevec();

        // If an instruction buffer is used write the bytes to the buffer
        // instead of directly to the tool.
        if let Some(buff) = &mut self.instr_buffer {

            // To avoid reallocation of the buffer flush the instruction
            // buffer if its length would go over capacity when the new
            // instruction is added.
            if buff.len() + bytes.len() > buff.capacity() {
                self.execute()?;
            }
        }

        if let Some(buff) = &mut self.instr_buffer {
            // buffer is guaranteed to be under capacity here
            buff.extend(bytes);
            return Ok(());
        }

        // Otherwise write directly to ArC2 (immediate)
        #[cfg(not(feature="dummy_writes"))]
        match self.efm.write_block(BASEADDR, &mut bytes, BLFLAGS_W) {

            Ok(()) => {
                thread::sleep(WRITEDELAY);
            },
            Err(err) => return Err(format!("Could not write buffer: {}", err))
        }

        #[cfg(feature="dummy_writes")]
        eprintln!("DW: {:?}", bytes);

        Ok(())
    }

    /// Zero an FPGA address chunk
    #[cfg(feature="zero_before_write")]
    fn _zero_chunk(&mut self, chunk: &Chunk) -> Result<(), String> {
        let mut zerobuf: [u8; INBUF] = [0u8; INBUF];
        let addr = chunk.addr();

        eprintln!("Trying to zero chunk");
        match self.efm.write_block(addr, &mut zerobuf, BLFLAGS_W) {
            Ok(()) => {

                #[cfg(feature="debug_packets")]
                eprintln!("ZERO: {:08x} → {:08x}", addr, (addr as usize)+INBUF-1);

                Ok(())
            },
            Err(err) => Err(format!("Error: {}; Could not zero address range {}", err, addr))
        }
    }

    /// Write all retained instructions to ArC2. Has no effect if
    /// `retained_mode` is `false` since all instructions are
    /// executed as they are issued.
    pub fn execute(&mut self) -> Result<&mut Self, String> {

        // If an instruction buffer is used empty it, otherwise do
        // nothing as all writes are immediates
        match self.instr_buffer {
            Some(ref mut buf) => {
                #[cfg(not(feature="dummy_writes"))] {
                    match self.efm.write_block(BASEADDR, buf, BLFLAGS_W) {
                        Ok(()) => {
                            thread::sleep(WRITEDELAY);
                            buf.clear();

                            {
                                #[cfg(feature="debug_packets")]
                                eprintln!("FLUSH");
                            }

                            Ok(self)
                        },
                        Err(err) => Err(format!("Could not write buffer: {}", err))
                    }
                }
                #[cfg(feature="dummy_writes")] {
                    eprintln!("DW: {:?}", buf);
                    Ok(self)
                }
            },
            None => Ok(self)
        }


    }

    /// Compiles and process an instruction
    pub fn compile_process<T: Instruction>(&mut self, instr: &mut T) -> Result<(), String> {

        self.process(instr.compile())

    }

    /// Add a delay to the FIFO command buffer
    pub fn add_delay(&mut self, nanos: u128) -> Result<&mut Self, String> {

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
    pub fn read_raw(&self, addr: u32) -> Result<Vec<u8>, String> {
        match self.efm.read_block(addr, INBUF as i32, BLFLAGS_R) {
            Ok(buf) => { pktdbg!(buf); Ok(buf) },
            Err(err) => Err(format!("Could not read block memory: {}", err))
        }
    }

    /// Reset all DACs on the tool. This will execute existing buffers.
    pub fn reset_dacs(&mut self) -> Result<&mut Self, String> {
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(20_000u128)?;
        self.execute()
    }

    /// Disconnect all channels
    pub fn float_all(&mut self) -> Result<&mut Self, String> {
        self.process(&*CHAN_FLOAT_ALL)?;
        Ok(self)
    }

    /// Ground all channels reverting them to VoltArb
    pub fn ground_all(&mut self) -> Result<&mut Self, String> {
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(20_000u128)?;
        self.process(&*PREP_AMP_ALL)?;
        self.add_delay(100_000u128)?;
        self.process(&*CHAN_ARB_ALL)?;

        Ok(self)
    }

    /// Set all DACs to ground maintaining current channel state
    pub fn ground_all_fast(&mut self) -> Result<&mut Self, String> {
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(20_000u128)?;

        Ok(self)
    }

    /// Set global 3.3 V logic level
    pub fn set_3v3_logic(&mut self) -> Result<&mut Self, String> {
        self.process(&*SET_3V3_LOGIC)?;
        self.load_dacs()
    }

    /// Update the DAC configuration
    pub fn load_dacs(&mut self) -> Result<&mut Self, String> {
        self.process(&*UPDATE_DAC)?;
        Ok(self)
    }

    /// Retrieve all channel currents from specific address segment. This function will
    /// always return a 64-element vector. Non-selected channels will be replaced by
    /// f32::NAN. The function will panic if a channel number exceeding the highest
    /// channel (presently 63) is provided or if an invalid base address is selected.
    /// Base address must be a multiple of 256.
    pub fn currents_from_address(&self, addr: u32, chans: &[usize]) -> Result<Vec<f32>, String> {

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
    pub fn word_currents_from_address(&self, addr: u32) -> Result<Vec<f32>, String> {

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
    pub fn bit_currents_from_address(&self, addr: u32) -> Result<Vec<f32>, String> {

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
        -> Result<Chunk, String> {

        let zero: u16 = vidx!(0.0);

        // generate a list of dac settings, only one channel in this case
        let setdacs = SetDAC::from_channels(&[(low as u16, vread, vread)], (zero, zero), false);

        // process them with the appropriate delay
        for mut instr in setdacs {
            self.process(instr.compile())?;
        }
        self.process(&*UPDATE_DAC)?;
        self.add_delay(20_000u128)?;

        // set all channels to Arbitrary Voltage
        let mut channelconf =
            UpdateChannel::from_regs_global_state(ChannelState::VoltArb);
        self.process(channelconf.compile())?;

        // Prepare the ADC mask for the readout
        let mut adcmask = ChanMask::new();

        for chan in highs {
            adcmask.set_enabled(*chan, true);
        }

        let memman = self.memman.clone();
        let mut manager = memman.lock().unwrap();
        let chunk = manager.alloc_chunk().unwrap();

        #[cfg(feature="zero_before_write")]
        match self._zero_chunk(&chunk) {
            Ok(()) => {},
            Err(err) => { eprintln!("Zeroing chunk at {} failed: {}", chunk.addr(), err) }
        };

        let mut currentread = CurrentRead::new(&adcmask, chunk.addr());
        self.process(currentread.compile())?;
        self.add_delay(1_000u128)?;

        // The read operation has been assembled, return back the readout address.
        Ok(chunk)
    }

    /// Perform a current read between the specified channels. A voltage
    /// of `-vread` will be applied to the `low` channel and current will
    /// be read from the `high` channel.
    pub fn read_one(&mut self, low: usize, high: usize, vread: f32) -> Result<f32, String> {

        // Reset DAC configuration
        self.reset_dacs()?;

        // Initiate a read operation, get the address of the data to be...
        let mut chunk = self._read_slice_inner(low, &[high], vidx!(-vread))?;

        // ... and finally withdraw voltage from the biasing channels
        self.ground_all()?.execute()?;

        // Read the data from the address chunk
        let data = self.read_raw(chunk.addr())?;

        // Assemble the number from the 4 neighbouring bytes
        let val: u32 = u32::from_le_bytes([data[4*high+0], data[4*high+1],
            data[4*high+2], data[4*high+3]]);

        // Free up the FPGA chunk for reuse
        let mut manager = self.memman.lock().unwrap();
        manager.free_chunk(&mut chunk)?;

        if high % 2 == 0 {
            Ok(_adc_to_current(val))
        } else {
            Ok(-1.0*_adc_to_current(val))
        }
    }

    /// Read all the values which have `chan` as the low potential channel
    ///
    /// If `chan` is between 0 and 15 or 32 and 47 (inclusive) this will correspond
    /// to a row read at `vread` in a standard 32×32 array, otherwise it's a column read.
    pub fn read_slice(&mut self, chan: usize, vread: f32) -> Result<Vec<f32>, String> {

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
            self.ground_all()?.execute()?;
            res = self.word_currents_from_address(chunk.addr())?;
        // Or the other way around. Channels 16..32 and
        // 48..64 correspond to columns so we want to read
        // from all the rows in the column.
        } else {
            // Initiate a read operation get the address of the data to be...
            chunk = self._read_slice_inner(chan, &*ALL_BITS, vidx!(-vread))?;
            // ... and finally withdraw voltage from the biasing channels
            self.ground_all()?.execute()?;
            res = self.bit_currents_from_address(chunk.addr())?;
        }

        // Free up the FPGA chunk for reuse
        let mut manager = self.memman.lock().unwrap();
        manager.free_chunk(&mut chunk)?;

        Ok(res)
    }

    /// Read the specified high channels that have `chan` as the low potential channel
    ///
    /// If `chan` is between 0 and 15 or 32 and 47 (inclusive) this will correspond
    /// to a row read at `vread` in a standard 32×32 array, otherwise it's a column read.
    /// This is equivalent to read_slice but replaces channels not contained in the
    /// mask with `f32::NAN`.
    pub fn read_slice_masked(&mut self, chan: usize, mask: &[usize], vread: f32) -> Result<Vec<f32>, String> {

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
        self.ground_all()?.execute()?;

        // Make an array to hold all the values of row/column
        let mut res: Vec<f32> = Vec::with_capacity(32);

        // Read the raw chunk of data
        let data = self.read_raw(chunk.addr())?;

        // Convert adc values to current
        for chan in all_channels {

            if all_channels_set.contains(&chan) {
                let raw_value = u32::from_le_bytes([data[4*chan+0],
                    data[4*chan+1], data[4*chan+2], data[4*chan+3]]);
                let cur = if chan % 2 == 0 {
                    _adc_to_current(raw_value)
                } else {
                    -1.0*_adc_to_current(raw_value)
                };
                res.push(cur);
            } else {
                res.push(f32::NAN);
            }
        }

        // Free up the FPGA chunk for reuse
        let mut manager = self.memman.lock().unwrap();
        manager.free_chunk(&mut chunk)?;

        Ok(res)
    }

    /// Same as [`Instrument::read_slice_masked`] but returning an [`Array`][`ndarray::Array`]
    /// for numpy compatibility
    pub fn read_slice_masked_as_ndarray(&mut self, chan: usize, mask: &[usize], vread: f32) ->
        Result<Array<f32, Ix1>, String> {

        let data = self.read_slice_masked(chan, mask, vread)?;
        Ok(Array::from(data))

    }

    /// Same as [`Instrument::read_slice`] but returning an [`Array`][`ndarray::Array`]
    /// for numpy compatibility
    pub fn read_slice_as_ndarray(&mut self, chan: usize, vread: f32) -> Result<Array<f32, Ix1>, String> {

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
    pub fn read_all(&mut self, vread: f32, order: BiasOrder) -> Result<Vec<f32>, String> {

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
    pub fn read_all_as_ndarray(&mut self, vread: f32, order: BiasOrder) -> Result<Array<f32, Ix2>, String> {

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
        let response = match self.efm.read_block(FIFOBUSYADDR, 1i32, BLFLAGS_R) {
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

    /// Apply a pulse between the specified channels. A voltage of `-voltage/2` will be
    /// applied to the `low` channel `voltage/2` to the `high` channel. When `preset_state`
    /// is true the state of high speed drivers will be initialised before the actual pulsing
    /// sequence begins.
    pub fn pulse_one(&mut self, low: usize, high: usize, voltage: f32, nanos: u128, preset_state: bool)
        -> Result<&mut Self, String> {

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
    pub fn pulse_slice(&mut self, chan: usize, voltage: f32, nanos: u128, preset_state: bool) -> Result<&mut Self, String> {

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
    pub fn pulse_slice_masked(&mut self, chan: usize, mask: &[usize], voltage: f32, nanos: u128, preset_state: bool) -> Result<&mut Self, String> {

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
        -> Result<(), String> {

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
        -> Result<&mut Self, String> {

        // set high and low channels as HS drivers
        let mut bias_conf = ChannelConf::new();
        bias_conf.set(high, ChannelState::VoltArb);
        bias_conf.set(low, ChannelState::VoltArb);

        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);
        self.process(conf.compile())?;

        // setup a non-high speed differential pulsing scheme
        self._setup_dacs_for_pulsing(&[(low, high, voltage)], false, true)?;
        self.add_delay(nanos+20_000u128)?;

        Ok(self)

    }

    /// Pulse a crosspoint using the high speed drivers. This function will *NOT*
    /// automatically flush output.
    fn pulse_one_fast(&mut self, low: usize, high: usize, voltage: f32, nanos: u128, preset_state: bool)
        -> Result<&mut Self, String> {

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
        // setup a high-speed differential pulsing scheme
        self._setup_dacs_for_pulsing(&[(low, high, voltage)], true, true)?;
        self.add_delay(20_000u128)?;
        // HS configuration
        self.process(hsconf.compile())?;
        // HS Pulse
        self.process(pulse)?;
        self.add_delay(nanos)?;

        Ok(self)

    }

    fn pulse_slice_slow(&mut self, chan: usize, voltage: f32, nanos: u128, mask: Option<&[usize]>) ->
        Result<&mut Self, String> {

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
        self.process(conf.compile())?;

        // setup a non-high speed differential pulsing scheme
        self._setup_dacs_for_pulsing(&channel_pairs, false, true)?;
        self.add_delay(nanos+20_000u128)?;

        Ok(self)
    }

    fn pulse_slice_fast(&mut self, chan: usize, voltage: f32, nanos: u128, mask: Option<&[usize]>, preset_state:bool) ->
        Result<&mut Self, String> {

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
        // setup a high speed differential pulsing scheme
        // WARNING! If a non-differential pulse (last argument is `false`)
        // is used instead `timings` for high channels above should be
        // set to 0 ns.
        self._setup_dacs_for_pulsing(&channel_pairs, true, true)?;
        self.add_delay(20_000u128)?;

        self.process(hsconf.compile())?;
        self.process(pulse.compile())?;
        self.add_delay(nanos)?;

        Ok(self)
    }

    /// Pulse all crosspoints at the specified voltage and pulse width
    ///
    /// This function will pulse available crosspoints on the array. This can be done
    /// either by high biasing the rows ([`BiasOrder::Rows`]) or columns ([`BiasOrder::Columns`]).
    pub fn pulse_all(&mut self, voltage: f32, nanos: u128, order: BiasOrder, preset_state: bool) -> Result<&mut Self, String> {

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

    /// Set control mode for daughterboards
    ///
    /// This can be set either to internal or header-controlled. For the 32NAA daughterboard
    /// the first scenario routes channels to the internal PLCC socket and the second to
    /// the on-board headers.
    pub fn set_control_mode(&mut self, mode: ControlMode) -> Result<&mut Self, String> {

        let mut iologic = match mode {
            ControlMode::Internal => UpdateLogic::new(false, true),
            ControlMode::Header => UpdateLogic::new(true, true)
        };

        self.process(iologic.compile())?;

        Ok(self)
    }

}


impl Drop for Instrument {
    fn drop(&mut self) {
        self.efm.close().unwrap();
    }
}

