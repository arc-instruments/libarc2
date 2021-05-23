use std::{time, thread};
use beastlink as bl;
use ndarray::{Array, Ix1, Ix2};

use crate::instructions::*;
use crate::registers::{DACMask, DACVoltage, ChannelState, ADCMask};
use crate::registers::{ChannelConf, PulseAttrs, ClusterMask};
use crate::memory::{MemMan};

const EFM03_VID: u16 = 0x10f8;
const EFM03_PID: u16 = 0xc583;
const BASEADDR: u32 = 0x80000000;
const FIFOBUSYADDR: u32 = 0x80020000;
const WRITEDELAY: time::Duration = time::Duration::from_nanos(1_250_000);
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
    memman: MemMan,
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
    /// writing instructions until an [`Instrument::flush()`]
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
                memman: MemMan::new()
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
        instr.add_delay(10_000u128)?;
        instr.flush()?;

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
                self.flush()?;
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

    /// Write all retained instructions to ArC2. Has no effect if
    /// `retained_mode` is `false`.
    pub fn flush(&mut self) -> Result<&mut Self, String> {

        // If an instruction buffer is used empty it, otherwise do
        // nothing as all writes are immediates
        match self.instr_buffer {
            Some(ref mut buf) => {
                #[cfg(not(feature="dummy_writes"))] {
                    match self.efm.write_block(BASEADDR, buf, BLFLAGS_W) {
                        Ok(()) => {
                            eprintln!("real write");
                            thread::sleep(WRITEDELAY);
                            buf.clear();
                            Ok(self)
                        },
                        Err(err) => Err(format!("Could not write buffer: {}", err))
                    }
                }
                #[cfg(feature="dummy_writes")] {
                    eprintln!("DW: {:?}", buf);
                    Ok(())
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

    /// Reset all DACs on the tool. This will flush output
    pub fn reset_dacs(&mut self) -> Result<&mut Self, String> {
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(10_000u128)?;
        self.flush()
    }

    /// Disconnect all channels
    pub fn float_all(&mut self) -> Result<&mut Self, String> {
        self.process(&*CHAN_FLOAT_ALL)?;
        self.flush()
    }

    pub fn ground_all(&mut self) -> Result<&mut Self, String> {
        self.process(&*CHAN_ARB_ALL)?;
        self.process(&*RESET_DAC)?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(10_000u128)?;
        self.flush()
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

    /// Perform a current read between the specified channels. A voltage
    /// of `-vread` will be applied to the `low` channel and current will
    /// be read from the `high` channel.
    pub fn read_one(&mut self, low: usize, high: usize, vread: f32) -> Result<f32, String> {

        // Reset DAC configuration
        self.reset_dacs()?;

        // +--------------------------+
        // | Output DAC configuration |
        // +--------------------------+

        // Create a channel mask containing only one channel
        // (the biasing one)
        let mut chanmask = DACMask::NONE;
        chanmask.set_channel(low);

        // Convert the voltage into a DAC value
        let voltage = vidx!(-vread);
        let mut dacvoltage = DACVoltage::new();
        // Set the DAC-/DAC+ channels halves of the DAC
        dacvoltage.set(low % dacvoltage.len(), voltage);

        // Make the SetDAC instruction for the biasing channel
        let mut setdac = SetDAC::with_regs(&chanmask, &dacvoltage);
        self.process(setdac.compile())?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(10_000u128)?;

        // +------------------------------+
        // | Output Channel configuration |
        // +------------------------------+

        // set all channels to Arbitrary Voltage
        let mut channelconf =
            UpdateChannel::from_regs_global_state(ChannelState::VoltArb);
        self.process(channelconf.compile())?;

        // +------------------+
        // | IO configuration |
        // +------------------+

        // set all channel to output
        let mut ioconf = UpdateLogic::new(true, true);
        self.process(ioconf.compile())?;

        // +-------------+
        // | CurrentRead |
        // +-------------+

        // Get a new memory address to store data
        let mut chunk = self.memman.alloc_chunk().unwrap();

        // Create a mask containing the read-out channel
        let mut adcmask = ADCMask::new();
        adcmask.set_enabled(high, true);

        let mut currentread = CurrentRead::new(&adcmask, chunk.addr());
        self.process(currentread.compile())?;
        self.add_delay(1_000u128)?;

        // And reset the DACs, removing all biasing
        // This will also flush the write buffer loading
        // the instructions on the FPGA
        self.reset_dacs()?;

        let data = self.read_raw(chunk.addr())?;

        // Assemble the number from the 4 neighbouring bytes
        let val: u32 = u32::from_le_bytes([data[4*high+0], data[4*high+1],
            data[4*high+2], data[4*high+3]]);

        // Free up the FPGA chunk for reuse
        match self.memman.free_chunk(&mut chunk) {
            Ok(()) => {},
            Err(_) => { return Err("Could not free up memory!!".to_string()); }
        }

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

        // Create a channel mask containing only one channel
        // (the biasing one)
        let mut chanmask = DACMask::NONE;
        chanmask.set_channel(chan);

        // Convert the voltage into a DAC value
        let voltage = vidx!(-vread);
        let mut dacvoltage = DACVoltage::new();
        // Set the DAC-/DAC+ channels halves of the DAC
        dacvoltage.set(chan % dacvoltage.len(), voltage);

        // Make the SetDAC instruction for the biasing channel
        let mut setdac = SetDAC::with_regs(&chanmask, &dacvoltage);
        self.process(setdac.compile())?;
        self.process(&*UPDATE_DAC)?;
        self.add_delay(10_000u128)?;

        // +------------------------------+
        // | Output Channel configuration |
        // +------------------------------+

        // set all channels to Arbitrary Voltage
        let mut channelconf =
            UpdateChannel::from_regs_global_state(ChannelState::VoltArb);
        self.process(channelconf.compile())?;

        // +------------------+
        // | IO configuration |
        // +------------------+

        // set all channel to output
        let mut ioconf = UpdateLogic::new(true, true);
        self.process(ioconf.compile())?;

        // Get a new memory address to current data
        let mut chunk = self.memman.alloc_chunk().unwrap();

        let mut channels: Vec<usize> = Vec::with_capacity(32);

        // Channels 0..16 and 32..48 correspond to rows
        // so we want to read from all the columns in the
        // row
        if (chan < 16) || ((32 <= chan) && (chan < 48)) {
            channels.append(&mut (16usize..32).collect::<Vec<usize>>());
            channels.append(&mut (48usize..64).collect::<Vec<usize>>());
        // Or the other way around. Channels 16..32 and
        // 48..64 correspond to columns so we want to read
        // from all the rows in the column.
        } else {
            channels.append(&mut ( 0usize..16).collect::<Vec<usize>>());
            channels.append(&mut (32usize..48).collect::<Vec<usize>>());
        }

        // +-------------+
        // | CurrentRead |
        // +-------------+

        // Prepare the ADC mask for the readout
        let mut adcmask = ADCMask::new();

        for chan in &channels {
            adcmask.set_enabled(*chan, true);
        }

        let mut currentread = CurrentRead::new(&adcmask, chunk.addr());
        self.process(currentread.compile())?;
        self.add_delay(1_000u128)?;

        // And reset the DACs, removing all biasing
        // This will also flush the write buffer loading
        // the instructions on the FPGA
        self.reset_dacs()?;

        // Make an array to hold all the values of row/column
        let mut res: Vec<f32> = Vec::with_capacity(32);

        // Read the raw chunk of data
        let data = self.read_raw(chunk.addr())?;

        // Convert adc values to current
        for chan in &channels {
            let raw_value = u32::from_le_bytes([data[4*chan+0],
                data[4*chan+1], data[4*chan+2], data[4*chan+3]]);
            let cur = if chan % 2 == 0 {
                _adc_to_current(raw_value)
            } else {
                -1.0*_adc_to_current(raw_value)
            };
            res.push(cur);
        }

        // Free up the FPGA chunk for reuse
        match self.memman.free_chunk(&mut chunk) {
            Ok(()) => {},
            Err(_) => { return Err("Could not free up memory!!".to_string()); }
        }

        Ok(res)
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
    /// either by biasing the columns ([`BiasOrder::Rows`]) or rows ([`BiasOrder::Columns`]).
    /// The result is stored in linear vector (and not a 2D matrix) in blocks of 32 values
    /// in channel order. When order is `Columns` the low potential channels are `[16..32)`
    /// and `[48..64)` (wordlines). When order is `Rows` the low potential channels are
    /// `[0..16)` and `[32..48)` (bitlines). Function [`Instrument::read_slice()`] is
    /// applied for every one of the selected channels.
    pub fn read_all(&mut self, vread: f32, order: BiasOrder) -> Result<Vec<f32>, String> {
        let mut bias_channels: Vec<usize> = Vec::with_capacity(32);

        let mut results = Vec::with_capacity(32*32);

        match order {
            BiasOrder::Rows => {
                bias_channels.append(&mut (16usize..32).collect::<Vec<usize>>());
                bias_channels.append(&mut (48usize..64).collect::<Vec<usize>>());

            },
            BiasOrder::Columns => {
                bias_channels.append(&mut ( 0usize..16).collect::<Vec<usize>>());
                bias_channels.append(&mut (32usize..48).collect::<Vec<usize>>());
            }
        };

        for chan in &bias_channels {
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
    /// applied to the `low` channel `voltage/2` to the `high` channel.
    pub fn pulse_one(&mut self, low: usize, high: usize, voltage: f32, nanos: u128)
        -> Result<&mut Self, String> {

        // use the high speed driver for all pulses faster than 500 ms
        if nanos < 500_000_000u128 {
            self.pulse_one_fast(low, high, voltage, nanos)?;
        } else {
            self.pulse_one_slow(low, high, voltage, nanos)?;
        }

        Ok(self)
    }

    /// Setup the biasing channels for single pulsing. This function will setup the
    /// output DACs for pulsing of a single crosspoint. It will set the high channel
    /// to `voltage/2` and the low channel to `-voltage/2`. If the `differential`
    /// argument is true then the DACs will be setup for high-speed differential
    /// pulsing as required by the High Speed drivers. No delays are introduced
    /// here as this will be handled either by a standard Delay instruction or
    /// a High Speed timer.
    fn _setup_dacs_for_single_pulsing(&mut self, low: usize, high: usize,
        voltage: f32, differential: bool) -> Result<(), String> {
        //
        // Set high channel to voltage/2
        let mut highmask = DACMask::NONE;
        highmask.set_channel(high);

        let mut high_v = DACVoltage::new();
        if differential {
            // in differential biasing we need to ensure that DAC+ is higher than DAC-
            if voltage > 0.0 {
                high_v.set_high(high % high_v.len(), vidx!(voltage/2.0));
                high_v.set_low(high % high_v.len(), vidx!(0.0));
            } else {
                high_v.set_low(high % high_v.len(), vidx!(voltage/2.0));
                high_v.set_high(high % high_v.len(), vidx!(0.0));
            }
        } else {
            // otherwise both DAC+ and DAC- are the same value
            high_v.set(high % high_v.len(), vidx!(voltage/2.0));
        }

        // Set low channel to -voltage/2
        let mut lowmask = DACMask::NONE;
        lowmask.set_channel(low);

        let mut low_v = DACVoltage::new();
        if differential {
            if voltage < 0.0 {
                low_v.set_high(low % low_v.len(), vidx!(-voltage/2.0));
                low_v.set_low(low % low_v.len(), vidx!(0.0));
            } else {
                low_v.set_low(low % low_v.len(), vidx!(-voltage/2.0));
                low_v.set_high(low % low_v.len(), vidx!(0.0));
            }
        } else {
            low_v.set(low % low_v.len(), vidx!(-voltage/2.0));
        }

        let mut setdac_high = SetDAC::with_regs(&highmask, &high_v);
        let mut setdac_low = SetDAC::with_regs(&lowmask, &low_v);

        self.process(setdac_high.compile())?;
        self.process(setdac_low.compile())?;
        self.process(&*UPDATE_DAC)

    }

    /// Pulse a crosspoint using conventional biasing and delaying
    fn pulse_one_slow(&mut self, low: usize, high: usize, voltage: f32, nanos: u128)
        -> Result<&mut Self, String> {

        eprintln!(">>> pulse slow");
        self.ground_all()?;
        // set high and low channels as HS drivers
        let mut bias_conf = ChannelConf::new();
        bias_conf.set(high, ChannelState::VoltArb);
        bias_conf.set(low, ChannelState::VoltArb);

        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);
        self.process(conf.compile())?;

        // setup a non-differential pulsing scheme
        self._setup_dacs_for_single_pulsing(low, high, voltage, false)?;
        self.add_delay(nanos+10_000u128)?;

        self.flush()?;

        // ground everything back to 0V
        self.ground_all()
    }

    /// Pulse a crosspoint using the high speed drivers
    fn pulse_one_fast(&mut self, low: usize, high: usize, voltage: f32, nanos: u128)
        -> Result<&mut Self, String> {

        self.ground_all()?;

        // set high and low channels as HS drivers
        let mut bias_conf = ChannelConf::new();
        bias_conf.set(high, ChannelState::HiSpeed);
        bias_conf.set(low, ChannelState::HiSpeed);

        let mut conf = UpdateChannel::from_regs_default_source(&bias_conf);
        self.process(conf.compile())?;

        // setup a differential pulsing scheme
        self._setup_dacs_for_single_pulsing(low, high, voltage, true)?;
        self.add_delay(10_000u128)?;

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
        self.process(hsconf.compile())?;

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

        let mut pulse = HSPulse::new_from_attrs(&pulse_attrs);
        self.process(pulse.compile())?;

        self.flush()?;

        // ground everything back to 0V
        self.ground_all()
    }

}


impl Drop for Instrument {
    fn drop(&mut self) {
        self.efm.close().unwrap();
    }
}

