use std::{time, thread};
use beastlink as bl;

use crate::instructions::*;

const EFM03_VID: u16 = 0x10f8;
const EFM03_PID: u16 = 0xc583;
const BASEADDR: u32 = 0x80000000;
const WRITEDELAY: time::Duration = time::Duration::from_millis(3);
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
    instr_buffer: Option<Vec<u8>>
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

impl Instrument {

    /// Create a new Instrument with a known ID.  Use [`find_ids`]
    /// to discover devices. Set `retained_mode` to `true` to defer
    /// writing instructions until an [`Instrument::flush()`]
    /// has been called explicitly.
    pub fn open(id: i32, retained_mode: bool) -> Result<Instrument, String> {

        find_ids()?;
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
                instr_buffer: buffer
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
        instr.flush()?;

        Ok(instr)
    }

    /// Process a compiled instruction
    pub fn process<T: Instruction>(&mut self, instr: &T) -> Result<(), String> {

        instrdbg!(instr);

        // If an instruction buffer is used write the bytes to the buffer
        // instead of directly to the tool.
        if let Some(buff) = &mut self.instr_buffer {
            buff.extend(instr.to_bytevec());
            return Ok(());
        }

        // Otherwise write directly to ArC2
        match self.efm.write_block(BASEADDR, &mut instr.to_bytevec(), BLFLAGS_W) {

            Ok(()) => {
                thread::sleep(WRITEDELAY);
            },
            Err(err) => return Err(format!("Could not write buffer: {}", err))
        }

        Ok(())
    }

    /// Write all retained instructions to ArC2. Has no effect if
    /// `retained_mode` is `false`.
    pub fn flush(&mut self) -> Result<(), String> {

        // If an instruction buffer is used empty it, otherwise do
        // nothing as all writes are immediates
        match self.instr_buffer {
            Some(ref mut buf) => {
                match self.efm.write_block(BASEADDR, buf, BLFLAGS_W) {
                    Ok(()) => {
                        thread::sleep(WRITEDELAY);
                        buf.clear();
                        Ok(())
                    },
                    Err(err) => Err(format!("Could not write buffer: {}", err))
                }
            },
            None => Ok(())
        }


    }

    /// Compiles and process an instruction
    pub fn compile_process<T: Instruction>(&mut self, instr: &mut T) -> Result<(), String> {

        self.process(instr.compile())

    }

    /// Read raw data from block memory
    pub fn read_raw(&self, addr: u32) -> Result<Vec<u8>, String> {
        match self.efm.read_block(addr, INBUF as i32, BLFLAGS_R) {
            Ok(buf) => { pktdbg!(buf); Ok(buf) },
            Err(err) => Err(format!("Could not read block memory: {}", err))
        }
    }

    /// Reset all DACs on the tool.
    pub fn reset_dacs(&mut self) -> Result<(), String> {
        self.process(&*RESET_DAC)?;
        self.load_dacs()
    }

    /// Set global 3.3 V logic level
    pub fn set_3v3_logic(&mut self) -> Result<(), String> {
        self.process(&*SET_3V3_LOGIC)?;
        self.load_dacs()
    }

    /// Update the DAC configuration
    pub fn load_dacs(&mut self) -> Result<(), String> {
        self.process(&*UPDATE_DAC)
    }

}

impl Drop for Instrument {
    fn drop(&mut self) {
        self.efm.close().unwrap();
    }
}

