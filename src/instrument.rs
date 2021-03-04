use std::{time, thread};
use beastlink as bl;

use crate::instructions::*;

const EFM03_VID: u16 = 0x10f8;
const EFM03_PID: u16 = 0xc583;
const BASEADDR: u32 = 0x80008000;
const WRITEDELAY: time::Duration = time::Duration::from_millis(3);
const BLFLAGS: bl::Flags = bl::Flags::ConstAddress;
const INBUF: usize = 32;


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
/// to process instructions and read back results.
pub struct Instrument {
    efm: bl::Device
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
    /// to discover devices.
    pub fn open(id: i32) -> Result<Instrument, String> {

        find_ids()?;

        match bl::Device::open(id) {
            Ok(d) => Ok(Instrument { efm: d }),
            Err(err) => Err(format!("Could not open device: {}", err))
        }
    }

    /// Load an FPGA bitstream from a file.
    /// The DAC clusters of the instrument will be initialised to their
    /// default state after successful load.
    pub fn load_firmware(&self, path: &str) -> Result<(), String> {
        match self.efm.program_from_file(&path) {
            Ok(()) => { Ok(()) },
            Err(err) => { Err(format!("Could not program FPGA: {}", err)) }
        }?;

        match self.reset_dacs() {
            Ok(()) => { Ok(()) },
            Err(err) => { Err(format!("Could not initialise DACs: {}", err)) }
        }

    }

    /// Open a new Instrument with a specified id and bitstream.
    /// This essentially combines [`Instrument::open()`] and
    /// [`Instrument::load_firmware()`].
    pub fn open_with_fw(id: i32, path: &str) -> Result<Instrument, String> {
        let instr = Instrument::open(id)?;
        instr.load_firmware(&path)?;
        instr.process(&*RESET_DAC)?;
        instr.process(&*SET_3V3_LOGIC)?;
        instr.process(&*UPDATE_DAC)?;

        Ok(instr)
    }

    /// Process a compiled instruction
    pub fn process<T: Instruction>(&self, instr: &T) -> Result<(), String> {

        match self.efm.write_block(BASEADDR, &mut instr.to_bytevec(), BLFLAGS) {

            Ok(()) => {
                thread::sleep(WRITEDELAY);
            },
            Err(err) => return Err(format!("Could not write buffer: {}", err))
        }

        Ok(())
    }

    /// Compiles and process an instruction
    pub fn compile_process<T: Instruction>(&self, instr: &mut T) -> Result<(), String> {

        self.process(instr.compile())

    }

    /// Read raw data from block memory
    pub fn read_raw(&self) -> Result<Vec<u8>, String> {
        match self.efm.read_block(BASEADDR, INBUF as i32, BLFLAGS) {
            Ok(buf) => Ok(buf),
            Err(err) => Err(format!("Could not read block memory: {}", err))
        }
    }

    /// Reset all DACs on the tool
    pub fn reset_dacs(&self) -> Result<(), String> {
        self.process(&*RESET_DAC)?;
        self.load_dacs()
    }

    /// Set global 3.3 V logic level
    pub fn set_3v3_logic(&self) -> Result<(), String> {
        self.process(&*SET_3V3_LOGIC)?;
        self.load_dacs()
    }

    /// Update the DAC configuration
    pub fn load_dacs(&self) -> Result<(), String> {
        self.process(&*UPDATE_DAC)
    }

}

impl Drop for Instrument {
    fn drop(&mut self) {
        self.efm.close().unwrap();
    }
}

