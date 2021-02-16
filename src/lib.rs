//use ndarray::{Array, Ix1};
use beastlink as bl;
use std::{time, thread};

const EFM03_VID: u16 = 0x10f8;
const EFM03_PID: u16 = 0xc583;
const PADDINGU32: u32 = 0x80008000;
const BASEADDR: u32 = 0x80008000;
const WRITEDELAY: time::Duration = time::Duration::from_millis(3);

pub struct Instrument {
    efm: bl::Device
}

impl Instrument {
    pub fn open(id: i32) -> Result<Instrument, String> {
        match bl::Device::open(id) {
            Ok(d) => Ok(Instrument { efm: d }),
            Err(err) => Err(format!("Could not open device: {}", err))
        }
    }

    pub fn find() -> Result<i32, String> {
        match bl::enumerate(EFM03_VID, EFM03_PID) {
            Ok(v) => Ok(v),
            Err(err) => Err(format!("Could not enumerate devices: {}", err))
        }
    }

    pub fn load_firmware(&self, path: &str) -> Result<(), String> {
        match self.efm.program_from_file(&path) {
            Ok(()) => { Ok(()) },
            Err(err) => { Err(format!("Could not program FPGA: {}", err)) }
        }
    }

    pub fn write_packet(&self, in_packet: &[u32]) -> Result<(), String> {

        let inlen: u32 = in_packet.len() as u32;

        // Input length MUST be a multiple of 8
        if inlen % 8 != 0 {
            return Err(String::from("Invalid packet length; must be N×8×(u32)"));
        }

        // Each value is 4 bytes. We need an additional u32 every 8 × u32 of the
        // input. So the total number of bytes is 4 × 9 × (length / 8), Instead
        // of 4 × length
        let mut packet: Vec<u8> = Vec::with_capacity(4*(9*(inlen/8)) as usize);

        // split the u32s into 4 × BE u8s
        for (i, num) in in_packet.iter().enumerate() {
            packet.extend_from_slice(&num.to_be_bytes());

            // padding bytes
            if i > 0 && (i+1) % 8 == 0 {
                packet.extend_from_slice(&PADDINGU32.to_be_bytes());
            }

            // abort if a packet could not be written, on success wait 3 ms
            // before continuing
            match self.efm.write_block(BASEADDR, &mut packet, bl::Flags::ConstAddress) {
                Ok(()) => { thread::sleep(WRITEDELAY); },
                Err(err) => return Err(format!("Could not write buffer: {}", err))
            }
        }

        Ok(())

    }

    pub fn reset_dacs(&self) -> Result<(), String> {
        let packet: [u32; 8] = [
            0x0000_0001, // opcode 0x1
            0x0000_ffff, // all DACs bitmask
            0x0000_0000, // unused
            0x0000_0000, // unused
            0x8000_8000, // +V: 0V; -V: 0V
            0x8000_8000, // +V: 0V; -V: 0V
            0x8000_8000, // +V: 0V; -V: 0V
            0x8000_8000  // +V: 0V; -V: 0V
        ];

        self.write_packet(&packet)
    }

    pub fn load_dacs(&self) -> Result<(), String> {
        let packet: [u32; 8] = [
            0x0000_0002, // opcode 0x2
            0x0000_0000, // unused
            0x0000_0000, // unused
            0x0000_0000, // unused
            0x0000_0000, // unused
            0x0000_0000, // unused
            0x0000_0000, // unused
            0x0000_0000  // unused
        ];

        self.write_packet(&packet)
    }

    /*pub fn make_buffer(&self, len: u32) -> Array<u32, Ix1> {
        let mut arr = Array::<u32, Ix1>::zeros(len as usize);
        arr[0]= 5u32;
        arr
    }*/
}

impl Drop for Instrument {
    fn drop(&mut self) {
        self.efm.close().unwrap();
    }
}
