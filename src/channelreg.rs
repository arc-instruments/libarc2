use bitvec::prelude::{BitVec, Msb0};

#[derive(Clone, Copy)]
pub enum ChannelStatus {
    Open = 0b001,
    CloseGND = 0b010,
    CapGND = 0b011,
    VoltArb = 0b100,
    CurArb = 0b101,
    HiSpeed = 0b110,
}

impl ChannelStatus {
    fn as_bools(&self) -> (bool, bool, bool) {
        let b0 = ((*self as u8 >> 0) & 1) == 1;
        let b1 = ((*self as u8 >> 1) & 1) == 1;
        let b2 = ((*self as u8 >> 2) & 1) == 1;

        (b0, b1, b2)
    }
}

pub struct ChannelStatusRegister {
    bits: BitVec<Msb0, u32>,
}

impl ChannelStatusRegister {
    pub fn new() -> ChannelStatusRegister {
        let vec: BitVec<Msb0, u32> = BitVec::repeat(false, 192);

        ChannelStatusRegister { bits: vec }
    }

    pub fn set_channel(&mut self, idx: usize, val: ChannelStatus) {
        let bits = self.bits.as_mut_bitslice();
        let bools = val.as_bools();

        bits.set(3 * idx, bools.2);
        bits.set(3 * idx + 1, bools.1);
        bits.set(3 * idx + 2, bools.0);
    }

    pub fn view_bytes(&self) -> &[u32] {
        self.bits.as_raw_slice()
    }
}

