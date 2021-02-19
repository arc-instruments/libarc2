pub mod channelconf {

    use std::convert::TryFrom;
    use bitvec::prelude::{BitVec, Msb0, BitStore, BitSlice, bitarr};
    use num_derive::FromPrimitive;
    use num_traits::FromPrimitive;

    const CHANSIZE: usize = 3;

    /// Different channel configurations currently
    /// supported by ArC2. Use these with [`ChannelConfRegister`]
    /// to control the behaviour of individual ArC2 channels.
    #[derive(Clone, Copy, FromPrimitive, Debug)]
    #[repr(u8)]
    pub enum ChannelConf {
        /// Open channel; channel will not be connected
        /// to anything.
        Open = 0b001,
        /// Channel is GND
        CloseGND = 0b010,
        /// Channel cap to GND
        CapGND = 0b011,
        /// Channel is set for arbitrary voltage operation
        VoltArb = 0b100,
        /// Channel is set for arbitrary current operation
        CurArb = 0b101,
        /// High-Speed pulse channel
        HiSpeed = 0b110,
    }

    impl ChannelConf {
        fn as_bools(&self) -> [bool; CHANSIZE] {

            let mut bools: [bool; CHANSIZE] = [false; CHANSIZE];

            for i in 0..CHANSIZE {
                bools[i] = ((*self as u8 >> i) & 1) == 1
            }

            bools
        }

        fn from_bools(bools: &[bool; CHANSIZE]) -> ChannelConf {
            let mut bitarr = bitarr![Msb0, u8; 0; 8];

            for i in 0..CHANSIZE {
               bitarr.set(8-CHANSIZE+i, bools[i])
            }

            let value: [u8; 1] = bitarr.value();
            ChannelConf::from_u8(value[0] as u8).unwrap()

        }

        fn from_bitslice(bools: &BitSlice<Msb0, u32>) -> Result<ChannelConf, String> {

            let len: usize;

            if bools.len() < CHANSIZE {
                return Err(String::from("Supplied slice is too small"));
            }

            if bools.len() > 8 {
                len = 8;
            } else {
                len = bools.len()
            }

            let mut bitarr = bitarr![Msb0, u8; 0; 8];

            for i in 0..len {
               bitarr.set(8-len+i, bools[i])
            }

            let value: [u8; 1] = bitarr.value();
            Ok(ChannelConf::from_u8(value[0] as u8).unwrap())
        }

    }

    impl From<&[bool; CHANSIZE]> for ChannelConf {
        fn from(bools: &[bool; CHANSIZE]) -> ChannelConf {
            ChannelConf::from_bools(&bools)
        }
    }

    impl TryFrom<&BitSlice<Msb0, u32>> for ChannelConf {

        type Error = String;

        fn try_from(v: &BitSlice<Msb0, u32>) -> Result<Self, Self::Error> {
            ChannelConf::from_bitslice(v)
        }
    }

    /// `ChannelConfRegister` contains the configuration of then channels for
    /// a given operation. Currently it's built for 3 bits per channel for a
    /// total of 64 channels (192-bits). The underlying implementation uses a
    /// [`BitVec`][bitvec::vec::BitVec] storing MSB bits and backed by [`u32`]s.
    /// This matches the structure that ArC2 is expecting for the channel
    /// configuration.
    ///
    /// To create a new register call [`ChannelConfRegister::new()`] with the
    /// desired number of channels. For typical ArC2 scenarios this should be 64.
    /// By default the register is populated with zeros (which is an invalid
    /// status for ArC2) and must be configured appropriately by setting the
    /// invididual channels to a [`ChannelConf`] value. The register will take
    /// care of flipping the correct bits in the internal representation in order
    /// to have a consistent 32bit representation.
    ///
    /// **See also**: [`ChannelConf`] for the available channel configurations.
    ///
    /// ## Examples
    ///
    /// ```
    /// // Initialise a new status register
    /// let mut reg = ChannelConfRegister::new(64);
    ///
    /// // Number of allocated channels
    /// let nchan = reg.len();
    ///
    /// // Set channel 31 to High speed pulsing mode
    /// reg.set(31, ChannelConf::HiSpeed);
    ///
    /// // Set all channels to arbitrary voltage operation
    /// reg.set_all(ChannelConf::VoltArb);
    ///
    /// // Traverse channels (non-consuming iterator)
    /// for channel in &reg {
    ///     println!("{:?}", channel);
    /// }
    ///
    /// // Print the internal representation
    /// // Should return
    /// //  0x92492492
    /// //  0x49249249
    /// //  0x24924924
    /// //  0x92492492
    /// //  0x49249249
    /// //  0x24924924
    /// for value in reg.as_repr() {
    ///    println!("0x{:x}", value);
    /// }
    ///
    /// ```
    pub struct ChannelConfRegister {
        bits: BitVec<Msb0, u32>,
    }

    impl ChannelConfRegister {

        /// Create a new register with the specified number of
        /// channels. This will be expanded to `CHANSIZE` × channels
        /// in the internal bit vector representation.
        pub fn new(channels: usize) -> ChannelConfRegister {
            // CHANSIZE bits for each channel
            let vec: BitVec<Msb0, u32> = BitVec::repeat(false, channels*CHANSIZE);

            ChannelConfRegister { bits: vec }
        }

        /// Set a channel to a [`ChannelConf`] value
        pub fn set(&mut self, idx: usize, val: ChannelConf) {
            let bits = self.bits.as_mut_bitslice();
            let bools = val.as_bools();

            for i in 0..bools.len() {
                bits.set(CHANSIZE * idx + i, bools[CHANSIZE-1-i]);
            }
        }

        /// Get the [`configuration`][`ChannelConf`] of a channel
        pub fn get(&self, idx: usize) -> ChannelConf {
            let v = &self.bits[idx*CHANSIZE..(idx+1)*CHANSIZE];

            ChannelConf::try_from(v).unwrap()
        }

        /// Get the number of allocated channels
        pub fn len(&self) -> usize {
            // len is always a multiple of CHANSIZE
            self.bits.len() / CHANSIZE
        }

        /// Set the status of all channels to the same value
        pub fn set_all(&mut self, val: ChannelConf) {
            let nchannels = self.len();

            for i in 0..nchannels {
                self.set(i, val);
            }
        }

        /// Get the serialisable format of this register specified
        /// as a slice whatever the internal representation is. This
        /// is presently a [`u32`] as this is the size of words that
        /// ArC2 is expecting as input.
        pub fn as_repr(&self) -> &[u32] {
            self.bits.as_raw_slice()
        }
    }

    #[doc(hidden)]
    pub struct ChannelConfRegisterIterator<'a> {
        register: &'a ChannelConfRegister,
        index: usize,
    }

    impl<'a> IntoIterator for &'a ChannelConfRegister {

        type Item = ChannelConf;
        type IntoIter = ChannelConfRegisterIterator<'a>;

        fn into_iter(self) -> Self::IntoIter {
            ChannelConfRegisterIterator {
                register: self,
                index: 0,
            }
        }

    }

    impl<'a> Iterator for ChannelConfRegisterIterator<'a> {

        type Item = ChannelConf;

        fn next(&mut self) -> Option<ChannelConf> {
            if self.index >= self.register.len() {
                return None;
            }

            let v = self.register.get(self.index);
            self.index += 1;
            Some(v)
        }

    }


    #[cfg(test)]
    mod tests {

        use crate::{ChannelConf, ChannelConfRegister};
        use assert_matches::assert_matches;

        #[test]
        fn get_channel() {
            let mut v = ChannelConfRegister::new(64);
            v.set(50, ChannelConf::VoltArb);
            let res = v.get(50);
            assert_matches!(res, ChannelConf::VoltArb);

            v.set(0, ChannelConf::Open);
            let res = v.get(0);
            assert_matches!(res, ChannelConf::Open);

            v.set(63, ChannelConf::HiSpeed);
            let res = v.get(63);
            assert_matches!(res, ChannelConf::HiSpeed);
        }

        #[test]
        fn channel_len() {
            let v = ChannelConfRegister::new(64);
            assert_eq!(v.len(), 64);
        }

        #[test]
        fn bools_to_status() {
            let status0 = ChannelConf::from(&[false, true, false]);
            assert_matches!(status0, ChannelConf::CloseGND);

            let status1 = ChannelConf::from(&[true, false, false]);
            assert_matches!(status1, ChannelConf::VoltArb);

            let status2 = ChannelConf::from(&[false, false, true]);
            assert_matches!(status2, ChannelConf::Open);
        }

        #[test]
        fn all_channel_test() {
            let mut v = ChannelConfRegister::new(64);
            v.set_all(ChannelConf::VoltArb);

            for channel in &v {
                assert_matches!(channel, ChannelConf::VoltArb);
            }

            let slice = v.as_repr();

            assert_eq!(slice[0], 0x92492492);
            assert_eq!(slice[1], 0x49249249);
            assert_eq!(slice[2], 0x24924924);
            assert_eq!(slice[3], 0x92492492);
            assert_eq!(slice[4], 0x49249249);
            assert_eq!(slice[5], 0x24924924);
        }
    }
}
