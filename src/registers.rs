//! Data that can be serialised into ArC2 instructions.
//!
//! A register is a piece of u32-encodable information that can is used
//! to form one or more instructions that ArC2 can parse. All registers
//! are expected to implement the [`ToU32s`][`crate::registers::ToU32s`]
//! trait that converts them into a serialisable `Vec<u32>`. This can be
//! then be processed by ArC2.

use std::time::Duration;
use num_derive::{FromPrimitive, ToPrimitive};
use bitvec::prelude::*;
use bitflags::bitflags;
use num_traits::{FromPrimitive};
use thiserror::Error;

mod wordreg {

    /// A trait denoting a word size; ie how many words
    /// a register is using.
    pub trait WordSize {
        const WORDS: usize;
    }

    /// One word
    pub struct Wx1;
    impl WordSize for Wx1 {
        const WORDS: usize = 1;
    }

    /// Two words
    pub struct Wx2;
    impl WordSize for Wx2 {
        const WORDS: usize = 2;
    }

    /// Three words
    pub struct Wx3;
    impl WordSize for Wx3 {
        const WORDS: usize = 3;
    }

    /// Four words
    pub struct Wx4;
    impl WordSize for Wx4 {
        const WORDS: usize = 4;
    }

    /// Five words
    pub struct Wx5;
    impl WordSize for Wx5 {
        const WORDS: usize = 5;
    }

    /// Six words
    pub struct Wx6;
    impl WordSize for Wx6 {
        const WORDS: usize = 6;
    }

    /// Seven words
    pub struct Wx7;
    impl WordSize for Wx7 {
        const WORDS: usize = 7;
    }
}

use wordreg::WordSize;


pub(crate) mod consts {
    use super::DACMask;
    use super::ClusterMask;

    pub(crate) const CHANMAP: [DACMask; 64] = [
        DACMask::CH00_03, DACMask::CH00_03, DACMask::CH00_03, DACMask::CH00_03,
        DACMask::CH04_07, DACMask::CH04_07, DACMask::CH04_07, DACMask::CH04_07,
        DACMask::CH08_11, DACMask::CH08_11, DACMask::CH08_11, DACMask::CH08_11,
        DACMask::CH12_15, DACMask::CH12_15, DACMask::CH12_15, DACMask::CH12_15,
        DACMask::CH16_19, DACMask::CH16_19, DACMask::CH16_19, DACMask::CH16_19,
        DACMask::CH20_23, DACMask::CH20_23, DACMask::CH20_23, DACMask::CH20_23,
        DACMask::CH24_27, DACMask::CH24_27, DACMask::CH24_27, DACMask::CH24_27,
        DACMask::CH28_31, DACMask::CH28_31, DACMask::CH28_31, DACMask::CH28_31,
        DACMask::CH32_35, DACMask::CH32_35, DACMask::CH32_35, DACMask::CH32_35,
        DACMask::CH36_39, DACMask::CH36_39, DACMask::CH36_39, DACMask::CH36_39,
        DACMask::CH40_43, DACMask::CH40_43, DACMask::CH40_43, DACMask::CH40_43,
        DACMask::CH44_47, DACMask::CH44_47, DACMask::CH44_47, DACMask::CH44_47,
        DACMask::CH48_51, DACMask::CH48_51, DACMask::CH48_51, DACMask::CH48_51,
        DACMask::CH52_55, DACMask::CH52_55, DACMask::CH52_55, DACMask::CH52_55,
        DACMask::CH56_59, DACMask::CH56_59, DACMask::CH56_59, DACMask::CH56_59,
        DACMask::CH60_63, DACMask::CH60_63, DACMask::CH60_63, DACMask::CH60_63
    ];

    pub(crate) const HSCLUSTERMAP: [ClusterMask; 8] = [
        ClusterMask::CL0,
        ClusterMask::CL1,
        ClusterMask::CL2,
        ClusterMask::CL3,
        ClusterMask::CL4,
        ClusterMask::CL5,
        ClusterMask::CL6,
        ClusterMask::CL7,
    ];

    pub(super) const CHANCONFSIZE: usize = 2;
    pub(super) const NCHANS: usize = 64;
}

#[derive(Error, Debug)]
pub(crate) enum Error {
    #[error("Supplied slice is too small")]
    SliceTooSmall
}


/// Convert a value into a `Vec<u32>`.
pub trait ToU32s {

    /// Convert an object, typically a register, into a serialisable
    /// vector of u32s. These are the values that are actually written
    /// to the ArC2 buffer.
    fn as_u32s(&self) -> Vec<u32>;
}


/// Opcodes designate an ArC2 operation
///
/// An [`OpCode`] is typically the first register in an ArC2 instruction
/// and may be followed by a series of arguments.
#[derive(Clone, Copy, FromPrimitive, ToPrimitive)]
#[repr(u32)]
pub enum OpCode {
    /// Set a DAC configuration.
    SetDAC         = 0x00000001,
    /// Enable a DAC configuration previously set with [`OpCode::SetDAC`].
    UpdateDAC      = 0x00000002,
    /// Read Current operation.
    CurrentRead    = 0x00000004,
    /// Read voltage operation.
    VoltageRead    = 0x00000008,
    /// Set selector.
    UpdateSelector = 0x00000010,
    /// Set logic levels.
    UpdateLogic    = 0x00000020,
    /// Update channel configuration.
    UpdateChannel  = 0x00000040,
    /// Clear instrument buffer.
    Clear          = 0x00000080,
    /// Configure for high speed pulse operation.
    HSPulseConfig  = 0x00000100,
    /// Initiate a high speed pulse operation.
    HSPulseStart   = 0x00000200,
    /// Modify channel configuration.
    ModifyChannel  = 0x00000400,
    /// Set DAC Offsets (currently nop).
    SetDACOffset   = 0x00001000,
    /// Delay (20 ns precision)
    Delay          = 0x00002000,
    /// OpAmp preparation
    AmpPrep        = 0x00010000
}

impl ToU32s for OpCode {
    fn as_u32s(&self) -> Vec<u32> {
        [*self as u32].to_vec()
    }
}



/// An empty register, typically used to pad instructions
/// to full length.
///
/// An [`Empty`] should never need to be invoked manually
/// as empty instructions are added appropriately
/// when an instruction is compiled.
pub struct Empty(u32);

impl Empty {
    /// Create a new empty register
    pub fn new() -> Empty {
        Empty(0x0)
    }
}

impl ToU32s for Empty {
    fn as_u32s(&self) -> Vec<u32> {
        [self.0].to_vec()
    }
}


/// Register used to terminate instructions.
///
/// As with [`Empty`] this should not need
/// to be invoked manually as it is typically compiled into
/// an instruction.
pub struct Terminate(u32);

impl Terminate {
    pub fn new() -> Terminate {
        Terminate(0x80008000)
    }
}

impl ToU32s for Terminate {
    fn as_u32s(&self) -> Vec<u32> {
        [self.0].to_vec()
    }
}



/// Register used to denote durations (50 MHz precision)
///
/// The `Duration` register is used to encode duration. This might be
/// required, for instance, to allow for delays when settling during
/// configuration of the system DACs or to create arbitrary waveforms
/// where the precision of High Speed Driver is not required. Durations
/// can either be created by specifying the nanosecond delay or from a
/// [`std::time::Duration`] object.
///
/// ## Examples
/// ```
/// use libarc2::registers::{Duration50, ToU32s};
/// use std::time::Duration;
///
/// let delay0 = Duration50::from_nanos(1210);
/// // int(1210/20) = 60 or 0x3C
/// assert_eq!(delay0.as_u32s()[0], 0x3C);
///
/// let delay1 = Duration50::from_duration(&Duration::from_nanos(1210));
/// assert_eq!(delay1.as_u32s()[0], 0x3C);
///
/// // This one exceeds the maximum value and will be capped.
/// let delay2 = Duration50::from_nanos(87000000000);
/// assert_eq!(delay2.as_u32s()[0], 0xFFFFFFFF);
/// ```
pub struct Duration50(u32);

impl Duration50 {
    // This is the maximum duration we can fit in a 32bit integer on
    // a 50 MHz clock (20 ns increments)
    // We can fit up to 2^32 20 ns steps
    // We choose u128 as the underlying type to maintain compatibility
    // with the std::time::Duration API. Values will never exceed this
    // value otherwise.
    const MAX_DURATION_NS: u128 = 20u128 * (std::u32::MAX as u128);

    /// Create a new delay register with 20 ns precision. Please note
    /// that even though a `u128` is used as an argument the maximum
    /// delay value that is supported by ArC2 is `2^32 × 20 ns`. Values
    /// over that maximum will be capped.
    pub fn from_nanos(ns: u128) -> Duration50 {
        if ns > Self::MAX_DURATION_NS {
            Duration50(std::u32::MAX)
        } else {
            Duration50((ns / 20) as u32)
        }
    }

    /// Create a new delay register with 20 ns precision from a
    /// [`Duration`][`std::time::Duration`] object.
    pub fn from_duration(duration: &std::time::Duration) -> Duration50 {
        Duration50::from_nanos(duration.as_nanos())
    }
}

impl ToU32s for Duration50 {
    fn as_u32s(&self) -> Vec<u32> {
        [self.0].to_vec()
    }
}


/// High Speed DAC clusters to manipulate
#[derive(Clone, Copy, FromPrimitive, ToPrimitive, Debug)]
#[repr(usize)]
pub enum DACCluster {
    /// Cluster 0
    CL0 = 0,
    /// Cluster 1
    CL1 = 1,
    /// Cluster 2
    CL2 = 2,
    /// Cluster 3
    CL3 = 3,
    /// Cluster 4
    CL4 = 4,
    /// Cluster 5
    CL5 = 5,
    /// Cluster 6
    CL6 = 6,
    /// Cluster 7
    CL7 = 7
}

bitflags! {
    /// High Speed cluster selection bitmask
    ///
    /// [`ClusterMask`] is used to create a suitable bitmask for
    /// activating a high speed pulse on one or more output DAC
    /// clusters.
    pub struct ClusterMask: u8 {
        const NONE = 0b00000000;
        const CL0  = 0b00000001;
        const CL1  = 0b00000010;
        const CL2  = 0b00000100;
        const CL3  = 0b00001000;
        const CL4  = 0b00010000;
        const CL5  = 0b00100000;
        const CL6  = 0b01000000;
        const CL7  = 0b10000000;

        const ALL = Self::CL0.bits | Self::CL1.bits | Self::CL2.bits |
                    Self::CL3.bits | Self::CL4.bits | Self::CL5.bits |
                    Self::CL6.bits | Self::CL7.bits;
    }
}


impl ClusterMask {

    /// Create a new [`ClusterMask`] by providing a slice of
    /// clusters instead of a bitmask.
    pub fn new_from_cluster_idx(clusters: &[u8]) -> Self {
        let mut mask = ClusterMask::NONE;
        for cl in clusters {
            mask |= consts::HSCLUSTERMAP[*cl as usize];
        }

        mask
    }

}


/// Delays for high speed pulse drivers
///
/// This register is used to configure the delays of the high speed
/// pulse drivers. It is essentially a 224-bit integer containing
/// all cluster timings adhering to following layout.
///
/// ```text
/// [CL7][CL6][CL5][CL4][CL3][CL2][CL1][CL0]
///                                       ^
///                                       |
///                                       +-- LSB
/// ```
///
/// ## Example
/// ```
/// use libarc2::registers::{HSDelay, ToU32s, DACCluster};
/// use std::time::Duration;
///
/// let mut delay = HSDelay::new();
///
/// // Set cluster 0 to 100 ns
/// delay.set_cluster_nanos(DACCluster::CL0, 100);
/// assert_eq!(delay.as_u32s(), [0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xa]);
///
/// // And back to 0
/// delay.set_cluster_nanos(DACCluster::CL0, 0);
/// assert_eq!(delay.as_u32s(), [0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]);
///
/// // Set cluster 1 to 220 ns
/// delay.set_cluster_nanos(DACCluster::CL1, 220);
/// assert_eq!(delay.as_u32s(), [0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x60000000]);
///
/// // And also cluster 0 to 300 ns using a Duration
/// delay.set_cluster_from_duration(DACCluster::CL0, &Duration::from_nanos(300));
/// assert_eq!(delay.as_u32s(), [0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x6000001e]);
/// ```
pub struct HSDelay {
    bits: BitVec<u32, Msb0>
}

impl HSDelay {

    const RESOLUTION: u128 = 10u128;
    const MAX_DELAY: u128 = Self::RESOLUTION * 2u128.pow(28);
    const CLUSTERS: usize = 8;
    const CLUSTSIZE: usize = 28;

    #[doc(hidden)]
    fn steps_to_bools(val: u32) -> [bool; Self::CLUSTSIZE] {
        let mut bools: [bool; Self::CLUSTSIZE] = [false; Self::CLUSTSIZE];
        for i in 0..Self::CLUSTSIZE {
            bools[i] = ( (val as u32 >> i) & 1 ) == 1
        }
        bools
    }

    /// Create a new delay with all clusters set to 0
    pub fn new() -> HSDelay {
        HSDelay { bits: BitVec::repeat(false, Self::CLUSTERS*Self::CLUSTSIZE) }
    }

    /// Set the delay of a cluster to a specified amount of ns. This is in
    /// increments of 10 ns (100 MHz clock). Delays will be capped to
    /// 2^{28} × 10 ns.
    pub fn set_cluster_nanos(&mut self, cluster: DACCluster, val: u128) {
        let steps = if val > Self::MAX_DELAY {
            Self::MAX_DELAY / Self::RESOLUTION
        } else {
            val / Self::RESOLUTION
        } as u32;

        let bools = HSDelay::steps_to_bools(steps);

        let bits = self.bits.as_mut_bitslice();

        let cluster = cluster as usize;

        for i in 0..bools.len() {
            bits.set(Self::CLUSTSIZE*(Self::CLUSTERS-1-cluster) + i,
                bools[Self::CLUSTSIZE - 1 - i]);
        }

    }

    /// Same as [`set_cluster_nanos`][`Self::set_cluster_nanos`] but with a
    /// [`Duration`] argument instead.
    pub fn set_cluster_from_duration(&mut self, cluster: DACCluster, val: &Duration) {
        self.set_cluster_nanos(cluster, val.as_nanos());
    }
}

impl ToU32s for HSDelay {
    fn as_u32s(&self) -> Vec<u32> {
        self.bits.as_raw_slice().to_vec()
    }
}

/// High Speed Pulse attributes
///
/// [`PulseAttrs`] is used to specify the attributes of an high speed
/// pulsing operation (which is typically followed after an HSConfig one).
/// Essentially it is used to define which output DAC clusters are used.
///
/// The default constructor covers the most basic operation which is is a
/// typical low/high/low pulse. The additional function
/// [`new_with_params()`][`Self::new_with_params`] allows to provide more
/// arguments that handle two different usecases
///
/// (A) The pulse on this channel is lower than the baseline voltage used on
/// the other channels. This is quite common when doing differential biasing.
/// For instance if a 2.0 V pulse is applied between high and low this can be
/// split into a 1.0 V pulse on the high channel and -1.0 V pulse on the low
/// channel. In that case the `polarity` bit of the cluster the low channel
/// belongs to should be asserted.
///
/// (B) The pulse previously configured
/// with a [`HSConfig`][`crate::instructions::HSConfig`] instruction has a zero-width
/// pulse width. This is a special case that essentially means "pulse +
/// hold". In this case the additional arguments of
/// [`new_with_params()`][`Self::new_with_params`] define the behaviour of
/// ArC2. If `polarity` is `true` (high) then once the pulse will revert to
/// the `DAC+` value of the last [`SetDAC`][`crate::instructions::SetDAC`] instruction.
/// If `polarity` is `false` (low) then ArC2 will revert to `DAC-`. If instead
/// `cancel` is set to `true` then `polarity` will be ignored and ArC2 will
/// connect the channel to a 10 MΩ resistor.
///
/// ## Example
///
/// ```
/// use libarc2::registers::{PulseAttrs, ClusterMask, ToU32s};
///
/// // Create a high speed pulse attribute for operation on DAC Clusters 0 and 1.
/// let attrs = PulseAttrs::new(ClusterMask::CL0 | ClusterMask::CL1);
///
/// assert_eq!(attrs.as_u32s(), &[0x00030000]);
///
/// // Similar but with extra parameters
/// let attrs = PulseAttrs::new_with_params(ClusterMask::CL0, ClusterMask::CL0, ClusterMask::CL0);
///
/// assert_eq!(attrs.as_u32s(), &[0x00010101]);
/// ```
pub struct PulseAttrs {
    // cluster bitmask
    cluster: ClusterMask,
    // polarity flag
    // if polarity is 0 it means that the pulse direction
    // will be low V → high V → low V. Low and High V are set
    // as usual with the SetDAC command. Most of the times
    // polarity will be 0 unless a specific operation is required
    // in conjuction with a cluster configured with 0ns pulse
    // width. In this case if polarity is 1 the instrument will go
    // from whatever its state to DAC+. If polarity is 0 will
    // revert to DAC- instead.
    polarity: ClusterMask,
    // If cancel is 1 and pulse width is 0 the channel will be
    // connected to a 10 MΩ resistor instead of returning to
    // DAC+ or DAC- depending on the value of polarity.
    cancel: ClusterMask
}

impl PulseAttrs {

    /// Create a new pulse attribute register enabling the specified
    /// DAC clusters for a low/high/low operation.
    pub fn new(clusters: ClusterMask) -> Self {
        Self::new_with_params(clusters, ClusterMask::NONE, ClusterMask::NONE)
    }

    /// Provide additional arguments (see documentation for [`PulseAttrs`]).
    pub fn new_with_params(clusters: ClusterMask, polarity: ClusterMask, cancel: ClusterMask) -> Self {
        PulseAttrs {
            cluster: clusters,
            polarity: polarity,
            cancel: cancel
        }
    }
}

impl ToU32s for PulseAttrs {
    fn as_u32s(&self) -> Vec<u32> {
        let value = u32::from_be_bytes([0x00,
                                        self.cluster.bits(),
                                        self.polarity.bits(),
                                        self.cancel.bits()]);
        [value].to_vec()
    }
}


/// An FPGA memory address associated with an operation
///
/// This is practically a [`u32`] underneath and can be treated
/// as such. Typically is associated with an operation that would need
/// to output data into the FPGA's memory such as
/// [`CurrentRead`][`crate::instructions::CurrentRead`] or
/// [`VoltageRead`][`crate::instructions::VoltageRead`].
#[derive(FromPrimitive, ToPrimitive)]
pub struct Address(u32);

impl Address {
    pub fn new(addr: u32) -> Address {
        Address::from_u32(addr).unwrap()
    }
}

impl ToU32s for Address {
    fn as_u32s(&self) -> Vec<u32> {
        [self.0].to_vec()
    }
}


bitflags! {
    /// DAC channel selection register.
    ///
    /// [`DACMask`] is used to create a suitable bitmask for a given
    /// selection of channels. Output channels in ArC2 are organised
    /// in 8 clusters and there is also an additional auxilliary DAC
    /// used for internal configuration. Each cluster contains 8
    /// possible channels that can be toggled up to a total of 64.
    /// This is usually paired with the
    /// [`OpCode::UpdateDAC`][`super::opcode::OpCode::UpdateDAC`]
    /// opcode to set the voltages of a channel. All the DACs are
    /// organised in halves so if one wants to address channel
    /// 3 (zero-indexed) would have to toggle the bits that correspond
    /// to DAC0; 1st half ([`DACMask::CH00_03`]), whereas for channel 29
    /// that would have to be DAC3; 2nd half ([`DACMask::CH28_31`]).
    ///
    /// Although one can create a suitable bitmask manually this
    /// implementation provides some convenient functions that can
    /// be used instead of calculating which of 16 halves corresponds
    /// to a given channel. This is abstracted away under functions
    /// [`set_channel`][`DACMask::set_channel`] and
    /// [`unset_channel`][`DACMask::unset_channel`].
    ///
    /// ## Example
    /// ```
    /// use libarc2::registers::DACMask;
    ///
    /// // Create a new DAC bitmask
    /// let mut clusters = DACMask::NONE;
    ///
    /// clusters.set_channels(&[2, 3, 50, 61]);
    ///
    /// assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH48_51 |
    ///     DACMask::CH60_63);
    /// assert_eq!(clusters.as_u32(), 0x00009001);
    /// clusters.set_channel(12);
    /// assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH12_15 |
    ///     DACMask::CH48_51 | DACMask::CH60_63);
    /// assert_eq!(clusters.as_u32(), 0x00009009);
    ///
    /// clusters.unset_channel(61);
    /// assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH12_15 |
    ///     DACMask::CH48_51);
    /// assert_eq!(clusters.as_u32(), 0x00001009);
    ///
    /// clusters.clear();
    /// assert_eq!(clusters, DACMask::NONE);
    /// assert_eq!(clusters.as_u32(), 0x0);
    /// ```
    pub struct DACMask: u32 {
        /// No Flags; invalid state
        const NONE    = 0b00000000000000000000000000000000;
        /// DAC0; first half
        const CH00_03 = 0b00000000000000000000000000000001;
        /// DAC0; second half
        const CH04_07 = 0b00000000000000000000000000000010;
        /// DAC1; first half
        const CH08_11 = 0b00000000000000000000000000000100;
        /// DAC1; second half
        const CH12_15 = 0b00000000000000000000000000001000;
        /// DAC2; first half
        const CH16_19 = 0b00000000000000000000000000010000;
        /// DAC2; second half
        const CH20_23 = 0b00000000000000000000000000100000;
        /// DAC3; first half
        const CH24_27 = 0b00000000000000000000000001000000;
        /// DAC3; second half
        const CH28_31 = 0b00000000000000000000000010000000;
        /// DAC4; first half
        const CH32_35 = 0b00000000000000000000000100000000;
        /// DAC4; second half
        const CH36_39 = 0b00000000000000000000001000000000;
        /// DAC5; first half
        const CH40_43 = 0b00000000000000000000010000000000;
        /// DAC5; second half
        const CH44_47 = 0b00000000000000000000100000000000;
        /// DAC6; first half
        const CH48_51 = 0b00000000000000000001000000000000;
        /// DAC6; second half
        const CH52_55 = 0b00000000000000000010000000000000;
        /// DAC7; first half
        const CH56_59 = 0b00000000000000000100000000000000;
        /// DAC7; second half
        const CH60_63 = 0b00000000000000001000000000000000;
        /// AUX DAC0
        const AUX0    = 0b00000000000000010000000000000000;
        /// AUX DAC1
        const AUX1    = 0b00000000000000100000000000000000;
        /// All channels of DAC0
        const DAC0    = Self::CH00_03.bits | Self::CH04_07.bits;
        /// All channels of DAC1
        const DAC1    = Self::CH08_11.bits | Self::CH12_15.bits;
        /// All channels of DAC2
        const DAC2    = Self::CH16_19.bits | Self::CH20_23.bits;
        /// All channels of DAC3
        const DAC3    = Self::CH24_27.bits | Self::CH28_31.bits;
        /// All channels of DAC4
        const DAC4    = Self::CH32_35.bits | Self::CH36_39.bits;
        /// All channels of DAC5
        const DAC5    = Self::CH40_43.bits | Self::CH44_47.bits;
        /// All channels of DAC6
        const DAC6    = Self::CH48_51.bits | Self::CH52_55.bits;
        /// All channels of DAC7
        const DAC7    = Self::CH56_59.bits | Self::CH60_63.bits;
        /// All channels
        const ALL = Self::CH00_03.bits | Self::CH04_07.bits |
                    Self::CH08_11.bits | Self::CH12_15.bits |
                    Self::CH16_19.bits | Self::CH20_23.bits |
                    Self::CH24_27.bits | Self::CH28_31.bits |
                    Self::CH32_35.bits | Self::CH36_39.bits |
                    Self::CH40_43.bits | Self::CH44_47.bits |
                    Self::CH48_51.bits | Self::CH52_55.bits |
                    Self::CH56_59.bits | Self::CH60_63.bits;
    }
}

impl DACMask {

    /// Enable the specified channel.
    pub fn set_channel(&mut self, chan: usize) {
        self.set_channels(&[chan]);
    }

    /// Disable the specified channel.
    pub fn unset_channel(&mut self, chan: usize) {
        self.unset_channels(&[chan]);
    }

    /// Enable the specified channels.
    pub fn set_channels(&mut self, chans: &[usize]) {
        for c in chans {
            *self |= consts::CHANMAP[*c];
        }
    }

    /// Disable the specified channels.
    pub fn unset_channels(&mut self, chans: &[usize]) {
        for c in chans {
            *self &= !consts::CHANMAP[*c];
        }
    }

    /// Clear all channels. This is effectively [`DACMask::NONE`].
    pub fn clear(&mut self) {
        self.bits = 0;
    }

    /// Get the representation of this bitmas ask u32.
    pub fn as_u32(&self) -> u32 {
        u32::from(self)
    }
}

impl From<&DACMask> for u32 {
    fn from(clusters: &DACMask) -> u32 {
        clusters.bits() as u32
    }
}

impl ToU32s for DACMask {
    fn as_u32s(&self) -> Vec<u32> {
        [self.as_u32()].to_vec()
    }
}

#[cfg(test)]
mod dacmask_tests {
    use super::DACMask;

    #[test]
    fn test_dac_mask() {

        let mut clusters = DACMask::NONE;

        clusters.set_channels(&[2, 3, 50, 61]);
        assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH48_51 |
            DACMask::CH60_63);
        assert_eq!(clusters.as_u32(), 0x00009001);

        clusters.set_channel(12);
        assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH12_15 |
            DACMask::CH48_51 | DACMask::CH60_63);
        assert_eq!(clusters.as_u32(), 0x00009009);

        clusters.unset_channel(61);
        assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH12_15 |
            DACMask::CH48_51);
        assert_eq!(clusters.as_u32(), 0x00001009);

        clusters.clear();
        assert_eq!(clusters, DACMask::NONE);
        assert_eq!(clusters.as_u32(), 0x0);

    }
}


/// Channel configurations currently supported by ArC2.
/// Use these with [`ChannelConf`] to control
/// individual ArC2 channels.
#[derive(Clone, Copy, FromPrimitive, Debug)]
#[repr(u8)]
pub enum ChannelState {
    /// Keep current state
    Maintain = 0b00,
    /// Open channel; channel will not be connected
    /// to anything.
    Open = 0b01,
    /// Channel is set for arbitrary voltage operation
    VoltArb = 0b10,
    /// High-Speed pulse channel
    HiSpeed = 0b11,
}

impl ChannelState {

    fn as_bools(&self) -> [bool; consts::CHANCONFSIZE] {

        let mut bools: [bool; consts::CHANCONFSIZE] = [false; consts::CHANCONFSIZE];

        for i in 0..consts::CHANCONFSIZE {
            bools[i] = ((*self as u8 >> i) & 1) == 1
        }

        bools
    }

    fn from_bools(bools: &[bool; consts::CHANCONFSIZE]) -> ChannelState {
        let mut bitarr = bitarr![u8, Msb0; 0; 8];

        for i in 0..consts::CHANCONFSIZE {
           bitarr.set(8-consts::CHANCONFSIZE+i, bools[i])
        }

        let value: &[u8] = bitarr.as_raw_slice();
        ChannelState::from_u8(value[0] as u8).unwrap()

    }

    fn from_bitslice(bools: &BitSlice<u32, Msb0>) -> Result<ChannelState, Error> {

        let len: usize;

        if bools.len() < consts::CHANCONFSIZE {
            return Err(Error::SliceTooSmall);
        }

        if bools.len() > 8 {
            len = 8;
        } else {
            len = bools.len()
        }

        let mut bitarr = bitarr![u8, Msb0; 0; 8];

        for i in 0..len {
           bitarr.set(8-len+i, bools[i])
        }

        let value: &[u8] = bitarr.as_raw_slice();
        Ok(ChannelState::from_u8(value[0] as u8).unwrap())
    }

}

impl From<&[bool; consts::CHANCONFSIZE]> for ChannelState {
    fn from(bools: &[bool; consts::CHANCONFSIZE]) -> ChannelState {
        ChannelState::from_bools(&bools)
    }
}

/// A set of DAC channel output configuration.
///
/// A `ChannelConf` is currently designed for 3 bits per channel for
/// a total of 64 channels (192-bits). The underlying implementation uses a
/// [`BitVec`][bitvec::vec::BitVec] storing MSB bits and backed by [`u32`]s.
/// This matches the structure that ArC2 is expecting for the channel
/// configuration. `ChannelConf` is typically paired with
/// [`OpCode::UpdateChannel`].
///
/// To create a new register call [`ChannelConf::new()`] with the
/// desired number of channels. For typical ArC2 scenarios this should be 64.
/// By default the register is populated with zeros (which is an invalid
/// status for ArC2) and must be configured appropriately by setting the
/// invididual channels to a [`ChannelState`] value. The register will take
/// care of flipping the correct bits in the internal representation in order
/// to have a consistent 32bit representation.
///
/// **See also**: [`ChannelState`] for the available channel configurations.
///
/// ## Examples
///
/// ```
/// use libarc2::registers::{ChannelConf, ChannelState, ToU32s};
///
/// // Initialise a new channel configuration register
/// let mut reg = ChannelConf::new();
///
/// // Number of allocated channels
/// let nchan = reg.len();
///
/// // Set channel 31 to High speed pulsing mode
/// reg.set(31, ChannelState::HiSpeed);
///
/// // Set all channels to arbitrary voltage operation
/// reg.set_all(ChannelState::VoltArb);
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
/// for value in reg.as_u32s() {
///    println!("0x{:x}", value);
/// }
/// ```
pub struct ChannelConf {
    bits: BitVec<u32, Msb0>,
}

impl ChannelConf {

    /// Create a new register with 64 channels.
    /// This will be expanded to `CHANSIZE` × channels
    /// in the internal bit vector representation.
    pub fn new() -> ChannelConf {
        // CHANSIZE bits for each channel
        let size = consts::NCHANS * consts::CHANCONFSIZE;
        let vec: BitVec<u32, Msb0> = BitVec::repeat(false, size);

        ChannelConf { bits: vec }
    }

    /// Create a new register with 64 channels and set all channels to
    /// the specified state.
    pub fn new_with_state(state: ChannelState) -> ChannelConf {
        let mut conf = ChannelConf::new();
        conf.set_all(state);

        conf
    }

    /// Set a channel to a [`ChannelState`] value
    pub fn set(&mut self, idx: usize, val: ChannelState) {
        let bits = self.bits.as_mut_bitslice();
        let bools = val.as_bools();

        for i in 0..bools.len() {
            let bitidx = consts::NCHANS*consts::CHANCONFSIZE -
                (consts::CHANCONFSIZE*idx+i) - 1;
            bits.set(bitidx, bools[i]);
        }
    }

    /// Get the [`state`][`ChannelState`] of a channel
    pub fn get(&self, idx: usize) -> ChannelState {
        let from = consts::NCHANS*consts::CHANCONFSIZE - (consts::CHANCONFSIZE*idx+1) - 1;
        let to = consts::NCHANS*consts::CHANCONFSIZE - (consts::CHANCONFSIZE*(idx));

        let v = &self.bits[from..to];

        ChannelState::from_bitslice(v).unwrap()
    }

    /// Get the number of allocated channels
    pub fn len(&self) -> usize {
        // len is always a multiple of CHANSIZE
        self.bits.len() / consts::CHANCONFSIZE
    }

    /// Set the status of all channels to the same value
    pub fn set_all(&mut self, val: ChannelState) {
        let nchannels = self.len();

        for i in 0..nchannels {
            self.set(i, val);
        }
    }

    /// Get the serialisable format of this register specified
    /// as a slice of whatever the internal representation is. This
    /// is presently a [`u32`] as this is the size of words that
    /// ArC2 is expecting as input.
    pub fn as_slice(&self) -> &[u32] {
        self.bits.as_raw_slice()
    }
}

impl ToU32s for ChannelConf {
    fn as_u32s(&self) -> Vec<u32> {
        let bits = self.bits.as_raw_slice();
        bits.to_vec()
    }
}

#[doc(hidden)]
pub struct ChannelConfIterator<'a> {
    register: &'a ChannelConf,
    index: usize,
}

impl<'a> IntoIterator for &'a ChannelConf {

    type Item = ChannelState;
    type IntoIter = ChannelConfIterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        ChannelConfIterator {
            register: self,
            index: 0,
        }
    }

}

impl<'a> Iterator for ChannelConfIterator<'a> {

    type Item = ChannelState;

    fn next(&mut self) -> Option<ChannelState> {
        if self.index >= self.register.len() {
            return None;
        }

        let v = self.register.get(self.index);
        self.index += 1;
        Some(v)
    }

}


#[cfg(test)]
mod channelconf_tests {

    use super::{ChannelConf, ChannelState};
    use crate::registers::ToU32s;
    use assert_matches::assert_matches;

    #[test]
    fn get_channel() {
        let mut v = ChannelConf::new();
        v.set(50, ChannelState::VoltArb);
        let res = v.get(50);
        assert_matches!(res, ChannelState::VoltArb);

        v.set(0, ChannelState::Open);
        let res = v.get(0);
        assert_matches!(res, ChannelState::Open);

        v.set(63, ChannelState::HiSpeed);
        let res = v.get(63);
        assert_matches!(res, ChannelState::HiSpeed);
    }

    #[test]
    fn channel_len() {
        let v = ChannelConf::new();
        assert_eq!(v.len(), 64);
    }

    #[test]
    fn bools_to_status() {
        let status1 = ChannelState::from(&[true, false]);
        assert_matches!(status1, ChannelState::VoltArb);

        let status2 = ChannelState::from(&[false, true]);
        assert_matches!(status2, ChannelState::Open);
    }

    #[test]
    fn all_channel_test() {
        let mut v = ChannelConf::new();
        v.set_all(ChannelState::VoltArb);

        for channel in &v {
            assert_matches!(channel, ChannelState::VoltArb);
        }

        let slice = v.as_u32s();

        assert_eq!(slice[0], 0xaaaaaaaa);
        assert_eq!(slice[1], 0xaaaaaaaa);
        assert_eq!(slice[2], 0xaaaaaaaa);
        assert_eq!(slice[3], 0xaaaaaaaa);
    }

    #[test]
    fn new_all_channels() {
        let v = ChannelConf::new_with_state(ChannelState::VoltArb);

        for channel in &v {
            assert_matches!(channel, ChannelState::VoltArb);
        }

        let slice = v.as_u32s();

        assert_eq!(slice[0], 0xaaaaaaaa);
        assert_eq!(slice[1], 0xaaaaaaaa);
        assert_eq!(slice[2], 0xaaaaaaaa);
        assert_eq!(slice[3], 0xaaaaaaaa);
    }
}


/// Current source configuration.
///
/// When a current source is in operation this enum
/// specifies its operation state and it is part of the
/// [`SourceConf`] register.
#[derive(Clone, Copy, FromPrimitive, ToPrimitive, Debug)]
#[repr(u8)]
pub enum CurrentSourceState {
    /// Maintain current status (default)
    Maintain   = 0b00000000,
    /// Disconnect current source
    Open       = 0b00000001,
    /// Arbitrary voltage operation
    VoltageArb = 0b00000010,
    /// High speed voltage pulse operation
    HiSpeed    = 0b00000011
}


/// Output source configuration
///
/// This register configures the status of the output source and
/// it's usually followed by a
/// [`ChannelConf`][`crate::registers::ChannelConf`].
/// There are two things that are specified by this register. The
/// *output digipot* and the state of the *current source*.
pub struct SourceConf {
    bits: BitVec<u32, Msb0>
}

impl SourceConf {

    /// Create a new source configuration register. This will
    /// initialise the digipot to a safe value (`0x1CD` or roughly
    /// 11 kΩ).
    pub fn new() -> SourceConf {
        let mut vec: BitVec<u32, Msb0> = BitVec::repeat(false, 32);
        let bits = vec.as_mut_bitslice();
        bits[0..10].store(0x1CD as u16);

        SourceConf { bits: vec }
    }

    /// Set digipot raw value. This is clamped between
    /// 0x000 and 0x300 to keep the instrument safe.
    pub fn set_digipot(&mut self, val: u16) {
        let actual_val;
        if val > 0x300 {
            actual_val = 0x300;
        } else {
            actual_val = val;
        }

        let bits = self.bits.as_mut_bitslice();
        bits[0..10].store(actual_val);
    }

    /// Get the current digipot raw value.
    pub fn get_digipot(&self) -> u16 {
        self.bits[0..10].load()
    }

    /// Set state output. See [`CurrentSourceState`] for possible
    /// values.
    pub fn set_cursource_state(&mut self, val: CurrentSourceState) {
        self.bits[28..32].store(val as u8);
    }

    /// Retrieves the current source state stores in this register.
    pub fn get_cursource_state(&self) -> CurrentSourceState {
        let val = self.bits[24..32].load::<u8>();
        CurrentSourceState::from_u8(val).unwrap()
    }
}

impl ToU32s for SourceConf {
    fn as_u32s(&self) -> Vec<u32> {
        let bits = self.bits.as_raw_slice();
        bits.to_vec()
    }
}

#[cfg(test)]
mod sourceconftests {

    use super::{SourceConf, CurrentSourceState, ToU32s};
    use assert_matches::assert_matches;

    #[test]
    fn test_sourceconf() {
        let mut c = SourceConf::new();
        c.set_digipot(0x200);
        assert_eq!(c.get_digipot(), 0x200);

        let slice = c.as_u32s();
        assert_eq!(slice[0], 0x80000000);

        c.set_cursource_state(CurrentSourceState::HiSpeed);
        assert_matches!(c.get_cursource_state(),
            CurrentSourceState::HiSpeed);

        let slice = c.as_u32s();
        assert_eq!(slice[0], 0x80000003);

        // Digipot must never exceed 0x300
        c.set_digipot(0x400);
        assert_eq!(c.get_digipot(), 0x300);

    }
}


const DACVZERO: u32 = 0x80008000;

/// Voltage configuration for DACs
///
/// This struct is used to configure the output voltages of the on-board
/// DACs. DAC voltage are represented with a `u16` value, `0x0000` being
/// the lowest possible voltage and `0xFFFF` being the highest. Assuming
/// the absolute maximum voltages are the same for two polarities value
/// `0x8000` is 0.0 volts which is the default value used when creating
/// this register.
///
/// DACs have two outputs, Vhigh and Vlow and typically Vhigh > Vlow in
/// most circumstances. In normal measurement scenarios both output
/// voltages should be the same and the user is advised to use the
/// [`DACVoltage::set()`] function to set a voltage for a DAC. By
/// default a new `DACVoltage` has four different channels as this
/// is exactly the number of channels that fit in 1/2 of a DAC cluster.
/// See documentation of [`DACMask`][`crate::registers::DACMask`] for
/// more details on that.
///
/// ## Examples
/// ```
/// use libarc2::registers::DACVoltage;
///
/// // Make a new register
/// let mut reg0 = DACVoltage::new();
/// // Set both values of the second channel
/// reg0.set(1, 0x8534);
/// assert_eq!(reg0.get(1), (0x8534, 0x8534));
///
/// let mut reg1 = DACVoltage::new();
/// // Set the high voltage of the third channel
/// reg1.set_high(2, 0x8534);
/// assert_eq!(reg1.get(2), (0x8000, 0x8534));
/// ```
pub struct DACVoltage {
    values: Vec<u32>
}

impl DACVoltage {

    /// Create a new register with four channels
    pub fn new() -> DACVoltage {
        Self::new_with_size_and_voltage(4, DACVZERO)
    }

    /// Create a new register with all four channels set at
    /// the specified levels
    ///
    /// ```
    /// use libarc2::registers::{DACVoltage, ToU32s};
    ///
    /// let reg = DACVoltage::new_at_levels(0xaaaa, 0xbbbb);
    ///
    /// for i in 0..4 {
    ///     assert_eq!(reg.get(i), (0xaaaa, 0xbbbb));
    /// }
    ///
    /// assert_eq!(reg.as_u32s(), &[0xbbbbaaaa, 0xbbbbaaaa,
    ///     0xbbbbaaaa, 0xbbbbaaaa]);
    /// ```
    pub fn new_at_levels(low: u16, high: u16) -> DACVoltage {
        let voltage: u32 =
            ((high as u32) << 16) | ((low as u32) & 0xFFFF);
        Self::new_with_size_and_voltage(4, voltage)
    }

    fn new_with_size_and_voltage(size: usize, volt: u32) -> DACVoltage {
        let mut vec: Vec<u32> = Vec::with_capacity(size);

        for _ in 0..size {
            vec.push(volt);
        }

        DACVoltage { values: vec }
    }

    pub(crate) fn from_raw_values(values: &[u32]) -> DACVoltage {
        DACVoltage { values: values.to_vec() }
    }

    /// Set the Vhigh value of a specified channel index
    pub fn set_high(&mut self, idx: usize, voltage: u16) {
        self.values[idx] = (voltage as u32) << 16 |
            (self.values[idx] & 0xFFFF);
    }

    /// Get the Vhigh value of a specified channel index
    pub fn get_high(&self, idx: usize) -> u16 {
        ((self.values[idx] & 0xFFFF0000) >> 16) as u16
    }

    /// Set the Vlow value of a specified channel index
    pub fn set_low(&mut self, idx: usize, voltage: u16) {
        self.values[idx] = (voltage as u32) <<  0 |
            (self.values[idx] & 0xFFFF0000);
    }

    /// Get the Vlow value of a specified channel index
    pub fn get_low(&self, idx: usize) -> u16 {
        (self.values[idx] & 0xFFFF) as u16
    }

    /// Set both Vhigh and Vlow of a specified channel index
    pub fn set(&mut self, idx: usize, voltage: u16) {
        self.set_low(idx, voltage);
        self.set_high(idx, voltage);
    }

    /// Get both Vhigh and Vlow of a specified channel index.
    /// The first `u16` of the tuple is Vlow, the second Vhigh.
    pub fn get(&self, idx: usize) -> (u16, u16) {
        (self.get_low(idx), self.get_high(idx))
    }

    /// Number of configured channels
    pub fn len(&self) -> usize {
        self.values.len()
    }

}

impl ToU32s for DACVoltage {
    fn as_u32s(&self) -> Vec<u32> {
        self.values.clone()
    }
}

#[cfg(test)]
mod dacvoltage_tests {

    use super::{DACVoltage};

    #[test]
    fn dacvoltage_new() {
        let v = DACVoltage::new();
        for value in v.values {
            assert_eq!(value, 0x80008000);
        }
    }

    #[test]
    fn dacvoltage_set_high() {
        let mut v = DACVoltage::new();
        v.set_high(3, 0xA0A0);

        assert_eq!(v.values[3], 0xA0A08000);
        assert_eq!(v.get_high(3), 0xA0A0);
        assert_eq!(v.get_low(3), 0x8000);
    }

    #[test]
    fn dacvoltage_set_low() {
        let mut v = DACVoltage::new();
        v.set_low(2, 0x90F3);

        assert_eq!(v.values[2], 0x800090F3);
        assert_eq!(v.get_high(2), 0x8000);
        assert_eq!(v.get_low(2), 0x90F3);
    }

    #[test]
    fn dacvoltage_set_both() {
        let mut v = DACVoltage::new();

        v.set(1, 0x8534);
        assert_eq!(v.values[1], 0x85348534);
        assert_eq!(v.get_low(1), 0x8534);
        assert_eq!(v.get_high(1), 0x8534);
        assert_eq!(v.get(1), (0x8534, 0x8534));
    }

}



/// A generic bitmask of the specified word size
pub struct U32Mask<T> {
    _words: T,
    bits: BitVec<u32, Msb0>,
}


impl<T: wordreg::WordSize> U32Mask<T> {

    /// Set a channel to enabled (`true`) or disabled (`false`).
    pub fn set_enabled(&mut self, idx: usize, status: bool) {
        let len = self.bits.len();
        let bits = self.bits.as_mut_bitslice();
        bits.set(len-1-idx, status)
    }

    /// Get the state of a channel, enabled (`true`) or disabled (`false`).
    pub fn get_enabled(&self, idx: usize) -> bool {
        let len = self.bits.len();
        self.bits[len-1-idx]
    }

    /// Get the number of allocated channels.
    pub fn len(&self) -> usize {
        self.bits.len()
    }

    /// Set all channels to enabled (`true`) or disabled (`false`).
    pub fn set_enabled_all(&mut self, status: bool) {
        let len = self.bits.len();
        let bits = self.bits.as_mut_bitslice();
        for i in 0..len {
            bits.set(len-1-i, status)
        }
    }

    /// Toggle selected channel.
    pub fn toggle(&mut self, idx: usize) {
        self.set_enabled(idx, !self.get_enabled(idx));
    }

    /// Get the serialisable format of this register specified
    /// as a slice of whatever the internal representation is. This
    /// is presently a [`u32`] as this is the size of words that
    /// ArC2 is expecting as input.
    pub fn as_slice(&self) -> &[u32] {
        self.bits.as_raw_slice()
    }

}

impl<T: wordreg::WordSize> ToU32s for U32Mask<T> {
    fn as_u32s(&self) -> Vec<u32> {
        self.bits.as_raw_slice().to_vec()
    }
}

impl U32Mask<wordreg::Wx1> {
    pub fn new() -> U32Mask<wordreg::Wx1> {
        let vec: BitVec<u32, Msb0> = BitVec::repeat(false, wordreg::Wx1::WORDS*32);
        U32Mask { _words: wordreg::Wx1{}, bits: vec }
    }
}

impl U32Mask<wordreg::Wx2> {
    pub fn new() -> U32Mask<wordreg::Wx2> {
        let vec: BitVec<u32, Msb0> = BitVec::repeat(false, wordreg::Wx2::WORDS*32);
        U32Mask { _words: wordreg::Wx2{}, bits: vec }
    }
}

impl U32Mask<wordreg::Wx3> {
    pub fn new() -> U32Mask<wordreg::Wx3> {
        let vec: BitVec<u32, Msb0> = BitVec::repeat(false, wordreg::Wx3::WORDS*32);
        U32Mask { _words: wordreg::Wx3{}, bits: vec }
    }
}

impl U32Mask<wordreg::Wx4> {
    pub fn new() -> U32Mask<wordreg::Wx4> {
        let vec: BitVec<u32, Msb0> = BitVec::repeat(false, wordreg::Wx4::WORDS*32);
        U32Mask { _words: wordreg::Wx4{}, bits: vec }
    }
}


/// Channel configuration bitmask.
///
/// An `ChanMask` is used to select one or more channels. Essentially
/// it defines which channels the Read Current, Read Voltage or Amp Prep
/// operations be applied to.
///
/// See [`U32Mask`][`crate::registers::U32Mask`] for details and
/// methods.
///
/// ## Example
/// ```
/// use libarc2::registers::{ChanMask, ToU32s};
///
/// let mut chan = ChanMask::new();
///
/// // set some channels
/// chan.set_enabled(31, true);
/// chan.set_enabled(0, true);
/// chan.set_enabled(62, true);
///
/// assert_eq!(chan.get_enabled(31), true);
///
/// // u32 representation
/// assert_eq!(chan.as_u32s(), &[0x40000000, 0x80000001]);
/// ```
pub type ChanMask = U32Mask<wordreg::Wx2>;

/// Averaging for read operations
#[derive(Clone, Copy, FromPrimitive, ToPrimitive, Debug)]
#[repr(u32)]
pub enum Averaging {
    Enabled = 1,
    Disabled = 0
}

impl ToU32s for Averaging {
    fn as_u32s(&self) -> Vec<u32> {
        vec![*self as u32; 1]
    }
}


#[cfg(test)]
mod adcmask_tests {
    use super::ChanMask;
    use crate::registers::ToU32s;

    #[test]
    fn get_set_channel() {
        let mut v = ChanMask::new();
        v.set_enabled(31, true);
        v.set_enabled(0, true);
        v.set_enabled(62, true);

        assert_eq!(v.get_enabled(31), true);
        assert_eq!(v.get_enabled(0), true);
        assert_eq!(v.get_enabled(62), true);

        v.set_enabled(62, false);
        assert_eq!(v.get_enabled(62), false);

    }

    #[test]
    fn get_set_all_channels() {
        let mut v = ChanMask::new();
        v.set_enabled_all(true);

        for c in 0..v.len() {
            assert_eq!(v.get_enabled(c), true);
        }

    }

    #[test]
    fn repr() {
        let mut v = ChanMask::new();
        v.set_enabled(31, true);
        v.set_enabled(0, true);
        v.set_enabled(62, true);

        assert_eq!(&v.as_u32s(), &[0x40000000, 0x80000001]);

    }

    #[test]
    fn toggle() {
        let mut v = ChanMask::new();
        v.set_enabled(31, true);
        v.set_enabled(0, true);
        v.set_enabled(62, true);

        assert_eq!(v.get_enabled(31), true);

        v.toggle(31);
        assert_eq!(v.get_enabled(31), false);
    }

}


/// I/O channel configuration bitmask.
///
/// An `IOMask` is used to configure the I/O channels of ArC2. Essentially
/// it defines which channels will be configured during the Update I/O
/// instruction.
///
/// See [`U32Mask`][`crate::registers::U32Mask`] for details and
/// methods.
///
/// ## Example
/// ```
/// use libarc2::registers::{IOMask, ToU32s};
///
/// let mut chan = IOMask::new();
///
/// // set some channels
/// chan.set_enabled(31, true);
/// chan.set_enabled(0, true);
///
/// assert_eq!(chan.get_enabled(31), true);
///
/// // u32 representation
/// assert_eq!(chan.as_u32s(), &[0x80000001]);
/// ```
pub type IOMask = U32Mask<wordreg::Wx1>;


#[cfg(test)]
mod iomask_tests {
    use super::IOMask;
    use crate::registers::ToU32s;

    #[test]
    fn get_set_channel() {
        let mut v = IOMask::new();
        v.set_enabled(31, true);
        v.set_enabled(0, true);

        assert_eq!(v.get_enabled(31), true);
        assert_eq!(v.get_enabled(0), true);

        v.set_enabled(31, false);
        assert_eq!(v.get_enabled(31), false);

    }

    #[test]
    fn get_set_all_channels() {
        let mut v = IOMask::new();
        v.set_enabled_all(true);

        for c in 0..v.len() {
            assert_eq!(v.get_enabled(c), true);
        }

    }

    #[test]
    fn repr() {
        let mut v = IOMask::new();
        v.set_enabled(31, true);
        v.set_enabled(0, true);

        assert_eq!(&v.as_u32s(), &[0x80000001]);

    }

    #[test]
    fn toggle() {
        let mut v = IOMask::new();
        v.set_enabled(31, true);
        v.set_enabled(0, true);

        assert_eq!(v.get_enabled(31), true);

        v.toggle(31);
        assert_eq!(v.get_enabled(31), false);
    }

}



/// IO enable configuration register.
///
/// Use this register to set whether I/Os are enabled and what's
/// their direction.
///
/// This register is laid out as a 4 bit bitmask plus one EN bit.
///
/// ```text
/// 0b[X][ CH3 CH2 CH1 CH0 ]
///    |    |   |   |   |
///    |    +---+---+---+ --- Set direction: LOW input; high OUPUT
///    |
///    + --- Set low to enable
/// ```
///
/// ## Example
/// ```
/// use libarc2::registers::{IOEnable, ToU32s};
///
/// // all IOs disabled and set as output: 0b[1][1111]
/// let none = IOEnable::new();
///
/// assert_eq!(none.as_u32s(), &[0x1f]);
///
/// // all IOs enabled and set as output: 0b[0][1111]
/// let all_outputs = IOEnable::all_output();
///
/// assert_eq!(all_outputs.as_u32s(), &[0xf]);
/// ```
pub struct IOEnable {
    bits: BitVec<u32, Lsb0>,
}

impl IOEnable {
    const LEN: usize = 5;

    /// Create a new `IOEnable`. IOs are OFF and set as outputs
    pub fn new() -> IOEnable {
        let vec: BitVec<u32, Lsb0> = BitVec::repeat(false, Self::LEN);
        let mut io = IOEnable { bits: vec };
        io.set_all_outputs(true);

        // IO is active low so we must set it to 1
        io.set_enabled(false);

        io
    }

    /// Create a new `IOEnable` with all IOs enabled and set to output
    pub fn all_output() -> IOEnable {
        let vec: BitVec<u32, Lsb0> = BitVec::repeat(false, Self::LEN);
        let mut io = IOEnable { bits: vec };
        io.set_all_outputs(true);

        io
    }

    /// Create a new `IOEnable` with all IOs enabled and set to input
    pub fn all_input() -> IOEnable {
        let vec: BitVec<u32, Lsb0> = BitVec::repeat(false, Self::LEN);
        let mut io = IOEnable { bits: vec };
        io.set_all_outputs(false);

        io
    }

    /// Toggle the enable bit
    pub fn set_enabled(&mut self, status: bool) {
        // IO Enable is active low
        self.set_output(Self::LEN-1, !status);
    }

    pub fn is_enabled(&self) -> bool {
        !self.bits[Self::LEN-1]
    }

    /// Set an I/O cluster as output (`true`) or not (`false`)
    ///
    /// ```text
    /// 0b[X][ CH3 CH2 CH1 CH0 ]
    ///    |    |   |   |   |
    ///    |    +---+---+---+ --- Set direction: LOW input; high OUPUT
    ///    |
    ///    + --- Set low to enable
    /// ```
    pub fn set_output(&mut self, idx: usize, status: bool) {
        self.bits.set(idx, status);
    }

    /// Set all I/O clusters as output (`true`) or not (`false`)
    pub fn set_all_outputs(&mut self, status: bool) {
        for i in 0..(Self::LEN-1) {
            self.set_output(i, status);
        }
    }

}

impl ToU32s for IOEnable {
    fn as_u32s(&self) -> Vec<u32> {
        self.bits.as_raw_slice().to_vec()
    }
}

#[cfg(test)]
mod ioenable_tests {
    use super::{IOEnable, ToU32s};

    #[test]
    fn ioenable_new() {
        // 0b[ 1 ] [ 1111 ]
        //     ^     ^^^^
        //     |     ||||
        //     |     ++++----- all four clusters as outputs...
        //     +-- ... and disabled (active low)
        assert_eq!(IOEnable::new().as_u32s(), &[0x1f]);
    }

    #[test]
    fn ioenable_all() {
        // 0b[ 0 ] [ 1111 ]
        //     ^     ^^^^
        //     |     ||||
        //     |     ++++----- all four clusters as outputs...
        //     +-- ... and enabled (active low)
        assert_eq!(IOEnable::all_output().as_u32s(), &[0xf]);
    }


    #[test]
    fn ioenable_new_mut() {
        let mut io = IOEnable::new();
        io.set_enabled(true);
        io.set_output(0, true);
        io.set_output(1, true);
        io.set_output(2, false);
        io.set_output(3, true);

        assert_eq!(io.as_u32s(), &[0xb]);

        io.set_output(2, true);
        assert_eq!(io.as_u32s(), &[0xf]);
    }
}
