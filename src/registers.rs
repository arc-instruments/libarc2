//! Data that can be serialised into ArC2 instructions.
//!
//! A register is a piece of u32-encodable information that can is used
//! to form one or more instructions that ArC2 can parse. All registers
//! are expected to implement the [`ToU32s`][`crate::registers::ToU32s`]
//! trait that converts them into a serialisable `Vec<u32>`. This can be
//! then be processed by ArC2.

use std::collections::HashMap;
use std::time::Duration;
use num_derive::{FromPrimitive, ToPrimitive};
use bitvec::prelude::*;
use bitflags::bitflags;
use num_traits::{ToPrimitive, FromPrimitive};
use thiserror::Error;
use std::ops::{BitAnd, BitXor};
use std::iter::zip;

mod wordreg {

    /// A trait denoting a word size; ie how many words
    /// a register is using.
    pub trait WordSize {
        const WORDS: usize;
    }

    /// One word
    #[derive(Clone, Debug, PartialEq)]
    pub struct Wx1;
    impl WordSize for Wx1 {
        const WORDS: usize = 1;
    }

    /// Two words
    #[derive(Clone, Debug, PartialEq)]
    pub struct Wx2;
    impl WordSize for Wx2 {
        const WORDS: usize = 2;
    }

    /// Three words
    #[derive(Clone, Debug, PartialEq)]
    pub struct Wx3;
    impl WordSize for Wx3 {
        const WORDS: usize = 3;
    }

    /// Four words
    #[derive(Clone, Debug, PartialEq)]
    pub struct Wx4;
    impl WordSize for Wx4 {
        const WORDS: usize = 4;
    }

    /// Five words
    #[derive(Clone, Debug, PartialEq)]
    pub struct Wx5;
    impl WordSize for Wx5 {
        const WORDS: usize = 5;
    }

    /// Six words
    #[derive(Clone, Debug, PartialEq)]
    pub struct Wx6;
    impl WordSize for Wx6 {
        const WORDS: usize = 6;
    }

    /// Seven words
    #[derive(Clone, Debug, PartialEq)]
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

    pub(crate) const DACHCLUSTERMAP: [DACMask; 16] = [
        DACMask::CH00_03, DACMask::CH04_07, DACMask::CH08_11, DACMask::CH12_15,
        DACMask::CH16_19, DACMask::CH20_23, DACMask::CH24_27, DACMask::CH28_31,
        DACMask::CH32_35, DACMask::CH36_39, DACMask::CH40_43, DACMask::CH44_47,
        DACMask::CH48_51, DACMask::CH52_55, DACMask::CH56_59, DACMask::CH60_63,
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
    pub(super) const AUXNCHANS: usize = 16;
    pub(super) const NCHANS: usize = 64;
    // 2 bits per channel for range config (upper and lower voltages)
    pub(super) const RANGCONFSIZE: usize = 2;
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
    /// DAC Range
    DACRange       = 0x00004000,
    /// OpAmp preparation
    AmpPrep        = 0x00010000
}

impl ToU32s for OpCode {
    fn as_u32s(&self) -> Vec<u32> {
        [*self as u32].to_vec()
    }
}


/// Auxiliary DAC function
///
/// This is typically used with
/// [`Instrument::config_aux_channels`][`crate::instrument::Instrument::config_aux_channels()`] to
/// set the configuration for the auxiliary DACs that manage the selectors and the current source
/// operation. Although logic level is also adjusted via the auxiliary DACs there is a dedicated
/// function for this operation and it is not done via the
/// [`Instrument::config_aux_channels`][`crate::instrument::Instrument::config_aux_channels()`]
/// route.
#[derive(Clone, Copy, FromPrimitive, ToPrimitive)]
#[repr(usize)]
pub enum AuxDACFn {
    /// Selector circuit pulls down to this voltage
    SELL = 0,
    /// Selector circuit pulls up to this voltage
    SELH = 1,
    /// Arbitrary power supply for DUTs - Max current 100 mA
    ARB4 = 2,
    /// Arbitrary power supply for DUTs - Max current 100 mA
    ARB3 = 3,
    /// Arbitrary power supply for DUTs - Max current 100 mA
    ARB1 = 4,
    /// Arbitrary power supply for DUTs - Max current 100 mA
    ARB2 = 5,
    /// Reference voltage that the current source sources/sinks
    /// current from/to. There should be a ≥3 V headroom between
    /// CREF and the expected operating point of the current source
    /// Must be within 1.5 V of CSET.
    CREF = 6,
    /// Sets output current of the current source. The difference
    /// between CSET and CREF divided by the resistor selected
    /// dictates the output current. This should never exceed 1.5 V.
    /// Must be within 1.5 V of CREF.
    CSET = 7,
}

impl AuxDACFn {

    /// Returns `true` if this AUX function occupies
    /// the lower 16 bits of the corresponding DAC
    /// instruction - See the protocol document for more
    pub(crate) fn is_lower(&self) -> bool {
        (*self as usize) % 2 != 0
    }
}



/// Output range for the DAC RNG instruction. There are only two options:
/// (a) [`OutputRange::STD`] standard ± 10 V range (at 300 μV precision)
/// or (b) [`OutputRange::EXT`] extended ± 20 V range (at 600 μV precision)
#[derive(Copy, Clone, Eq)]
pub struct OutputRange(bool);

impl OutputRange {
    /// Standard output range ± 10 V - 300 μV precision
    pub const STD: OutputRange = Self(false);
    /// Extended output range ± 20 V - 600 μV precision
    pub const EXT: OutputRange = Self(true);

}

impl PartialEq for OutputRange {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl PartialEq<bool> for OutputRange {
    fn eq(&self, other: &bool) -> bool {
        self.0 == *other
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
    /// assert_eq!(u32::from(&clusters), 0x00009001);
    /// clusters.set_channel(12);
    /// assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH12_15 |
    ///     DACMask::CH48_51 | DACMask::CH60_63);
    /// assert_eq!(u32::from(&clusters), 0x00009009);
    ///
    /// clusters.unset_channel(61);
    /// assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH12_15 |
    ///     DACMask::CH48_51);
    /// assert_eq!(u32::from(&clusters), 0x00001009);
    ///
    /// clusters.clear();
    /// assert_eq!(clusters, DACMask::NONE);
    /// assert_eq!(u32::from(&clusters), 0x0);
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

}

impl From<&DACMask> for u32 {
    fn from(clusters: &DACMask) -> u32 {
        clusters.bits() as u32
    }
}

impl ToU32s for DACMask {
    fn as_u32s(&self) -> Vec<u32> {
        [u32::from(self)].to_vec()
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
        assert_eq!(u32::from(&clusters), 0x00009001);

        clusters.set_channel(12);
        assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH12_15 |
            DACMask::CH48_51 | DACMask::CH60_63);
        assert_eq!(u32::from(&clusters), 0x00009009);

        clusters.unset_channel(61);
        assert_eq!(clusters, DACMask::CH00_03 | DACMask::CH12_15 |
            DACMask::CH48_51);
        assert_eq!(u32::from(&clusters), 0x00001009);

        clusters.clear();
        assert_eq!(clusters, DACMask::NONE);
        assert_eq!(u32::from(&clusters), 0x0);

    }
}


/// Channel configurations currently supported by ArC2.
/// Use these with [`ChannelConf`] to control
/// individual ArC2 channels.
#[derive(Clone, PartialEq, Copy, FromPrimitive, Debug)]
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

    /// Create a new register from raw words. This is only meant
    /// to be used when reconstucting ChannelConfs from existing
    /// instructions and not when creating a new register from
    /// scratch.
    #[doc(hidden)]
    pub(crate) fn from_raw_words(words: &[u32]) -> ChannelConf {
        let size = consts::NCHANS * consts::CHANCONFSIZE;
        let mut vec: BitVec<u32, Msb0> = BitVec::with_capacity(size);
        vec.extend_from_raw_slice(words);

        ChannelConf { bits: vec }
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

    /// Generate a [`mask`][`ChanMask`] that contains all channels
    /// at the specified channel state. This is typically done through
    /// the [`UpdateChannel`][`crate::instruction::UpdateChannel`]
    /// instruction.
    ///
    /// ```
    /// use libarc2::instructions::*;
    /// use libarc2::registers::*;
    ///
    /// let mut conf = ChannelConf::new();
    /// conf.set(3, ChannelState::VoltArb);
    /// conf.set(19, ChannelState::VoltArb);
    /// let upch = UpdateChannel::from_regs_default_source(&conf);
    ///
    /// let mask = upch.mask(ChannelState::VoltArb);
    ///
    /// assert_eq!(mask.as_slice(), &[0x00000000, 0x00080008]);
    /// ```
    pub(crate) fn mask(&self, val: ChannelState) -> ChanMask {
        let mut mask = ChanMask::none();

        for idx in 0..consts::NCHANS {
            if self.get(idx) == val {
                mask.set_enabled(idx, true);
            }
        }

        mask
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

bitflags! {
    /// Resistor selection for current source configuration
    ///
    /// Use this struct with [`CurrentSourceState::mos_with_resistors`] to
    /// create an appropriately configured register for sourcing/sinking
    /// current.
    pub struct CurrentSourceResistor: u8 {
        const R220K = 0b01000000;
        const R3_6M = 0b00100000;
        const R68M  = 0b00010000;
        const R51   = 0b00000100;
        const RDIGI = 0b00000010;
    }
}

lazy_static! {
    static ref FIXED_RESISTORS: HashMap<CurrentSourceResistor, f32> = {
        let mut map: HashMap<CurrentSourceResistor, f32> = HashMap::new();
        map.insert(CurrentSourceResistor::R220K, 220e3);
        map.insert(CurrentSourceResistor::R3_6M, 3.6e6);
        map.insert(CurrentSourceResistor::R68M, 68e6);
        map.insert(CurrentSourceResistor::R51, 51.0);
        map
    };
}

impl CurrentSourceResistor {
    fn value(self) -> Option<f32> {
        FIXED_RESISTORS.get(&self).copied()
    }
}


#[derive(Clone, Copy, FromPrimitive, ToPrimitive, PartialEq, Debug)]
#[repr(u8)]
pub enum CurrentSourceMOSFET {
    NMOS = 0b10000000,
    PMOS = 0b00001000
}

#[derive(Debug)]
#[repr(transparent)]
pub struct CurrentSourceState(u8);

impl CurrentSourceState {

    /// Generate a new current source configuration connected to the
    /// internal 2.5 V reference source
    pub fn vref() -> CurrentSourceState {
        // Assert bits 0 and 7 (2.5V and NMOS)
        CurrentSourceState(0b10000001)
    }

    /// Generate a current source configuration with either a PMOS or NMOS and at
    /// least one current source resistor. This can be any of the fixed series
    /// resistances or a digipot. Please not that the actual digipot value is not
    /// configured here but through the higher level instruction
    /// [`crate::instructions::UpdateChannel`].
    ///
    /// ```
    /// use libarc2::registers::{CurrentSourceState, CurrentSourceMOSFET};
    /// use libarc2::registers::CurrentSourceResistor;
    ///
    /// let state0 = CurrentSourceState::mos_with_resistors(
    ///     CurrentSourceMOSFET::PMOS, CurrentSourceResistor::R220K | CurrentSourceResistor::RDIGI);
    ///
    /// let rawval = state0.to_u8();
    /// assert_eq!(rawval, 0b01001010);
    /// assert!(rawval & CurrentSourceResistor::R220K.bits() > 0);
    /// assert!(rawval & CurrentSourceResistor::RDIGI.bits() > 0);
    /// assert!(rawval & CurrentSourceResistor::R51.bits() == 0);
    /// ```
    pub fn mos_with_resistors(mos: CurrentSourceMOSFET, res: CurrentSourceResistor)
        -> CurrentSourceState {

        // This is safe as we know it fits in a u8
        let mut val: u8 = mos.to_u8().unwrap();
        val |= res.bits();

        CurrentSourceState(val)
    }

    /// Get the internal representation of this register
    pub fn to_u8(self) -> u8 {
        self.0
    }

    /// Get selected MOSFET for this register
    pub fn get_mos(self) -> Option<CurrentSourceMOSFET> {
        if (self.0 & CurrentSourceMOSFET::NMOS.to_u8().unwrap()) > 0 {
            Some(CurrentSourceMOSFET::NMOS)
        } else if (self.0 & CurrentSourceMOSFET::PMOS.to_u8().unwrap()) > 0 {
            Some(CurrentSourceMOSFET::PMOS)
        } else {
            None
        }
    }

    /// Returns the bitflags for the selected current source resistor,
    /// or None if no resistors are selected
    pub fn get_selected_resistors(self) -> Option<CurrentSourceResistor> {
        let v: u8 = self.0 & 0b01110110;
        if v == 0 {
            None
        } else {
            Some(CurrentSourceResistor { bits: v })
        }
    }
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

    /// Set digipot raw value. This is clamped to 2^10-1
    pub fn set_digipot(&mut self, val: u16) {
        let actual_val;
        if val > 0x3ff {
            actual_val = 0x3ff;
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
        self.bits[24..32].store(val.to_u8());
    }

    /// Retrieves the current source state stores in this register.
    pub fn get_cursource_state(&self) -> CurrentSourceState {
        let val = self.bits[24..32].load::<u8>();
        CurrentSourceState(val)
    }

    fn _find_digipot_and_voltage(current: f32) -> Option<(f32, f32)> {
        let mut voltage: f32 = 1.0;
        let acurrent = current.abs();

        // We start from 1.0 V and work our way down in 50 mV steps to
        // find the first valid digipot resistance. If we drop below
        // then no voltage/resistance configuration exists.
        while voltage > 0.049 {
            let res = voltage / acurrent;
            // Resistance must be between 670 Ω and 19.5 kΩ
            // (safe digipot values)
            if res < 19500.0 && res > 670.0 {
                return Some((voltage, res));
            }
            voltage -= 0.05;
        }
        None
    }

    /// Construct a new source configuration for a specified current.
    /// This will also return the associated voltage drop required
    /// between the CREF and CSET references to satisfy the requirements.
    /// If no possible configuration exists for the selected current the
    /// function will return `None`.
    pub fn for_current(current: f32) -> Option<(SourceConf, f32)> {

        let mut sourceconf = SourceConf::new();

        let mosfet = if current > 0.0 {
            CurrentSourceMOSFET::NMOS
        } else {
            CurrentSourceMOSFET::PMOS
        };

        // Check if we can produce the necessary current with on of the values
        // in the fixed resistor bank
        for res in &[CurrentSourceResistor::R51, CurrentSourceResistor::R220K,
            CurrentSourceResistor::R3_6M, CurrentSourceResistor::R68M] {

            // unwrap here is safe as the fixed resistors have predefined
            // values in the FIXED_RESISTORS map
            let voltage = res.value().unwrap() * current.abs();
            if voltage > 0.05 && voltage <= 1.0 {
                // found voltage + resistor combination
                let state = CurrentSourceState::mos_with_resistors(mosfet, *res);
                sourceconf.set_cursource_state(state);
                return Some((sourceconf, voltage));
            }
        }

        // If we reach this point it means we need to use the digipot instead of
        // the fixed resistors
        match Self::_find_digipot_and_voltage(current) {
            Some ((resistance, voltage)) => {
                let code = (1024.0*(1.0 - resistance/20000.0)).floor() as u16;
                let state = CurrentSourceState::mos_with_resistors(mosfet,
                    CurrentSourceResistor::RDIGI);
                sourceconf.set_digipot(code);
                sourceconf.set_cursource_state(state);
                Some((sourceconf, voltage))
            }
            None => None
        }

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

    use super::{SourceConf, ToU32s};

    #[test]
    fn test_sourceconf() {
        let mut c = SourceConf::new();
        c.set_digipot(0x200);
        assert_eq!(c.get_digipot(), 0x200);

        let slice = c.as_u32s();
        assert_eq!(slice[0], 0x80000000);

        // Digipot must never exceed 0x3ff
        c.set_digipot(0x500);
        assert_eq!(c.get_digipot(), 0x3ff);

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
/// DACs have two outputs, upper and lower with upper > lower in
/// most circumstances. These are represented by a single word (u32) with
/// the upper 16 bits containing the upper voltage and the lower 16 bits
/// the lower voltage. In normal non fast pulsing scenarios both output
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
/// // Set the upper half voltage of the third channel
/// reg1.set_upper(2, 0x8534);
/// assert_eq!(reg1.get(2), (0x8000, 0x8534));
/// ```
pub struct DACVoltage {
    values: Vec<u32>
}

impl DACVoltage {

    const NUMVOLTAGES: usize = 4;

    /// Create a new register with four channels
    pub fn new() -> DACVoltage {
        Self::new_with_size_and_voltage(Self::NUMVOLTAGES, DACVZERO)
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

    // Similar to `from_raw_values` but Optional inputs, Somes will
    // be unwrapped, Nones will be replaced with zero
    pub(crate) fn from_raw_values_opt(values: &[Option<u32>]) -> DACVoltage {
        let mut raw_values = Vec::with_capacity(Self::NUMVOLTAGES);
        for value in values {
            let raw = match value {
                Some(v) => *v,
                None => DACVZERO
            };
            raw_values.push(raw);
        }

        DACVoltage { values: raw_values }
    }

    /// Set the upper value of a specified channel index
    pub fn set_upper(&mut self, idx: usize, voltage: u16) {
        self.values[idx] = (voltage as u32) << 16 |
            (self.values[idx] & 0xFFFF);
    }

    /// Get the upper value of a specified channel index
    pub fn get_upper(&self, idx: usize) -> u16 {
        ((self.values[idx] & 0xFFFF0000) >> 16) as u16
    }

    /// Set the lower value of a specified channel index
    pub fn set_lower(&mut self, idx: usize, voltage: u16) {
        self.values[idx] = (voltage as u32) <<  0 |
            (self.values[idx] & 0xFFFF0000);
    }

    /// Get the lower value of a specified channel index
    pub fn get_lower(&self, idx: usize) -> u16 {
        (self.values[idx] & 0xFFFF) as u16
    }

    /// Set both upper and lower of a specified channel index
    pub fn set(&mut self, idx: usize, voltage: u16) {
        self.set_lower(idx, voltage);
        self.set_upper(idx, voltage);
    }

    /// Get both upper and lower of a specified channel index.
    /// The first `u16` of the tuple is lower, the second upper.
    pub fn get(&self, idx: usize) -> (u16, u16) {
        (self.get_lower(idx), self.get_upper(idx))
    }

    /// Number of configured channels
    pub fn len(&self) -> usize {
        Self::NUMVOLTAGES
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
    fn dacvoltage_set_upper() {
        let mut v = DACVoltage::new();
        v.set_upper(3, 0xA0A0);

        assert_eq!(v.values[3], 0xA0A08000);
        assert_eq!(v.get_upper(3), 0xA0A0);
        assert_eq!(v.get_lower(3), 0x8000);
    }

    #[test]
    fn dacvoltage_set_lower() {
        let mut v = DACVoltage::new();
        v.set_lower(2, 0x90F3);

        assert_eq!(v.values[2], 0x800090F3);
        assert_eq!(v.get_upper(2), 0x8000);
        assert_eq!(v.get_lower(2), 0x90F3);
    }

    #[test]
    fn dacvoltage_set_both() {
        let mut v = DACVoltage::new();

        v.set(1, 0x8534);
        assert_eq!(v.values[1], 0x85348534);
        assert_eq!(v.get_lower(1), 0x8534);
        assert_eq!(v.get_upper(1), 0x8534);
        assert_eq!(v.get(1), (0x8534, 0x8534));
    }

}


bitflags! {
    /// Channel selector for DAC half-clusters
    ///
    /// [`DACVoltageMask`] is used to create a bitmask that limits
    /// the application of a [`SetDAC`][`crate::instructions::SetDAC`]
    /// instruction to one or more channels. Typically ArC2 will apply
    /// the configured voltages to all four channels of a DAC half-cluster
    /// but this can be limited to specific channels if a suitable
    /// [`DACVoltageMask`] is supplied.
    pub struct DACVoltageMask: u32 {
        /// Select no channels
        const NONE = 0b00000000000000000000000000000000;
        /// Channel 0
        const CH0  = 0b00000000000000000000000000001000;
        /// Channel 1
        const CH1  = 0b00000000000000000000000000000100;
        /// Channel 2
        const CH2  = 0b00000000000000000000000000000010;
        /// Channel 3
        const CH3  = 0b00000000000000000000000000000001;
        /// Select all channels
        const ALL  = Self::CH0.bits | Self::CH1.bits |
                     Self::CH2.bits | Self::CH3.bits;
    }
}

impl DACVoltageMask {

    const ALL_MASKS: [Self; 4] = [Self::CH0, Self::CH1, Self::CH2, Self::CH3];

    /// Create a new DACVoltageMask from individual indices within a
    /// DAC half-cluster
    pub(crate) fn from_indices(indices: &[usize]) -> Self {
        let mut mask = Self::NONE;
        for idx in indices {

            if *idx >= 4 {
                break;
            }

            mask |= Self::ALL_MASKS[*idx]
        }

        mask
    }

    /// Create a new DACVoltageMask from individual voltages within
    /// a DAC half-cluster. This must be 4-items long, None values will
    /// deselect the associated index
    pub(crate) fn from_indices_opt<T>(voltages: &[Option<T>]) -> Self {
        let mut mask = Self::NONE;
        for (idx, smth) in voltages.iter().enumerate() {
            if idx >=4 {
                break;
            }

            if smth.is_some() {
                mask |= Self::ALL_MASKS[idx];
            }
        }

        mask
    }
}

impl From<&DACVoltageMask> for u32 {
    fn from(mask: &DACVoltageMask) -> u32 {
        mask.bits() as u32
    }
}

impl ToU32s for DACVoltageMask {
    fn as_u32s(&self) -> Vec<u32> {
        [u32::from(self)].to_vec()
    }
}


/// A generic bitmask of the specified word size
#[derive(Clone, Debug, PartialEq)]
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

// Used to create the `from_vals` and `from_indices` function of U32Mask<T>
macro_rules! make_from_values_impl {
    ($ws:ty, $wx:expr) => {

        /// Create a new [`U32Mask`] from an existing bitvec
        #[allow(dead_code)]
        pub(crate) fn from_bitvec(vec: BitVec<u32, Msb0>) -> U32Mask<$ws> {
            U32Mask { _words: $wx, bits: vec }
        }

        /// Create a new [`U32Mask`] from a series of `u32s`.
        pub fn from_vals(vals: &[u32]) -> U32Mask<$ws> {
            let mut vec: BitVec<u32, Msb0> = BitVec::repeat(false, <$ws>::WORDS*32);
            for idx in 0..vals.len() {
                vec[idx*32..(idx+1)*32].store::<u32>(vals[idx]);
            }

            U32Mask { _words: $wx, bits: vec }
        }

        /// Create a new [`U32Mask`] from a series of selected indices
        pub fn from_indices(chans: &[usize]) -> U32Mask<$ws> {
            let mut mask = Self::new();

            for c in chans {
                if *c >= <$ws>::WORDS*32 { continue; }
                mask.set_enabled(*c, true);
            }

            mask
        }

        /// Create a new bitmask with no channels selected
        pub fn none() -> Self {
            Self::new()
        }

        /// Create a new bitmask with all channels selected
        pub fn all() -> Self {
            let mut mask = Self::new();
            mask.set_enabled_all(true);

            mask
        }

        /// Create a new bitmask with all channels in `chans` selected
        pub fn from_channels(chans: &[usize]) -> Self {
            let mut mask = Self::new();
            for c in chans {
                mask.set_enabled(*c, true);
            }

            mask
        }

        /// Set all the channels from `chans` to the specified state. This
        /// will add to the existing channels
        pub fn set_channels_enabled(&mut self, chans: &[usize], enabled: bool) {
            for c in chans {
                self.set_enabled(*c, enabled);
            }
        }

        /// Get a list of the enabled channels for this mask
        pub fn channels(&self) -> Vec<usize> {

            let mut res: Vec<usize> = Vec::with_capacity(consts::NCHANS);

            for ch in 0..consts::NCHANS {
                if self.get_enabled(ch) {
                    res.push(ch);
                }
            }

            res

        }

        /// Check if no channels are selected in this bitmask
        pub fn is_empty(&self) -> bool {
            *self == Self::none()
        }
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

    make_from_values_impl!(wordreg::Wx1, wordreg::Wx1{});
}

impl U32Mask<wordreg::Wx2> {
    pub fn new() -> U32Mask<wordreg::Wx2> {
        let vec: BitVec<u32, Msb0> = BitVec::repeat(false, wordreg::Wx2::WORDS*32);
        U32Mask { _words: wordreg::Wx2{}, bits: vec }
    }

    make_from_values_impl!(wordreg::Wx2, wordreg::Wx2{});
}

impl U32Mask<wordreg::Wx3> {
    pub fn new() -> U32Mask<wordreg::Wx3> {
        let vec: BitVec<u32, Msb0> = BitVec::repeat(false, wordreg::Wx3::WORDS*32);
        U32Mask { _words: wordreg::Wx3{}, bits: vec }
    }

    make_from_values_impl!(wordreg::Wx3, wordreg::Wx3{});
}

impl U32Mask<wordreg::Wx4> {
    pub fn new() -> U32Mask<wordreg::Wx4> {
        let vec: BitVec<u32, Msb0> = BitVec::repeat(false, wordreg::Wx4::WORDS*32);
        U32Mask { _words: wordreg::Wx4{}, bits: vec }
    }

    make_from_values_impl!(wordreg::Wx4, wordreg::Wx4{});
}


/// Channel configuration bitmask.
///
/// A `ChanMask` is used to select one or more channels. Essentially
/// it defines which channels the Read Current, Read Voltage, Amp Prep
/// or Modify Channel operations are applied to.
///
/// See [`U32Mask`][`crate::registers::U32Mask`] for details and
/// methods.
///
/// ## Example
///
/// ### Toggling individual channels
/// ```
/// use libarc2::registers::{ChanMask, ToU32s};
///
/// // new chanmask with everything set to 0
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
/// ### Creating a mask from a set of indices
/// ```
/// use libarc2::registers::{ChanMask, ToU32s};
///
/// let channels: Vec<usize> = vec![0, 7, 45];
/// let chan = ChanMask::from_indices(&channels);
/// assert_eq!(chan.get_enabled(0), true);
/// assert_eq!(chan.get_enabled(7), true);
/// assert_eq!(chan.get_enabled(45), true)
///
/// ```
///
///
pub type ChanMask = U32Mask<wordreg::Wx2>;

impl BitAnd for &ChanMask {

    type Output = ChanMask;

    fn bitand(self, other: Self) -> Self::Output {
        let mut output = ChanMask::new();

        let slice = self.bits.as_bitslice();
        let other = other.bits.as_bitslice();

        for (i, (t, o)) in zip(slice, other).enumerate() {
            output.set_enabled(consts::NCHANS-1-i, *t & *o);
        }

        output

    }

}

impl BitXor for &ChanMask {

    type Output = ChanMask;

    fn bitxor(self, other: Self) -> Self::Output {
        let mut output = ChanMask::new();

        let slice = self.bits.as_bitslice();
        let other = other.bits.as_bitslice();

        for (i, (t, o)) in zip(slice, other).enumerate() {
            output.set_enabled(consts::NCHANS-1-i, *t ^ *o);
        }
        output
    }

}

/// Register used to construct an appropriate range selection
/// bitmask for all available DACs on ArC TWO. For differential
/// biasing both DAC halves of each individual channel can be
/// ranged independently.
///
/// ## Examples
/// ```
/// use libarc2::registers::{RangeMask, OutputRange, AuxDACFn, ToU32s};
///
/// // Create a new mask, all channels at standard range
/// let mut mask = RangeMask::new();
///
/// // Set the 8th channel to extended range
/// mask.set_lower(7usize, OutputRange::EXT);
///
/// // Set the SELH Aux function to extended range
/// mask.set_aux(AuxDACFn::SELH, OutputRange::EXT);
///
/// // Set the logic level range to extended
/// mask.set_logic(OutputRange::EXT);
///
/// assert_eq!(mask.as_u32s(), [0x00000801, 0x00000000, 0x00000000,
///     0x00000000, 0x00004000])
/// ```
pub struct RangeMask {
    bits: BitVec<u32, Msb0>,
}

impl RangeMask {

    // The virtual channel that corresponds to
    // the LGC range bit in the AUX1 DAC
    const LGC_VCHANNEL: usize = 70;

    pub fn new() -> RangeMask {

        // due to the issue LS bits and MS bytes we ensure that the
        // underlying bytes are full 5 u32s long. The upper 16 bits
        // will always be empty
        let size: usize = consts::NCHANS*consts::RANGCONFSIZE + consts::AUXNCHANS;
        let nbits = std::mem::size_of::<u32>()*8;
        let words = if size % nbits == 0 {
            size / nbits
        } else {
            size / nbits + 1
        };
        let vec: BitVec<u32, Msb0> = BitVec::repeat(false, words*nbits);

        RangeMask { bits: vec }
    }

    /// Set the range for both upper and lower half of the specified channel
    pub fn set(&mut self, idx: usize, range: OutputRange) {
        self.set_upper(idx, range);
        self.set_lower(idx, range);
    }

    pub fn set_aux(&mut self, func: AuxDACFn, range: OutputRange) {
        // 0, 1 → 64 | 2, 3 → 65 | 4, 5 → 66 | 6, 7 → 67
        // L, U      | L, U      | L, U      | L, U

        // this finds the "virtual" channel idx of the aux fn
        let aux_chan: usize = consts::NCHANS + (func as usize)/2usize;

        if func.is_lower() {
            self.set_lower(aux_chan, range);
        } else {
            self.set_upper(aux_chan, range);
        }
    }

    pub fn set_logic(&mut self, range: OutputRange) {
        self.set_upper(Self::LGC_VCHANNEL, range);
    }

    /// Set the range for the upper half of the specified channel
    pub fn set_upper(&mut self, idx: usize, range: OutputRange) {

        let actual_idx: usize = self.bits.len() - 1 - idx*consts::RANGCONFSIZE;
        self.bits.set(actual_idx+1, range.0);
    }

    /// Set the range for the lower half of the specified channel
    pub fn set_lower(&mut self, idx: usize, range: OutputRange) {

        let actual_idx: usize = self.bits.len() - 1 - idx*consts::RANGCONFSIZE;
        self.bits.set(actual_idx, range.0);
    }

}

impl ToU32s for RangeMask {
    fn as_u32s(&self) -> Vec<u32> {
        let bits = self.bits.as_raw_slice();
        bits.to_vec()
    }
}


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
    fn from_vals() {
        let mut v = IOMask::from_vals(&[0x80000001]);

        assert_eq!(&v.as_u32s(), &[0x80000001]);
        assert_eq!(v.get_enabled(31), true);
        assert_eq!(v.get_enabled(0), true);
        v.set_enabled(1, true);
        assert_eq!(&v.as_u32s(), &[0x80000003]);

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
        io.set_en(false);

        io
    }

    /// Create a new `IOEnable` with all IOs enabled and set to output
    pub fn all_output() -> IOEnable {
        let vec: BitVec<u32, Lsb0> = BitVec::repeat(false, Self::LEN);
        let mut io = IOEnable { bits: vec };
        io.set_en(true);
        io.set_all_outputs(true);

        io
    }

    /// Create a new `IOEnable` with all IOs enabled and set to input
    pub fn all_input() -> IOEnable {
        let vec: BitVec<u32, Lsb0> = BitVec::repeat(false, Self::LEN);
        let mut io = IOEnable { bits: vec };
        io.set_en(true);
        io.set_all_outputs(false);

        io
    }

    /// Toggle the enable bit
    pub fn set_en(&mut self, status: bool) {
        // IO Enable is active low
        self.set_output(Self::LEN-1, !status);
    }

    pub fn get_en(&self) -> bool {
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
        io.set_en(true);
        io.set_output(0, true);
        io.set_output(1, true);
        io.set_output(2, false);
        io.set_output(3, true);

        assert_eq!(io.as_u32s(), &[0xb]);

        io.set_output(2, true);
        assert_eq!(io.as_u32s(), &[0xf]);
    }
}
