//! Commands processable by ArC2

use std::convert::TryInto;
use std::collections::BTreeMap;
use crate::registers;
use crate::registers::OutputRange;
use crate::registers::ToU32s;
use crate::registers::Terminate;
use crate::registers::consts::{DACHCLUSTERMAP, SELECTORMAP};
use crate::registers::{OpCode, Empty, DACMask, ArbMask, DACVoltage, DACVoltageMask};
use crate::registers::{ChannelConf, SourceConf, ChannelState};
use crate::registers::{IOEnable, IOMask, IODir, ChanMask, Averaging};
use crate::registers::{Duration50, Address, HSDelay, DACCluster};
use crate::registers::{ClusterMask, PulseAttrs, AuxDACFn, SelectorMask};
use num_traits::FromPrimitive;
use thiserror::Error;

macro_rules! make_vec_instr_impl {
    ($t:ident, $f:ident, $n:expr) => {

        #[doc(hidden)]
        type S = Self;

        fn create() -> $t {
            $t { $f: Vec::with_capacity(Self::LENGTH) }
        }

        fn len(&self) -> usize {
            self.$f.len()
        }

        fn push_u32s(&mut self, reg: &[u32]) {
            self.$f.extend_from_slice(&reg);
        }

        fn view(&self) -> &[u32] {
            &self.$f
        }

        fn name(&self) -> &'static str {
            $n
        }
    }
}

macro_rules! dacvoltage {
    ($low: expr, $high: expr) => {
        (($high as u32) << 16) | (($low as u32) & 0xFFFF)
    }
}

// Check if a pair of DAC Voltages is valid. The DAC-
// portion must be AT MOST 1u16 greater than the DAC+
// portion of the DAC Voltage
macro_rules! dacvoltages_ok {
    ($low: expr, $high: expr) => {

        match $low {
            low if low == u16::MAX => $high >= u16::MAX - 1,
            low => match $high {
                high if high == u16::MAX => true,
                high => low <= high + 1
            }
        }

    }
}

#[derive(Error, Debug)]
pub enum InstructionError {
    #[error("Invalid DAC composition at DAC-: {0} DAC+: {1}")]
    InvalidDACComposition(u16, u16),
    #[error("CREF and CSET must be both set and within 1.5 V of each other")]
    InvalidCREFCSETCombination,
    #[error("Selected channel is outside the maximum range of {0}")]
    ChannelRangeError(usize)
}


/// Common behaviour of all available instructions.
///
/// The Instruction trait is implemented by all instructions. Typically
/// instructions must implement the `create`, `push_u32s`, `len` and `view`
/// functions. Everything else is derived from these.
///
/// Instructions can be "compiled" into a `&[u32]` buffer and can be written
/// directly with [`process`][`crate::Instrument::process()`].
pub trait Instruction {

    #[doc(hidden)]
    const LENGTH: usize = 9;
    #[doc(hidden)]
    const U32SIZE: usize = std::mem::size_of::<u32>();

    /// Type of the instruction. For most cases this should be `Self`.
    type S: Instruction;

    /// Create a new empty instruction
    fn create() -> Self::S;

    /// Add an array of u32s to the instruction
    fn push_u32s(&mut self, reg: &[u32]);

    /// Number of allocated registers
    fn len(&self) -> usize;

    /// Get a u32 view of the current instruction
    fn view(&self) -> &[u32];

    /// Name of the instruction
    fn name(&self) -> &'static str;

    /// Pad and terminate instruction for consumption. This will be
    /// typically communicated to ArC2 using the
    /// [`process`][`crate::Instrument::process()`] and
    /// [`compile_process`][`crate::Instrument::compile_process`] functions.
    ///
    /// ```no_run
    /// use libarc2::{Instrument};
    /// use libarc2::instructions::{ResetDAC, Instruction};
    ///
    /// let mut arc2 = Instrument::open_with_fw(0, "fw.bin", true, true).unwrap();
    ///
    /// let mut reset = ResetDAC::new();
    /// arc2.process(reset.compile());
    /// ```
    fn compile(&mut self) -> &mut Self {
        if self.len() == Self::LENGTH {
            // this instruction is already compiled
            return self;
        }
        for _ in 0..(Self::LENGTH-self.len()-1) {
            self.push_register(&Empty::new());
        }

        for _ in 0..(Self::LENGTH-self.len()) {
            self.push_register(&Terminate::new());
        }

        self
    }

    #[doc(hidden)]
    /// Create a new instruction from a bunch of registers
    fn from_registers(registers: &[&dyn ToU32s]) -> Self::S {

        let mut instr = Self::create();

        for reg in registers {
            instr.push_u32s(&reg.as_u32s());
        }

        instr
    }

    /// Add a new register to this instruction
    fn push_register<T: ToU32s>(&mut self, register: &T) {
        self.push_u32s(&register.as_u32s());
    }

    /// Convert this instruction into a byte array.
    ///
    /// Get the BE bytes for this instruction.
    fn to_bytevec(&self) -> Vec<u8> {
        let cap = Self::LENGTH * Self::U32SIZE;
        let mut vec: Vec<u8> = Vec::with_capacity(cap);
        for num in self.view() {
            vec.extend_from_slice(&num.to_le_bytes());
        }
        vec
    }

}


/// Reset DACs to 0.0 V
///
/// This instruction is used to reset all output DACs back to zero
/// voltage. For this instruction to actually have any effec it must
/// be followed by a [`UpdateDAC`] one. It's a special case of the
/// [`SetDAC`] instruction.
///
/// ## Instruction layout
///
/// ```text
///        +--------+--------------+-------+---------------------+------------------+
///        | OpCode | DACMask::ALL | Empty | DACVoltageMask::ALL | DACVoltage::ZERO |
///        +--------+--------------+-------+---------------------+------------------+
/// Words:     1           1           1              1                   4
/// ```
pub struct ResetDAC {
    instrs: Vec<u32>
}

impl ResetDAC {

    /// Create a new instruction
    pub fn new() -> Self {
        Self::from_registers(&[&OpCode::SetDAC,
                               &DACMask::ALL,
                               &Empty::new(),
                               &DACVoltageMask::ALL,
                               &DACVoltage::new()])
    }
}

impl Instruction for ResetDAC { make_vec_instr_impl!(ResetDAC, instrs, "RESET"); }


/// Update DAC configuration previously set with [`ResetDAC`] or [`SetDAC`]
///
/// This instruction typically follows a [`ResetDAC`] of [`SetDAC`]
/// instruction and it must be sent to the instrument for any of them to
/// have any effect.
///
/// ## Instruction layout
///
/// ```text
///        +--------+
///        | OpCode |
///        +--------+
/// Words:     1
/// ```
pub struct UpdateDAC {
    instrs: Vec<u32>
}

impl UpdateDAC {

    /// Create a new instruction
    pub fn new() -> Self {
        Self::from_registers(&[&OpCode::UpdateDAC])
    }
}

impl Instruction for UpdateDAC { make_vec_instr_impl!(UpdateDAC, instrs, "UP DAC"); }


/// Set a DAC configuration
///
/// ## Instruction layout
///
/// ```text
///        +--------+----------+--------+----------------+------------+
///        | OpCode |  DACMask |  Empty | DACVoltageMask | DACVoltage |
///        +--------+----------+--------+----------------+------------+
/// Words:     1         1         1            1              4
/// ```
///
/// ## Example
/// ```
/// use libarc2::registers::{DACMask, DACVoltageMask, DACVoltage, ToU32s};
/// use libarc2::instructions::{SetDAC, Instruction};
///
/// // Enable first two half channels (8 channels)
/// let mut mask = DACMask::NONE;
/// mask.set_channels(&[0, 1, 2, 3, 4, 5, 6, 7]);
///
/// // Set the voltages to
/// // CH0 0x0000_8000 channel 0 (high) and 1 (low)
/// // CH1 0x8000_8000 channel 2 (high) and 3 (low)
/// // CH2 0x8000_9000 channel 4 (high) and 5 (low)
/// // CH3 0x9000_FFFF channel 6 (high) and 7 (low)
/// let mut voltages = DACVoltage::new();
/// voltages.set_upper(0, 0x8000);
/// voltages.set_lower(0, 0x0000);
/// voltages.set_upper(1, 0x8000);
/// voltages.set_lower(1, 0x8000);
/// voltages.set_upper(2, 0x9000);
/// voltages.set_lower(2, 0x8000);
/// voltages.set_upper(3, 0xFFFF);
/// voltages.set_lower(3, 0x9000);
///
/// let mut instr = SetDAC::with_regs(&mask, &voltages, &DACVoltageMask::ALL).unwrap();
///
/// assert_eq!(instr.compile().view(),
///     &[0x1, 0x3, 0x0, 0x0000000F, 0x80000000, 0x80008000,
///       0x90008000, 0xFFFF9000, 0x80008000]);
/// ```
pub struct SetDAC {
    instrs: Vec<u32>
}


impl SetDAC {

    /// Create a new unconfigured instruction
    pub fn new() -> Self {
        Self::from_registers(&[&OpCode::SetDAC,
                               &Empty::new(), &Empty::new(),
                               &Empty::new(), &Empty::new(),
                               &Empty::new(), &Empty::new(),
                               &Empty::new()])
    }

    /// Create a new instruction that grounds all channels
    pub fn new_all_ground() -> Self {
        let mut instr = SetDAC::create();
        instr.push_register(&OpCode::SetDAC);
        instr.push_register(&DACMask::ALL);
        instr.push_register(&Empty::new());
        instr.push_register(&DACVoltageMask::ALL);
        instr.push_register(&DACVoltage::new());

        instr
    }

    /// Create a new instruction with all channels set at
    /// the specified levels. Will throw an error if DAC- > DAC+.
    pub fn new_all_at_bias(low: u16, high: u16) -> Result<Self, InstructionError> {

        if !dacvoltages_ok!(low, high) {
            return Err(InstructionError::InvalidDACComposition(low, high));
        }

        let mut instr = SetDAC::create();
        instr.push_register(&OpCode::SetDAC);
        instr.push_register(&DACMask::ALL);
        instr.push_register(&Empty::new());
        instr.push_register(&DACVoltageMask::ALL);
        instr.push_register(&DACVoltage::new_at_levels(low, high));

        Ok(instr)
    }

    pub(crate) fn new_logic(level: f32) -> (DACRange, Self) {

        // 3.81 * 2.62 = 9.98 V - need to switch to higher
        // range for higher voltage levels
        let range = if level > 3.81 {
            OutputRange::EXT
        } else {
            OutputRange::STD
        };

        let mask = ArbMask::from_aux_channels(&[AuxDACFn::LGC]);
        let dacrange = DACRange::aux_chans_at_range(&mask, &range);
        let setdac = if range == OutputRange::STD {
            SetDAC::for_logic(level*2.62)
        } else {
            SetDAC::for_logic(level*1.31)
        };
        (dacrange, setdac)
    }

    /// Create a new instruction with specified registers
    pub fn with_regs(chanmask: &DACMask, voltages: &DACVoltage, voltmask: &DACVoltageMask) ->
        Result<Self, InstructionError> {

        for idx in 0..voltages.len() {
            let l = voltages.get_lower(idx);
            let h = voltages.get_upper(idx);

            if !dacvoltages_ok!(l, h) {
                return Err(InstructionError::InvalidDACComposition(l, h));
            }

        }

        Ok(SetDAC::with_regs_unchecked(chanmask, voltages, voltmask))
    }

    /// Create a new instruction with specific registers - no DAC+/DAC- check
    /// This is only used internally - never, ever call this function unless
    /// you know what you're doing
    fn with_regs_unchecked(chanmask: &DACMask, voltages: &DACVoltage, voltmask: &DACVoltageMask) -> Self {
        let mut instr = SetDAC::create();
        instr.push_register(&OpCode::SetDAC);
        instr.push_register(chanmask);
        instr.push_register(&Empty::new());
        instr.push_register(voltmask);
        instr.push_register(voltages);

        instr
    }

    /// Assemble a minimum set of instructions that correspond to the supplied
    /// voltages and channels. The first argument of this function is a slice of
    /// tuples that conform to this specification
    ///
    /// ```text
    /// (channel idx, DAC- voltage, DAC+ voltage)
    /// ```
    ///
    /// Unselected channels will be set to whatever the `base` argument indicates and
    /// set to a state specified by `unselected_state`. The format for `base` is
    /// is the same eg. (DAC- voltage, DAC+ voltage). Argument `selected_state`,
    /// will be the channel configuration for the selected channels. This will
    /// typically be either [`VoltArb`][`crate::registers::ChannelState::VoltArb`], for
    /// arbitrary voltage operations, or
    /// [`HiSpeed`][`crate::registers::ChannelState::HiSpeed`], for high voltage pulsing.
    /// State of the unselected channels can be provided by `unselected_state`. This
    /// will typically be [`VoltArb`][`crate::registers::ChannelState::VoltArb`] or
    /// [`HiSpeed`][`crate::registers::ChannelState::HiSpeed`] if channels should be
    /// biased or [`Maintain`][`crate::registers::ChannelState::Maintain`] if the
    /// previous configuration should be kept instead. Although you can set the
    /// unselected channels to [`Open`][`crate::registers::ChannelState::Open`]
    /// this does not guarantee that the channels will in fact be floated as
    /// they might be tied to a hard ground by a previous command.
    ///
    /// The function will return two values, the first is an [`UpdateChannel`]
    /// instruction that should be used to properly configure the channels. The
    /// second is the actual vector of [`SetDAC`] instructions that set the
    /// voltages of all channels at appropriate levels. Both of them should be
    /// processed by ArC TWO (unless you know what you're doing).
    ///
    /// ## Example
    /// ```
    /// use libarc2::instructions::{SetDAC, Instruction};
    /// use libarc2::registers::ChannelState;
    ///
    /// let input: Vec<(u16, u16, u16)> = vec![
    ///    ( 0, 0x8ccc, 0x8ccc), ( 1, 0x8ccc, 0x8ccc), ( 2, 0x6000, 0x7000),
    ///    ( 4, 0x8ccc, 0x8ccc), ( 5, 0x8ccc, 0x8ccc), ( 6, 0x6000, 0x7000),
    ///    (12, 0x8ccc, 0x8ccc), (13, 0x8ccc, 0x8ccc), (14, 0x6000, 0x7000),
    ///    (22, 0x7000, 0x7000), (17, 0x6000, 0x7000), (63, 0x7000, 0x9000)
    /// ];
    ///
    /// let (_, mut instructions) = SetDAC::from_channels(&input, Some((0x8000, 0x8000)),
    ///     &ChannelState::VoltArb, &ChannelState::VoltArb).unwrap();
    ///
    /// assert_eq!(instructions.len(), 5);
    /// // This sets all values at base voltage
    /// assert_eq!(instructions[0].compile().view(),
    ///     &[0x00000001, 0x0000ffff, 0x00000000, 0x0000000f, 0x80008000,
    ///       0x80008000, 0x80008000, 0x80008000, 0x80008000]);
    /// // The next four instructions are calculated so the channels in `input`
    /// // are set to their specified voltages
    /// assert_eq!(instructions[1].compile().view(),
    ///     &[0x00000001, 0x00008000, 0x00000000, 0x00000001, 0x80008000,
    ///       0x80008000, 0x80008000, 0x90007000, 0x80008000]);
    /// assert_eq!(instructions[2].compile().view(),
    ///     &[0x00000001, 0x00000020, 0x00000000, 0x00000002, 0x80008000,
    ///       0x80008000, 0x70007000, 0x80008000, 0x80008000]);
    /// assert_eq!(instructions[3].compile().view(),
    ///     &[0x00000001, 0x00000010, 0x00000000, 0x00000004, 0x80008000,
    ///       0x70006000, 0x80008000, 0x80008000, 0x80008000]);
    /// assert_eq!(instructions[4].compile().view(),
    ///     &[0x00000001, 0x0000000b, 0x00000000, 0x0000000e, 0x8ccc8ccc,
    ///       0x8ccc8ccc, 0x70006000, 0x80008000, 0x80008000]);
    /// ```
    pub fn from_channels(input: &[(u16, u16, u16)], base: Option<(u16, u16)>,
        selected_state: &ChannelState, unselected_state: &ChannelState) ->
        Result<(UpdateChannel, Vec<Self>), InstructionError> {

        let mut result: Vec<Self> = Vec::new();
        let mut chanconf = ChannelConf::new_with_state(*unselected_state);
        let mut bucket: BTreeMap<[Option<u32>; 4], DACMask> = BTreeMap::new();
        let mut dacs: [Option<u32>; 64] = [None; 64];

        // Preinitialise all channels at the specified voltage
        match base {
            Some((vlow, vhigh)) => {
                result.push(SetDAC::new_all_at_bias(vlow, vhigh)?);
            },
            _ => {}
        }

        // Assign active channel states and voltages
        for (ch, clow, chigh) in input {
            let idx = *ch as usize;
            chanconf.set(idx, *selected_state);
            dacs[idx] = Some(dacvoltage!(*clow, *chigh));
        }

        // Split them into groups of 4
        for (cidx, chunk) in dacs.chunks(4).enumerate() {
            // This shouldn't happen but skip half-clusters
            // > 15
            if cidx >= 16usize {
                break;
            }

            // Ignore clusters that are completely empty
            if chunk.iter().all(|item| item.is_none()) {
                continue;
            }

            // We know this is the correct length because we
            // are moving in steps of 4 elements at a time
            let key: [Option<u32>; 4] = chunk.try_into().unwrap();

            if bucket.contains_key(&key) {
                let entry = bucket.get_mut(&key).unwrap();
                *entry |= DACHCLUSTERMAP[cidx];
            } else {
                bucket.insert(key, DACHCLUSTERMAP[cidx]);
            }

        }

        // Construct the SetDACs
        for (voltage, dacmask) in &bucket {
            let voltages = DACVoltage::from_raw_values_opt(voltage);

            // Find the non-none indices
            let dacvoltagemask = DACVoltageMask::from_indices_opt(voltage);
            result.push(SetDAC::with_regs(&dacmask, &voltages, &dacvoltagemask)?);

        }

        Ok((UpdateChannel::from_regs_default_source(&chanconf), result))
    }

    /// Convenience function to generate an LD VOLT instruction for
    /// the LGC part of the auxiliary DACs. LGC is in a non-continuous
    /// index with respect to the rest of auxiliary DACs and is also
    /// on AUX1 instead of AUX0 so it needs special handling.
    fn for_logic(voltage: f32) -> Self {
        let mut voltages = DACVoltage::new();
        let idx = AuxDACFn::LGC;
        let dac_idx = (idx as usize - 8usize) / 2usize;

        if idx.is_lower() {
            voltages.set_lower(dac_idx, vidx!(voltage));
        } else {
            voltages.set_upper(dac_idx, vidx!(voltage));
        }

        SetDAC::with_regs_unchecked(&DACMask::AUX1, &voltages, &DACVoltageMask::ALL)
    }

    /// Set values for the auxiliary DACs
    pub(crate) fn from_channels_aux(input: &[(AuxDACFn, f32)])
        -> Result<Vec<Self>, InstructionError> {

        // This will hold at most 4 values, a SetDAC to clear, a DACRange
        // and up to two actual SetDACs
        let mut result: Vec<SetDAC> = Vec::with_capacity(4usize);

        // all AUX channels (except for LGC) live on AUX0
        let mask = DACMask::AUX0;

        let mut voltages = DACVoltage::new();

        // we need to ensure that CSET and CREF are within 1V of
        // each other and also that if one is set the other one
        // is too
        let mut cset: Option<f32> = None;
        let mut cref: Option<f32> = None;

        for (idx, voltage) in input {

            match idx {
                AuxDACFn::CSET => {
                    cset = Some(*voltage)
                },
                AuxDACFn::CREF => {
                    cref = Some(*voltage)
                },
                AuxDACFn::LGC => {
                    // If AuxDACFn::LGC is present make a new instruction
                    // and add it; no need for further processing
                    result.push(SetDAC::for_logic(*voltage));
                    continue;
                }
                _ => {}
            }

            // get one of the four DACs that corresponds to the
            // selected idx - remember the DAC voltages are
            // addressed as half words, so we need to find the
            // DAC which corresponds to half-word #idx
            let dac_idx = (*idx as usize) / 2usize;

            if idx.is_lower() { // lower 16 bits
                voltages.set_lower(dac_idx, vidx!(*voltage));
            } else { // upper 16 bits
                voltages.set_upper(dac_idx, vidx!(*voltage));
            }
        }


        // Don't bother to check CSET and CREF if both are not selected
        if !(cset.is_none() && cref.is_none()) {
            // If CSET is set but not CREF (or the other way around) throw
            // an error - both or none must be set
            let csetv = cset.ok_or(InstructionError::InvalidCREFCSETCombination)?;
            let crefv = cref.ok_or(InstructionError::InvalidCREFCSETCombination)?;

            // Throw an error if CSET and CREF are more than 1 V apart
            if (csetv - crefv).abs() > 1.5 {
                return Err(InstructionError::InvalidCREFCSETCombination);
            }

            // If we reached this point without InstructionError we're good!

        }

        result.push(SetDAC::with_regs_unchecked(&mask, &voltages, &DACVoltageMask::ALL));
        Ok(result)
    }

    /// Set the channel mask of this instruction
    pub fn set_mask(&mut self, chanmask: &DACMask) {
        self.instrs[1] = chanmask.as_u32s()[0];
    }

    /// Set the voltage registers of this instruction
    pub fn set_voltages(&mut self, voltages: &DACVoltage) {
        let voltages_u32 = voltages.as_u32s();
        for (idx, num) in voltages_u32.iter().enumerate() {
            self.instrs[idx+4] = *num;
        }
    }
}

impl Instruction for SetDAC { make_vec_instr_impl!(SetDAC, instrs, "LD VOLT"); }


/// Set channel configuration
///
/// The `UpdateChannel` instruction is used to configure all the channels
/// of ArC2 based on their functionality; see [`ChannelState`] for more details.
///
/// ## Instruction layout
///
/// ```text
///        +--------+-------------+-------+-------+---------------+
///        | OpCode |  SourceConf | Empty | Empty |  ChannelConf  |
///        +--------+-------------+-------+-------+---------------+
/// Words:     1            1         1       1           4
/// ```
///
/// ## Example
/// ```
/// use libarc2::registers::{ChannelConf, SourceConf, ChannelState};
/// use libarc2::instructions::{UpdateChannel, Instruction};
///
/// // new channel configuration
/// let mut chanconf = ChannelConf::new();
/// // set all output channels to arbitrary voltage
/// chanconf.set_all(ChannelState::VoltArb);
///
/// // source configuration; use default, 11 kΩ digipot
/// let sourceconf = SourceConf::new();
///
/// // make the instruction
/// let mut instr = UpdateChannel::from_regs(&sourceconf, &chanconf);
///
/// assert_eq!(instr.compile().view(), &[0x40, 0x73400000,
///     0x00000000, 0x00000000, 0xaaaaaaaa, 0xaaaaaaaa,
///     0xaaaaaaaa, 0xaaaaaaaa, 0x80008000]);
/// ```
pub struct UpdateChannel {
    instrs: Vec<u32>
}

impl UpdateChannel {

    /// Create a new instruction with invalid state.
    pub fn new() -> Self {
        Self::from_registers(&[&OpCode::UpdateChannel,
            &SourceConf::new(), &Empty::new(), &Empty::new(),
            &ChannelConf::new()])
    }

    /// Create a new instruction from existing configuration.
    pub fn from_regs(sourceconf: &SourceConf, chanconf: &ChannelConf) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::UpdateChannel);
        instr.push_register(sourceconf);
        instr.push_register(&Empty::new());
        instr.push_register(&Empty::new());
        instr.push_register(chanconf);

        instr
    }

    /// Create a new instruction with standard source configuration
    pub fn from_regs_default_source(chanconf: &ChannelConf) -> Self {
        let mut instr = UpdateChannel::create();
        instr.push_register(&OpCode::UpdateChannel);
        instr.push_register(&SourceConf::new());
        instr.push_register(&Empty::new());
        instr.push_register(&Empty::new());
        instr.push_register(chanconf);

        instr
    }

    /// Create a new instruction with default source and specified state
    ///
    /// ```
    /// use libarc2::registers::{ChannelState};
    /// use libarc2::instructions::{UpdateChannel, Instruction};
    ///
    /// // Arbitrary voltage output
    /// let state = ChannelState::VoltArb;
    ///
    /// let mut instr = UpdateChannel::from_regs_global_state(state);
    ///
    /// assert_eq!(instr.compile().view(), &[0x40, 0x73400000,
    ///     0x00000000, 0x00000000, 0xaaaaaaaa, 0xaaaaaaaa,
    ///     0xaaaaaaaa, 0xaaaaaaaa, 0x80008000]);
    ///
    /// ```
    pub fn from_regs_global_state(state: ChannelState) -> Self {
        let mut conf = ChannelConf::new();
        conf.set_all(state);
        Self::from_regs_default_source(&conf)
    }

    /// Generate a [`mask`][`ChanMask`] that contains all channels
    /// at the specified channel state.
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
    pub fn mask(&self, val: ChannelState) -> ChanMask {
        ChannelConf::from_raw_words(&self.instrs[4..8]).mask(val)
    }

}


impl Instruction for UpdateChannel { make_vec_instr_impl!(UpdateChannel, instrs, "UP CH"); }


/// Modify a channel with respect to ground
///
/// The `ModifyChannel` instruction will set the connections of the 64 channels with
/// respect to ground. The instruction can connect channels to GND, cap GND or the
/// current source.
///
/// ## Instruction layout
///
/// ```text
///                             GND      Cap GND    Cur Src
///        +--------+-------+----------+----------+----------+
///        | OpCode | Empty | ChanMask | ChanMask | ChanMask |
///        +--------+-------+----------+----------+----------+
/// Words:     1        1        2          2          2
/// ```
///
/// ## Example
/// ```
/// use libarc2::registers::ChanMask;
/// use libarc2::instructions::{Instruction, ModifyChannel};
///
/// // Do any Update Channel operations here
///
/// let mut chan_gnd = ChanMask::new();
/// let mut chan_capgnd = ChanMask::new();
/// let mut chan_cursrc = ChanMask::new();
///
/// // Set channel #0 to GND
/// chan_gnd.set_enabled(0, true);
/// // Set channel #1 to Cap GND
/// chan_capgnd.set_enabled(1, true);
/// // Connect channel #2 to the current source
/// chan_cursrc.set_enabled(2, true);
///
/// let mut instr = ModifyChannel::from_masks(&chan_gnd, &chan_capgnd, &chan_cursrc);
///
/// assert_eq!(instr.compile().view(), &[0x400, 0x0,
///     0x0, 0x1, 0x0, 0x2, 0x0, 0x4, 0x80008000]);
/// ```
pub struct ModifyChannel {
    instrs: Vec<u32>
}

impl ModifyChannel {

    pub fn new() -> Self {
        Self::from_registers(&[&OpCode::ModifyChannel,
            &Empty::new(), &ChanMask::new(),
            &ChanMask::new(), &ChanMask::new() ])
    }

    pub fn from_masks(gnd: &ChanMask, capgnd: &ChanMask, cursrc: &ChanMask) -> Self {
        Self::from_registers(&[&OpCode::ModifyChannel, &Empty::new(), gnd,
            capgnd, cursrc])
    }
}

impl Instruction for ModifyChannel { make_vec_instr_impl!(ModifyChannel, instrs, "MOD CH"); }

/// Delays with 20 ns precision
///
/// The `Delay` instruction is used to configure to insert delays into the ArC2
/// command buffer. This can be used for settling other instructions or creating
/// arbitrary waveforms. The delay is essentially a 50 MHz timer as described in
/// the documentation of [`Duration50`][`crate::registers::Duration50`]. The
/// maximum delay we can implement on board is `2^32 × 20 ns` although realistically
/// for delays greater than a second one might want to use software delays. Values
/// exceeding this maximum will be capped to fit. We cannot also do delays lower than
/// 320 ns (16 increments) since that's the time required by the FPGA to process the
/// instruction. So all arguments will be implicitly decremented by 320 ns to
/// account for that (hence the 320 ns minimum).
///
/// ## Instruction layout
///
/// ```text
///        +--------+------------+
///        | OpCode | Duration50 |
///        +--------+------------+
/// Words:     1          1
/// ```
///
/// ## Example
/// ```
/// use libarc2::instructions::{Delay, Instruction};
/// use std::time::Duration;
///
/// // Delays are rounded to the lowest increment of 20 ns
/// let mut instr0 = Delay::from_nanos(1210);
/// // Actual time is 320 ns (0x10 increments) lower than whatever the
/// // argument specifies to account for settling times
/// assert_eq!(instr0.compile().view(),
///     &[0x2000, 0x3C-0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);
///
/// // Same but with a `Duration` object
/// let mut instr1 = Delay::from_duration(&Duration::from_nanos(1210));
/// assert_eq!(instr1.compile().view(),
///     &[0x2000, 0x3C-0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);
///
/// let mut instr2 = Delay::from_nanos(200);
/// // Can't go below 320 ns (0x10 increments)
/// assert_eq!(instr2.compile().view(),
///     &[0x2000, 0x10-0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);
/// ```
pub struct Delay {
    instrs: Vec<u32>
}

impl Delay {

    pub(crate) const MIN_NS: u128 = 320;

    /// Create a new Delay instruction from the specified
    /// number of nanoseconds.
    pub fn from_nanos(ns: u128) -> Delay {
        let mut instr = Delay::create();

        let actual_ns = if ns <= Self::MIN_NS {
            0
        } else {
            ns - Self::MIN_NS
        };

        instr.push_register(&OpCode::Delay);
        instr.push_register(&Duration50::from_nanos(actual_ns));

        instr
    }

    pub fn from_duration(duration: &std::time::Duration) -> Delay {
        Delay::from_nanos(duration.as_nanos())
    }
}

impl Instruction for Delay { make_vec_instr_impl!(Delay, instrs, "DELAY"); }


/// Configure I/O logic.
///
/// This instruction enables or disables the logic I/Os and configures them
/// as either input or output relative to the FPGA. As a general guideline I/Os
/// should always be configured regardless whether they are actually
/// used or not.
///
/// ## Instruction layout
///
/// ```text
///        +--------+--------+------------+
///        | OpCode | IOMask |  IOEnable  |
///        +--------+--------+------------+
/// Words:     1        1          1
/// ```
///
/// ## Example
/// ```
/// use libarc2::instructions::{UpdateLogic, Instruction};
///
/// // Create a new update logic instruction setting all i/o as
/// // output (first argument) and enabling them (second argument).
/// let mut instr = UpdateLogic::new(true, true);
///
/// assert_eq!(instr.compile().view(),
///     &[0x20, 0xffffffff, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);
/// ```

pub struct UpdateLogic {
    instrs: Vec<u32>
}

impl UpdateLogic {

    /// Create a new instruction configuring _all_ I/O channels as
    /// either output (`true`) or input (`false`). If you want to
    /// tune individual channels use [`with_regs`][`UpdateLogic::with_regs`]
    /// with custom registers.
    pub fn new(output: bool, enable: bool) -> Self {

        let mut mask = IOMask::new();
        mask.set_enabled_all(output);

        let mut en = IOEnable::new();
        en.set_en(enable);

        Self::with_regs(&mask, &en)

    }

    /// Create a new instruction from registers.
    pub fn with_regs(iomask: &IOMask, ioenable: &IOEnable) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::UpdateLogic);
        instr.push_register(iomask);
        instr.push_register(ioenable);
        instr
    }

    /// Create a new output instruction from registers. This is
    /// essentially [`with_regs`][`UpdateLogic::with_regs`] but
    /// with all channels set as output and !EN enabled.
    pub fn with_mask(mask: &IOMask) -> Self {
        let mut ioenable = IOEnable::new();
        ioenable.set_en(true);

        Self::with_regs(&mask, &ioenable)
    }

    /// Create a new IO instruction on selected channels with specified
    /// cluster directions. Each cluster represents a block of contiguous
    /// GPIOs. Only channels defined in the GPIO mask will be affected.
    pub fn with_directions(cl0: IODir, cl1: IODir, cl2: IODir, cl3: IODir, mask: &IOMask) -> Self {
        let ioenable = IOEnable::with_iodirs(cl0, cl1, cl2, cl3);
        Self::with_regs(&mask, &ioenable)
    }
}

impl Instruction for UpdateLogic { make_vec_instr_impl!(UpdateLogic, instrs, "UP LGC"); }


/// Perform a current read operation on selected channels.
///
/// This will create a new current read operation on selected channels.
/// See documentation on [`ChanMask`][`crate::registers::ChanMask`] on how
/// to select one or more input channels.
///
/// ## Instruction layout
///
/// ```text
///        +--------+----------+---------+-------------+
///        | OpCode | ChanMask | Address | FlagAddress |
///        +--------+----------+---------+-------------+
/// Words:     1         2         1            1
/// ```
///
/// ## Example
///
/// ```
/// use libarc2::registers::{ChanMask, Address};
/// use libarc2::instructions::{CurrentRead, Instruction};
///
/// // Select channels 31, 0, 62
/// let mut mask = ChanMask::new();
/// mask.set_enabled(31, true);
/// mask.set_enabled(0, true);
/// mask.set_enabled(62, true);
///
/// let mut instr = CurrentRead::new(&mask, 0x60000000, 0x78000000, 0xcafebabe);
///
/// assert_eq!(instr.compile().view(), &[0x4, 0x40000000, 0x80000001,
///     0x60000000, 0x78000000, 0xcafebabe, 0x0, 0x0, 0x80008000]);
/// ```
pub struct CurrentRead {
    instrs: Vec<u32>
}

impl CurrentRead {

    /// Create a new current read instruction
    pub fn new(channels: &ChanMask, addr: u32, flag_addr: u32, flag: u32) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::CurrentRead);
        instr.push_register(channels);
        instr.push_register(&Address::new(addr));
        instr.push_register(&Address::new(flag_addr));
        instr.push_register(&Address::new(flag));
        instr
    }

}

impl Instruction for CurrentRead { make_vec_instr_impl!(CurrentRead, instrs, "C READ"); }


/// Perform a voltage read operation on selected channels.
///
/// This will create a new voltage read operation on selected channels.
/// See documentation on [`ChanMask`][`crate::registers::ChanMask`] on how
/// to select one or more input channels.
///
/// ## Instruction layout
///
/// ```text
///        +--------+---------+-----------+----------+-------------+
///        | OpCode | ChanMask | Averaging | Address | FlagAddress |
///        +--------+---------+-----------+----------+-------------+
/// Words:     1         2          1          1            1
/// ```
///
/// ## Example
///
/// ```
/// use libarc2::registers::{ChanMask, Address};
/// use libarc2::instructions::{VoltageRead, Instruction};
///
/// // Select channels 31, 0, 62
/// let mut mask = ChanMask::new();
/// mask.set_enabled(31, true);
/// mask.set_enabled(0, true);
/// mask.set_enabled(62, true);
///
/// let mut instr = VoltageRead::new(&mask, true, 0x60000000, 0x78000000, 0xcafebabe);
///
/// assert_eq!(instr.compile().view(), &[0x8, 0x40000000, 0x80000001,
///     0x1, 0x60000000, 0x78000000, 0xcafebabe, 0x0, 0x80008000]);
/// ```
pub struct VoltageRead {
    instrs: Vec<u32>
}

impl VoltageRead {

    /// Create a new voltage read instruction
    pub fn new(channels: &ChanMask, averaging: bool, addr: u32, flag_addr: u32, flag: u32) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::VoltageRead);
        instr.push_register(channels);
        if averaging {
            instr.push_register(&Averaging::Enabled);
        } else {
            instr.push_register(&Averaging::Disabled);
        }
        instr.push_register(&Address::new(addr));
        instr.push_register(&Address::new(flag_addr));
        instr.push_register(&Address::new(flag));
        instr
    }

}

impl Instruction for VoltageRead { make_vec_instr_impl!(VoltageRead, instrs, "V READ"); }


/// Setup High Speed drivers
///
/// This instruction is used to prepare the high speed driver for ultra-fast
/// pulsing.
///
/// ## Instruction layout
///
/// ```test
///        +--------+-----------+
///        | OpCode |  HSDelay  |
///        +--------+-----------+
/// Words:     1          7
/// ```
///
/// ## Example
/// ```
/// use libarc2::instructions::{HSConfig, Instruction};
/// use std::time::Duration;
///
/// // Configure cluster 0 to 110 ns, disable the rest
/// // DAC cluster:   0    1    2    3    4    5    6    7
/// let timings0 = [110,   0,   0,   0,   0,   0,   0,   0];
///
/// let mut conf0 = HSConfig::new(timings0);
///
/// // Real value is 10 ns lower to account for settling time
/// assert_eq!(conf0.compile().view(),
///     &[0x100, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xa, 0x80008000]);
///
/// let timings1 = [&Duration::from_nanos(110),
///                 &Duration::from_nanos(0),
///                 &Duration::from_nanos(0),
///                 &Duration::from_nanos(0),
///                 &Duration::from_nanos(0),
///                 &Duration::from_nanos(0),
///                 &Duration::from_nanos(0),
///                 &Duration::from_nanos(0)];
///
/// let mut conf1 = HSConfig::new_from_durations(timings1);
///
/// assert_eq!(conf1.compile().view(),
///     &[0x100, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xa, 0x80008000]);
/// ```
pub struct HSConfig {
    instrs: Vec<u32>
}

impl HSConfig {

    const MIN_DELAY: u32 = 40;
    const DEAD_TIME: u32 = 10;

    /// Create a new High Speed driver config with specified timings. Minimum
    /// supported delay is 40 ns and max is 2.68 s. You can still set a
    /// cluster to 0 for immediate transitions (or non high speed clusters)
    /// but values between 1 and 40 will be capped to 40 ns. For pulses longer
    /// than 200 ms the use of individual bias + delay steps is recommended
    /// instead.
    pub fn new(timings: [u32; 8]) -> Self {

        let mut instr = Self::create();

        let mut delays = HSDelay::new();
        for i in 0..timings.len() {
            // 0 is a valid value, but anything between 1 and 40 (incl.) is
            // capped to 40 ns.
            let actual_timing = if timings[i] == 0 {
                0u32
            } else if timings[i] > 0 && timings[i] <= Self::MIN_DELAY {
                Self::MIN_DELAY - Self::DEAD_TIME
            } else {
                timings[i] - Self::DEAD_TIME
            };

            // This is OK to unwrap since the cluster number can't go over
            // 7 as a result of the iteration
            delays.set_cluster_nanos(DACCluster::from_usize(i).unwrap(),
                actual_timing as u128);
        }

        instr.push_register(&OpCode::HSPulseConfig);
        instr.push_register(&delays);

        instr

    }

    /// Same as [`HSConfig::new()`] but with [`Duration`][`std::time::Duration`]
    /// arguments instead.
    pub fn new_from_durations(timings: [&std::time::Duration; 8]) -> Self {

        let mut delays: [u32; 8] = [0u32; 8];
        for i in 0..timings.len() {
            delays[i] = timings[i].as_nanos() as u32;
        }

        HSConfig::new(delays)

    }

}

impl Instruction for HSConfig { make_vec_instr_impl!(HSConfig, instrs, "HS CONF"); }


/// Initiate a high speed pulse operation
///
/// This instruction typically follows a [`HSConfig`] instruction and
/// actually initiates the high speed pulsing operation specified before.
///
/// ## Instruction layout
///
/// ```text
///        +--------+------------+
///        | OpCode | PulseAttrs |
///        +--------+------------+
/// Words:     1          1
/// ```
///
/// ## Example
///
/// ```
/// use libarc2::instructions::{HSConfig, HSPulse, Instruction};
/// use libarc2::registers::{PulseAttrs, ClusterMask, ToU32s};
///
/// // Configure cluster 0 for 110 ns pulses
/// let mut config  = HSConfig::new([110, 0, 0, 0, 0, 0, 0, 0]);
///
/// // arc2.process_compile(&config);
///
/// // Initiate a pulse on cluster 0
/// let mut pulse = HSPulse::new(ClusterMask::CL0);
///
/// // arc2.process_compile(&pulse);
///
/// assert_eq!(pulse.compile().view(), &[0x200, 0x00010000, 0x0,
///                                       0x0, 0x0, 0x0, 0x0, 0x0,
///                                       0x80008000]);
/// ```
pub struct HSPulse {
    instrs: Vec<u32>
}

impl HSPulse {

    /// Initiate a high speed pulse operation on the specified DAC
    /// clusters. This assumes normal low/high/low operation. For
    /// more elaborate pulsing schemes use
    /// [`new_from_attrs()`][`Self::new_from_attrs`] with custom
    /// [`PulseAttrs`].
    pub fn new(clusters: ClusterMask) -> Self {
        Self::new_from_attrs(&PulseAttrs::new(clusters))
    }

    /// Same as [`HSPulse::new`] but providing a slice of clusters
    /// to enable instead of a [`ClusterMask`].
    pub fn new_from_cluster_idx(clusters: &[u8]) -> Self {
        let clusters = ClusterMask::new_from_cluster_idx(clusters);
        Self::new(clusters)
    }

    pub fn new_from_attrs(attrs: &PulseAttrs) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::HSPulseStart);
        instr.push_register(attrs);

        instr
    }
}

impl Instruction for HSPulse { make_vec_instr_impl!(HSPulse, instrs, "HS PLS"); }

/// Configure selector circuits
///
/// This command is used to manage the on-board selector circuits. Unlike the
/// 32 generic I/Os selector circuits are exclusively output and they are
/// typically used for managing transistor gates or other EN signals for
/// ASICs.
///
/// Please note that this instruction only flips the outputs between high and
/// low. Actual voltages for low and high levels should be adjusted by setting
/// the SELL and SELH voltages respectively through a SetDAC instruction on
/// the AUX DACs. This is typically done on the top-level through the
/// [`Instrument::config_aux_channels`][`crate::instrument::Instrument::config_aux_channels()`]
/// function by setting [`AuxDACFn`][`crate::registers::AuxDACFn`]`.SELL` and
/// `.SELH`.
///
/// ## Instruction layout
/// ```text
///        +--------+--------------+
///        | OpCode | SelectorMask |
///        +--------+--------------+
/// Words:     1         1
/// ```
pub struct UpdateSelector {
    instrs: Vec<u32>
}

impl UpdateSelector {

    /// Create a new selector update instruction with
    /// the specified selector channel mask.
    pub fn new(channels: &SelectorMask) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::UpdateSelector);
        instr.push_register(channels);
        instr
    }

    /// Create a new selector update instructions with
    /// the specified selector channels pulled high.
    pub fn new_from_channels(chans: &[usize]) -> Result<Self, InstructionError> {
        let maxchan: usize = registers::consts::NSELECTORS;
        let mut actualchans: Vec<usize> = Vec::with_capacity(chans.len());

        // Convert the selector channels to actual selector bits
        for c in chans {
            if *c >= maxchan {
                return Err(InstructionError::ChannelRangeError(maxchan));
            }
            actualchans.push(SELECTORMAP[*c]);
        }
        let mask = SelectorMask::from_channels(&actualchans);
        let mut instr = Self::create();
        instr.push_register(&OpCode::UpdateSelector);
        instr.push_register(&mask);
        Ok(instr)
    }

}

impl Instruction for UpdateSelector { make_vec_instr_impl!(UpdateSelector, instrs, "UP SEL"); }

pub struct DACRange {
    instrs: Vec<u32>
}

impl DACRange {

    pub fn new(en_channels: &ChanMask, rng_channels: &ChanMask, en_aux_channels: &ArbMask, rng_aux_channels: &ArbMask) -> Self {
        let mut instr = DACRange::create();
        instr.push_register(&OpCode::DACRange);
        instr.push_register(en_aux_channels);
        instr.push_register(en_channels);
        instr.push_register(rng_aux_channels);
        instr.push_register(rng_channels);

        instr
    }

    pub fn new_with_ranges(chans: &ChanMask, rng_chans: &OutputRange, aux_chans: &ArbMask, rng_aux_chans: &OutputRange) -> Self {

        let rng_chans_actual: ChanMask;
        if *rng_chans == OutputRange::STD {
            rng_chans_actual = ChanMask::none();
        } else {
            rng_chans_actual = ChanMask::all();
        }

        let rng_aux_chans_actual: ArbMask;
        if *rng_aux_chans == OutputRange::STD {
            rng_aux_chans_actual = ArbMask::none();
        } else {
            rng_aux_chans_actual = ArbMask::all();
        }

        DACRange::new(chans, &rng_chans_actual, aux_chans, &rng_aux_chans_actual)

    }

    pub fn chans_at_range(chans: &ChanMask, rng: &OutputRange) -> Self {
        DACRange::new_with_ranges(chans, rng, &ArbMask::none(), &OutputRange::STD)
    }

    pub fn aux_chans_at_range(chans: &ArbMask, rng: &OutputRange) -> Self {
        DACRange::new_with_ranges(&ChanMask::none(), &OutputRange::STD, chans, rng)
    }
}

impl Instruction for DACRange { make_vec_instr_impl!(DACRange, instrs, "DAC RNG"); }

/// Connect feedback resistors to the op-amps
///
/// This command is used to connect the feedback resistors to the op-amps
/// in a controlled manner to avoid voltage transients on the channel. It
/// should be used before setting a channel as arbitrary voltage or when
/// commanding a current/voltage read.
///
/// ## Instruction layout
///
/// ```text
///        +--------+----------+
///        | OpCode | ChanMask |
///        +--------+----------+
/// Words:     1         2
/// ```
pub struct AmpPrep {
    instrs: Vec<u32>
}

impl AmpPrep {

    /// Create a new opamp prepare instruction
    pub fn new(channels: &ChanMask) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::AmpPrep);
        instr.push_register(channels);
        instr
    }

    /// Create a new opamp prepare instruction for the specified
    /// slice of channels
    pub fn new_from_channels(channels: &[usize]) -> Self {
        let mut instr = Self::create();
        let mut mask = ChanMask::new();
        for c in channels {
            mask.set_enabled(*c, true);
        }
        instr.push_register(&OpCode::AmpPrep);
        instr.push_register(&mask);

        instr
    }

}

impl Instruction for AmpPrep { make_vec_instr_impl!(AmpPrep, instrs, "AMP PRP"); }

/// Reset hardware to default state
///
/// This instruction resets all output DACs to 0.0 and disables I/O channels
///
/// ## Instruction layout
///
/// ```text
///        +--------+
///        | OpCode |
///        +--------+
/// Words:     1
/// ```
pub struct Clear {
    instrs: Vec<u32>
}

impl Clear {

    /// Create a new instruction
    pub fn new() -> Self {
        Self::from_registers(&[&OpCode::Clear])
    }
}

impl Instruction for Clear { make_vec_instr_impl!(Clear, instrs, "CLR"); }

#[cfg(test)]
mod tests {

    use crate::registers::*;
    use crate::instructions::*;

    #[test]
    fn new_reset_dac() {
        let mut instr = ResetDAC::new();

        assert_eq!(instr.compile().view(),
            &[0x1, 0xffff, 0x0, 0xf, 0x80008000, 0x80008000, 0x80008000,
              0x80008000, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x01, 0x00, 0x00, 0x00,
              0xFF, 0xFF, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x0F, 0x00, 0x00, 0x00,
              0x00, 0x80, 0x00, 0x80,
              0x00, 0x80, 0x00, 0x80,
              0x00, 0x80, 0x00, 0x80,
              0x00, 0x80, 0x00, 0x80,
              0x00, 0x80, 0x00, 0x80]);
    }

    #[test]
    fn new_update_dac() {
        let mut instr = UpdateDAC::new();
        assert_eq!(instr.compile().view(),
            &[0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x02, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x80, 0x00, 0x80]);
    }

    #[test]
    fn new_clear() {
        let mut instr = Clear::new();
        assert_eq!(instr.compile().view(),
            &[0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x80, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x80, 0x00, 0x80]);
    }

    #[test]
    fn new_set_dac_with_regs() {

        // Enable first two half channels (8 channels)
        let mut mask = DACMask::NONE;
        mask.set_channels(&[0, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(mask, DACMask::CH00_03 | DACMask::CH04_07);

        // Set the voltages to
        // CH0 0x0000_8000 channel 0 (low) and 1 (high)
        // CH1 0x8000_8000 channel 2 (low) and 3 (high)
        // CH2 0x8000_9000 channel 4 (low) and 5 (high)
        // CH3 0x9000_FFFF channel 6 (low) and 7 (high)
        let mut voltages = DACVoltage::new();
        voltages.set_upper(0, 0x8000);
        voltages.set_lower( 0, 0x0000);
        voltages.set_upper(1, 0x8000);
        voltages.set_lower( 1, 0x8000);
        voltages.set_upper(2, 0x9000);
        voltages.set_lower( 2, 0x8000);
        voltages.set_upper(3, 0xFFFF);
        voltages.set_lower( 3, 0x9999);
        assert_eq!(voltages.as_u32s(),
            &[0x80000000, 0x80008000, 0x90008000, 0xFFFF9999]);

        let mut instr = SetDAC::with_regs(&mask, &voltages, &DACVoltageMask::ALL).unwrap();

        assert_eq!(instr.compile().view(),
            &[0x1, 0x3, 0x0, 0x0000000f, 0x80000000, 0x80008000,
              0x90008000, 0xFFFF9999, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x01, 0x00, 0x00, 0x00,
              0x03, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x0f, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x80,
              0x00, 0x80, 0x00, 0x80,
              0x00, 0x80, 0x00, 0x90,
              0x99, 0x99, 0xFF, 0xFF,
              0x00, 0x80, 0x00, 0x80]);

    }

    #[test]
    fn new_set_dac_edit_regs() {

        // Enable first two half channels (8 channels)
        let mut mask = DACMask::NONE;
        mask.set_channels(&[0, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(mask, DACMask::CH00_03 | DACMask::CH04_07);

        // Set the voltages to
        // CH0 0x0000_8000 channel 0 (high) and 1 (low)
        // CH1 0x8000_8000 channel 2 (high) and 3 (low)
        // CH2 0x8000_9000 channel 4 (high) and 5 (low)
        // CH3 0x9000_FFFF channel 6 (high) and 7 (low)
        let mut voltages = DACVoltage::new();
        voltages.set_upper(0, 0x0000);
        voltages.set_lower( 0, 0x8000);
        voltages.set_upper(1, 0x8000);
        voltages.set_lower( 1, 0x8000);
        voltages.set_upper(2, 0x8000);
        voltages.set_lower( 2, 0x9000);
        voltages.set_upper(3, 0x9000);
        voltages.set_lower( 3, 0xFFFF);
        assert_eq!(voltages.as_u32s(),
            &[0x00008000, 0x80008000, 0x80009000, 0x9000FFFF]);

        let mut instr = SetDAC::new();

        instr.set_mask(&mask);
        instr.set_voltages(&voltages);

        assert_eq!(instr.compile().view(),
            &[0x1, 0x3, 0x0, 0x0, 0x8000, 0x80008000,
              0x80009000, 0x9000FFFF, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x01, 0x00, 0x00, 0x00,
              0x03, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x80, 0x00, 0x00,
              0x00, 0x80, 0x00, 0x80,
              0x00, 0x90, 0x00, 0x80,
              0xFF, 0xFF, 0x00, 0x90,
              0x00, 0x80, 0x00, 0x80]);

    }

    #[test]
    fn new_update_channel_with_regs() {

        let mut chanconf = ChannelConf::new();
        chanconf.set_all(ChannelState::VoltArb);

        let sourceconf = SourceConf::new();

        let mut instr = UpdateChannel::from_regs(&sourceconf, &chanconf);

        assert_eq!(instr.compile().view(), &[0x40, 0x73400000,
            0x00000000, 0x00000000, 0xaaaaaaaa, 0xaaaaaaaa,
            0xaaaaaaaa, 0xaaaaaaaa, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x40, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x40, 0x73,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0xaa, 0xaa, 0xaa, 0xaa,
              0xaa, 0xaa, 0xaa, 0xaa,
              0xaa, 0xaa, 0xaa, 0xaa,
              0xaa, 0xaa, 0xaa, 0xaa,
              0x00, 0x80, 0x00, 0x80]);
    }

    #[test]
    fn new_update_logic() {
        let mut instr = UpdateLogic::new(true, true);

        assert_eq!(instr.compile().view(),
            &[0x20, 0xffffffff, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x20, 0x00, 0x00, 0x00,
              0xff, 0xff, 0xff, 0xff,
              0x0f, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x80, 0x00, 0x80]);
    }

    #[test]
    fn new_current_read() {

        let mut mask = ChanMask::new();
        mask.set_enabled(31, true);
        mask.set_enabled(0, true);
        mask.set_enabled(62, true);


        let mut instr = CurrentRead::new(&mask, 0x60000000, 0x78000000, 0xcafebabe);

        assert_eq!(instr.compile().view(), &[0x4, 0x40000000, 0x80000001,
            0x60000000, 0x78000000, 0xcafebabe, 0x0, 0x0, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x04, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x40,
              0x01, 0x00, 0x00, 0x80,
              0x00, 0x00, 0x00, 0x60,
              0x00, 0x00, 0x00, 0x78,
              0xbe, 0xba, 0xfe, 0xca,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x80, 0x00, 0x80]);
    }

    #[test]
    fn new_voltage_read() {

        let mut mask = ChanMask::new();
        mask.set_enabled(31, true);
        mask.set_enabled(0, true);
        mask.set_enabled(62, true);

        let mut instr = VoltageRead::new(&mask, true, 0x60000000, 0x78000000, 0xcafebabe);

        assert_eq!(instr.compile().view(), &[0x8, 0x40000000, 0x80000001,
            0x1, 0x60000000, 0x78000000, 0xcafebabe, 0x0, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x08, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x40,
              0x01, 0x00, 0x00, 0x80,
              0x01, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x60,
              0x00, 0x00, 0x00, 0x78,
              0xbe, 0xba, 0xfe, 0xca,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x80, 0x00, 0x80]);
    }

}
