//! Commands processable by ArC2

use std::collections::HashMap;
use crate::registers::ToU32s;
use crate::registers::Terminate;
use crate::registers::{OpCode, Empty, DACMask, DACVoltage};
use crate::registers::{ChannelConf, SourceConf, ChannelState};
use crate::registers::{IOEnable, IOMask, ChanMask, Averaging};
use crate::registers::{Duration50, Address, HSDelay, DACCluster};
use crate::registers::{ClusterMask, PulseAttrs};
use num_traits::FromPrimitive;

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
    /// ```ignore
    /// use libarc2::{Instrument};
    /// use libarc2::instructions::{ResetDAC, Instruction};
    ///
    /// let arc2 = Instrument::open_with_fw(0, "fw.bin").unwrap();
    ///
    /// let reset = ResetDAC::new();
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
///        +--------+--------------+-------+-------+------------------+
///        | OpCode | DACMask::ALL | Empty | Empty | DACVoltage::ZERO |
///        +--------+--------------+-------+-------+------------------+
/// Words:     1           1           1       1            4
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
                               &Empty::new(),
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
///        +--------+----------+--------+--------+------------+
///        | OpCode |  DACMask |  Empty |  Empty | DACVoltage |
///        +--------+----------+--------+--------+------------+
/// Words:     1         1         1        1          4
/// ```
///
/// ## Example
/// ```
/// use libarc2::registers::{DACMask, DACVoltage, ToU32s};
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
/// voltages.set_high(0, 0x0000);
/// voltages.set_low(0, 0x8000);
/// voltages.set_high(1, 0x8000);
/// voltages.set_low(1, 0x8000);
/// voltages.set_high(2, 0x8000);
/// voltages.set_low(2, 0x9000);
/// voltages.set_high(3, 0x9000);
/// voltages.set_low(3, 0xFFFF);
///
/// let mut instr = SetDAC::with_regs(&mask, &voltages);
///
/// assert_eq!(instr.compile().view(),
///     &[0x1, 0x3, 0x0, 0x0, 0x8000, 0x80008000,
///       0x80009000, 0x9000FFFF, 0x80008000]);
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
        instr.push_register(&Empty::new());
        instr.push_register(&DACVoltage::new());

        instr
    }

    /// Create a new instruction with all channels set at
    /// the specified levels
    pub fn new_all_at_bias(low: u16, high: u16) -> Self {
        let mut instr = SetDAC::create();
        instr.push_register(&OpCode::SetDAC);
        instr.push_register(&DACMask::ALL);
        instr.push_register(&Empty::new());
        instr.push_register(&Empty::new());
        instr.push_register(&DACVoltage::new_at_levels(low, high));

        instr
    }

    /// Create a new logic instruction
    ///
    /// This instruction operates on the AUX1 DAC which is used
    /// to set the logic level for the board. This function will
    /// return an instruction suitable for 3.3V logic;
    pub fn new_3v3_logic() -> Self {
        let mut instr = SetDAC::create();
        instr.push_register(&OpCode::SetDAC);
        instr.push_register(&DACMask::AUX1);
        instr.push_register(&Empty::new());
        instr.push_register(&Empty::new());

        let mut voltages = DACVoltage::new();
        voltages.set_high(0, 0x0000);
        voltages.set_low(0, 0x0000);

        // Voltage will be divided by 2.62 internally to get
        // the actual voltage: 8.646/2.62 = 3.3.
        voltages.set_high(1, vidx!(8.646));
        voltages.set_low(1, 0x0000);
        voltages.set_high(2, 0x0000);
        voltages.set_low(2, 0x0000);
        voltages.set_high(3, 0x0000);
        voltages.set_low(3, 0x0000);
        instr.push_register(&voltages);

        instr
    }

    /// Create a new instruction with specified registers
    pub fn with_regs(chanmask: &DACMask, voltages: &DACVoltage) -> Self {
        let mut instr = SetDAC::create();
        instr.push_register(&OpCode::SetDAC);
        instr.push_register(chanmask);
        instr.push_register(&Empty::new());
        instr.push_register(&Empty::new());
        instr.push_register(voltages);

        instr
    }

    /// Assemble a minimum set of instructions that correspond to the supplied
    /// voltages and channels. The argument of this function is a slice of
    /// tuples that conform to this specification
    ///
    /// ```text
    /// (channel idx, DAC- voltage, DAC+ voltage)
    /// ```
    /// Unselected channels will be set to whatever the `base` argument indicates.
    /// Again the format is (DAC- voltage, DAC+ voltage).
    ///
    /// If `clear` is set to true an additional instruction will
    /// be emitted to ensure that all clusters are set to `base` voltage
    /// _before_ any further SetDAC instructions are emitted.
    ///
    /// ## Example
    /// ```
    /// use libarc2::instructions::{SetDAC, Instruction};
    ///
    /// let input: Vec<(u16, u16, u16)> = vec![
    ///    ( 0, 0x8ccc, 0x8ccc), ( 1, 0x8ccc, 0x8ccc), ( 2, 0x6000, 0x7000),
    ///    ( 4, 0x8ccc, 0x8ccc), ( 5, 0x8ccc, 0x8ccc), ( 6, 0x6000, 0x7000),
    ///    (12, 0x8ccc, 0x8ccc), (13, 0x8ccc, 0x8ccc), (14, 0x6000, 0x7000),
    ///    (22, 0x7000, 0x7000), (17, 0x6000, 0x7000), (63, 0x7000, 0x9000)
    /// ];
    ///
    /// let mut instructions = SetDAC::from_channels(&input, (0x8000, 0x8000), false);
    ///
    /// assert_eq!(instructions.len(), 4);
    /// assert_eq!(instructions[0].compile().view(),
    ///     &[0x00000001, 0x0000000b, 0x00000000, 0x00000000, 0x8ccc8ccc,
    ///       0x8ccc8ccc, 0x70006000, 0x80008000, 0x80008000]);
    /// assert_eq!(instructions[1].compile().view(),
    ///     &[0x00000001, 0x00000010, 0x00000000, 0x00000000, 0x80008000,
    ///       0x70006000, 0x80008000, 0x80008000, 0x80008000]);
    /// assert_eq!(instructions[2].compile().view(),
    ///     &[0x00000001, 0x00000020, 0x00000000, 0x00000000, 0x80008000,
    ///       0x80008000, 0x70007000, 0x80008000, 0x80008000]);
    /// assert_eq!(instructions[3].compile().view(),
    ///     &[0x00000001, 0x00008000, 0x00000000, 0x00000000, 0x80008000,
    ///       0x80008000, 0x80008000, 0x90007000, 0x80008000]);
    ///
    /// // If `clear` is set an additional instruction will be
    /// // emitted first to set all channels to the specified voltage.
    /// // In this case all inactive channels are at 0.0 V.
    /// let mut instructions = SetDAC::from_channels(&input, (0x8000, 0x8000), true);
    ///
    /// assert_eq!(instructions.len(), 5);
    /// assert_eq!(instructions[0].compile().view(),
    ///     &[0x00000001, 0x0000ffff, 0x00000000, 0x00000000, 0x80008000,
    ///       0x80008000, 0x80008000, 0x80008000, 0x80008000]);
    /// assert_eq!(instructions[1].compile().view(),
    ///     &[0x00000001, 0x0000000b, 0x00000000, 0x00000000, 0x8ccc8ccc,
    ///       0x8ccc8ccc, 0x70006000, 0x80008000, 0x80008000]);
    /// assert_eq!(instructions[2].compile().view(),
    ///     &[0x00000001, 0x00000010, 0x00000000, 0x00000000, 0x80008000,
    ///       0x70006000, 0x80008000, 0x80008000, 0x80008000]);
    /// assert_eq!(instructions[3].compile().view(),
    ///     &[0x00000001, 0x00000020, 0x00000000, 0x00000000, 0x80008000,
    ///       0x80008000, 0x70007000, 0x80008000, 0x80008000]);
    /// assert_eq!(instructions[4].compile().view(),
    ///     &[0x00000001, 0x00008000, 0x00000000, 0x00000000, 0x80008000,
    ///       0x80008000, 0x80008000, 0x90007000, 0x80008000]);
    ///
    /// ```
    pub fn from_channels(input: &[(u16, u16, u16)], base: (u16, u16), clear: bool) -> Vec<Self> {
        let mut result: Vec<SetDAC> = Vec::new();

        let (low, high) = (base.0, base.1);

        if clear {
            result.push(SetDAC::new_all_at_bias(low, high));
        }

        let mut channels: [(u16, u16); 64] = [(low, high); 64];
        let mut bucket: HashMap<[u32; 4], Vec<u16>> = HashMap::new();
        let mut keys: Vec<[u32; 4]> = Vec::with_capacity(64);

        for (idx, low, high) in input {
            channels[*idx as usize] = (*low, *high);
        }

        for (idx, chunk) in channels.chunks(4usize).enumerate() {

            let unchanged: bool = {
                let mut res = true;
                for (clow, chigh) in chunk {
                    if *clow != low || *chigh != high {
                        res = false;
                        break;
                    }
                }
                res
            };

            if unchanged { continue; }

            let key = {
                let mut res: [u32; 4] = [0x80008000; 4];
                for _idx in 0..4usize {
                    let value = chunk[_idx];
                    res[_idx as usize] = dacvoltage!(value.0, value.1);
                }
                res
            };

            if bucket.contains_key(&key) {
                let entry = bucket.get_mut(&key).unwrap();
                entry.push(2u16.pow(idx as u32) as u16);
            } else {
                keys.push(key);
                bucket.insert(key, vec![2u16.pow(idx as u32) as u16]);
            }

        }

        for key in keys {
            let bits: u32 = (*bucket.get(&key).unwrap()).iter().sum::<u16>() as u32;
            let mask = DACMask::from_bits(bits).unwrap();
            let voltages = DACVoltage::from_raw_values(&key);

            result.push(SetDAC::with_regs(&mask, &voltages));
        }


        result
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
///        +--------+-------------+---------------+
///        | OpCode |  SourceConf |  ChannelConf  |
///        +--------+-------------+---------------+
/// Words:     1            1             6
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

    /// Alter the instruction's channel configuration.
    pub fn set_channel_conf(&mut self, conf: &ChannelConf) {
        let chan_conf = conf.as_u32s();
        for (idx, num) in chan_conf.iter().enumerate() {
            self.instrs[idx+2] = *num;
        }
    }

    /// Alter the instruction's current source configuration.
    pub fn set_source_conf(&mut self, conf: &SourceConf) {
        let chan_conf = conf.as_u32s();
        for (idx, num) in chan_conf.iter().enumerate() {
            self.instrs[idx+1] = *num;
        }
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
            &[0x1, 0xffff, 0x0, 0x0, 0x80008000, 0x80008000, 0x80008000,
              0x80008000, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x01, 0x00, 0x00, 0x00,
              0xFF, 0xFF, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
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
        // CH0 0x0000_8000 channel 0 (high) and 1 (low)
        // CH1 0x8000_8000 channel 2 (high) and 3 (low)
        // CH2 0x8000_9000 channel 4 (high) and 5 (low)
        // CH3 0x9000_FFFF channel 6 (high) and 7 (low)
        let mut voltages = DACVoltage::new();
        voltages.set_high(0, 0x0000);
        voltages.set_low( 0, 0x8000);
        voltages.set_high(1, 0x8000);
        voltages.set_low( 1, 0x8000);
        voltages.set_high(2, 0x8000);
        voltages.set_low( 2, 0x9000);
        voltages.set_high(3, 0x9000);
        voltages.set_low( 3, 0xFFFF);
        assert_eq!(voltages.as_u32s(),
            &[0x00008000, 0x80008000, 0x80009000, 0x9000FFFF]);

        let mut instr = SetDAC::with_regs(&mask, &voltages);

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
        voltages.set_high(0, 0x0000);
        voltages.set_low( 0, 0x8000);
        voltages.set_high(1, 0x8000);
        voltages.set_low( 1, 0x8000);
        voltages.set_high(2, 0x8000);
        voltages.set_low( 2, 0x9000);
        voltages.set_high(3, 0x9000);
        voltages.set_low( 3, 0xFFFF);
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
    fn new_set_dac_3v3_logic() {
        let mut instr = SetDAC::new_3v3_logic();

        assert_eq!(instr.compile().view(), &[0x1, 0x20000, 0x0, 0x0, 0x0,
            0xeeab0000, 0x0, 0x0, 0x80008000]);

        assert_eq!(instr.to_bytevec(),
            &[0x01, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x02, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0xab, 0xee,
              0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00,
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
