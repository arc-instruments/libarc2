use crate::register::ToU32s;
use crate::register::Terminate;
use crate::register::{OpCode, Empty, DACMask, DACVoltage};
use crate::register::{ChannelConf, SourceConf, ChannelState};
use crate::register::{IOEnable, IOMask, ADCMask, Averaging};

macro_rules! make_vec_instr_impl {
    ($t:ident, $f:ident) => {

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
    }
}


/// Common behaviour of all available instructions.
///
/// The Instruction trait is implemented by all instructions. Typically
/// instructions must implement the `create`, `push_u32s`, `len` and `view`
/// functions. Everything else is derived from these.
///
/// Instructions can be "compiled" into a `&[u32]` buffer that is
/// suitable padded and terminated and can be written directly with
/// [`write_packet`][`crate::Instrument::write_packet`].
pub trait Instruction {

    #[doc(hidden)]
    const LENGTH: usize = 9;

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

    /// Pad and terminate instruction for consumption. This will be
    /// typically communicated to ArC2 using the
    /// [`write_packet`][`crate::Instrument::write_packet`].
    ///
    /// ```ignore
    /// use libarc2::{Instrument, ResetDAC, Instruction};
    ///
    /// let arc2 = Instrument::open_with_fw(0, "fw.bin").unwrap();
    ///
    /// let reset = ResetDAC::new();
    /// arc2.write_packet(reset.compile());
    /// ```
    fn compile(&mut self) -> &[u32] {
        for _ in 0..(Self::LENGTH-self.len()-1) {
            self.push_register(&Empty::new());
        }

        for _ in 0..(Self::LENGTH-self.len()) {
            self.push_register(&Terminate::new());
        }

        &self.view()
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

impl Instruction for ResetDAC { make_vec_instr_impl!(ResetDAC, instrs); }


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

impl Instruction for UpdateDAC { make_vec_instr_impl!(UpdateDAC, instrs); }


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
/// use libarc2::register::{DACMask, DACVoltage, ToU32s};
/// use libarc2::{SetDAC, Instruction};
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
/// assert_eq!(instr.compile(),
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

impl Instruction for SetDAC { make_vec_instr_impl!(SetDAC, instrs); }


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
/// use libarc2::register::{ChannelConf, SourceConf, ChannelState};
/// use libarc2::{UpdateChannel, Instruction};
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
/// assert_eq!(instr.compile(), &[0x40, 0x73400000,
///     0x92492492, 0x49249249, 0x24924924, 0x92492492,
///     0x49249249, 0x24924924, 0x80008000]);
/// ```
pub struct UpdateChannel {
    instrs: Vec<u32>
}

impl UpdateChannel {

    /// Create a new instruction with invalid state.
    pub fn new() -> Self {
        Self::from_registers(&[&OpCode::UpdateChannel,
            &SourceConf::new(), &ChannelConf::new()])
    }

    /// Create a new instruction from existing configuration.
    pub fn from_regs(sourceconf: &SourceConf, chanconf: &ChannelConf) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::UpdateChannel);
        instr.push_register(sourceconf);
        instr.push_register(chanconf);

        instr
    }

    /// Create a new instruction with standard source configuration
    pub fn from_regs_default_source(chanconf: &ChannelConf) -> Self {
        let mut instr = UpdateChannel::create();
        instr.push_register(&OpCode::UpdateChannel);
        instr.push_register(&SourceConf::new());
        instr.push_register(chanconf);

        instr
    }

    /// Create a new instruction with default source and specified state
    ///
    /// ```
    /// use libarc2::register::{ChannelState};
    /// use libarc2::{UpdateChannel, Instruction};
    ///
    /// // Arbitrary voltage output
    /// let state = ChannelState::VoltArb;
    ///
    /// let mut instr = UpdateChannel::from_regs_global_state(state);
    ///
    /// assert_eq!(instr.compile(), &[0x40, 0x73400000,
    ///     0x92492492, 0x49249249, 0x24924924, 0x92492492,
    ///     0x49249249, 0x24924924, 0x80008000]);
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

impl Instruction for UpdateChannel { make_vec_instr_impl!(UpdateChannel, instrs); }


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
/// use libarc2::{UpdateLogic, Instruction};
///
/// // Create a new update logic instruction setting all i/o as
/// // output (first argument) and enabling them (second argument).
/// let mut instr = UpdateLogic::new(true, true);
///
/// assert_eq!(instr.compile(),
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

        let mut dir = IOEnable::new();
        dir.set_enabled(enable);

        Self::with_regs(&mask, &dir)

    }

    /// Create a new instruction from registers.
    pub fn with_regs(iomask: &IOMask, ioenable: &IOEnable) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::UpdateLogic);
        instr.push_register(iomask);
        instr.push_register(ioenable);
        instr
    }
}

impl Instruction for UpdateLogic { make_vec_instr_impl!(UpdateLogic, instrs); }


/// Perform a current read operation on selected channels.
///
/// This will create a new current read operation on selected channels.
/// See documentation on [`ADCMask`][`crate::register::ADCMask`] on how
/// to select one or more input channels.
///
/// ## Instruction layout
///
/// ```text
///        +--------+---------+
///        | OpCode | ADCMask |
///        +--------+---------+
/// Words:     1        2
/// ```
///
/// ## Example
///
/// ```
/// use libarc2::register::ADCMask;
/// use libarc2::{CurrentRead, Instruction};
///
/// // Select channels 31, 0, 62
/// let mut mask = ADCMask::new();
/// mask.set_enabled(31, true);
/// mask.set_enabled(0, true);
/// mask.set_enabled(62, true);
///
/// let mut instr = CurrentRead::new(&mask);
///
/// assert_eq!(instr.compile(), &[0x4, 0x40000000, 0x80000001,
///     0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);
/// ```
pub struct CurrentRead {
    instrs: Vec<u32>
}

impl CurrentRead {

    /// Create a new current read instruction
    pub fn new(channels: &ADCMask) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::CurrentRead);
        instr.push_register(channels);
        instr
    }

}

impl Instruction for CurrentRead { make_vec_instr_impl!(CurrentRead, instrs); }


/// Perform a voltage read operation on selected channels.
///
/// This will create a new voltage read operation on selected channels.
/// See documentation on [`ADCMask`][`crate::register::ADCMask`] on how
/// to select one or more input channels.
///
/// ## Instruction layout
///
/// ```text
///        +--------+---------+-----------+
///        | OpCode | ADCMask | Averaging |
///        +--------+---------+-----------+
/// Words:     1         2          1
/// ```
///
/// ## Example
///
/// ```
/// use libarc2::register::ADCMask;
/// use libarc2::{VoltageRead, Instruction};
///
/// // Select channels 31, 0, 62
/// let mut mask = ADCMask::new();
/// mask.set_enabled(31, true);
/// mask.set_enabled(0, true);
/// mask.set_enabled(62, true);
///
/// let mut instr = VoltageRead::new(&mask, true);
///
/// assert_eq!(instr.compile(), &[0x8, 0x40000000, 0x80000001,
///     0x1, 0x0, 0x0, 0x0, 0x0, 0x80008000]);
/// ```
pub struct VoltageRead {
    instrs: Vec<u32>
}

impl VoltageRead {

    /// Create a new voltage read instruction
    pub fn new(channels: &ADCMask, averaging: bool) -> Self {
        let mut instr = Self::create();
        instr.push_register(&OpCode::VoltageRead);
        instr.push_register(channels);
        if averaging {
            instr.push_register(&Averaging::Enabled);
        } else {
            instr.push_register(&Averaging::Disabled);
        }
        instr
    }

}

impl Instruction for VoltageRead { make_vec_instr_impl!(VoltageRead, instrs); }

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

impl Instruction for Clear { make_vec_instr_impl!(Clear, instrs); }

#[cfg(test)]
mod tests {

    use crate::register::*;
    use super::{ResetDAC, UpdateDAC, SetDAC, UpdateChannel, Clear, Instruction};
    use super::{UpdateLogic, CurrentRead, VoltageRead};

    #[test]
    fn new_reset_dac() {
        let mut instr = ResetDAC::new();

        assert_eq!(instr.compile(),
            &[0x1,
              0xffff,
              0x0,
              0x0,
              0x80008000,
              0x80008000,
              0x80008000,
              0x80008000,
              0x80008000]);
    }

    #[test]
    fn new_update_dac() {
        let mut instr = UpdateDAC::new();
        assert_eq!(instr.compile(),
            &[0x2,
              0x0,
              0x0,
              0x0,
              0x0,
              0x0,
              0x0,
              0x0,
              0x80008000]);
    }

    #[test]
    fn new_clear() {
        let mut instr = Clear::new();
        assert_eq!(instr.compile(),
            &[0x80,
              0x0,
              0x0,
              0x0,
              0x0,
              0x0,
              0x0,
              0x0,
              0x80008000]);
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

        assert_eq!(instr.compile(),
            &[0x1, 0x3, 0x0, 0x0, 0x8000, 0x80008000,
              0x80009000, 0x9000FFFF, 0x80008000]);

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

        assert_eq!(instr.compile(),
            &[0x1, 0x3, 0x0, 0x0, 0x8000, 0x80008000,
              0x80009000, 0x9000FFFF, 0x80008000]);

    }

    #[test]
    fn new_update_channel_with_regs() {

        let mut chanconf = ChannelConf::new();
        chanconf.set_all(ChannelState::VoltArb);

        let sourceconf = SourceConf::new();

        let mut instr = UpdateChannel::from_regs(&sourceconf, &chanconf);

        assert_eq!(instr.compile(), &[0x40, 0x73400000,
            0x92492492, 0x49249249, 0x24924924, 0x92492492,
            0x49249249, 0x24924924, 0x80008000]);
    }

    #[test]
    fn new_update_logic() {
        let mut instr = UpdateLogic::new(true, true);

        assert_eq!(instr.compile(),
            &[0x20, 0xffffffff, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);
    }

    #[test]
    fn new_current_read() {

        let mut mask = ADCMask::new();
        mask.set_enabled(31, true);
        mask.set_enabled(0, true);
        mask.set_enabled(62, true);

        let mut instr = CurrentRead::new(&mask);

        assert_eq!(instr.compile(), &[0x4, 0x40000000, 0x80000001,
            0x0, 0x0, 0x0, 0x0, 0x0, 0x80008000]);
    }

    #[test]
    fn new_voltage_read() {

        let mut mask = ADCMask::new();
        mask.set_enabled(31, true);
        mask.set_enabled(0, true);
        mask.set_enabled(62, true);

        let mut instr = VoltageRead::new(&mask, true);

        assert_eq!(instr.compile(), &[0x8, 0x40000000, 0x80000001,
            0x1, 0x0, 0x0, 0x0, 0x0, 0x80008000]);
    }

}
