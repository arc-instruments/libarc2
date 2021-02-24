//use ndarray::{Array, Ix1};

mod instrument;
mod registers;
mod instructions;

pub mod register {

    //! Operations that can be serialised into ArC2 instructions.
    //!
    //! A register is a piece of u32-encodable information that can is used
    //! to form one or more instructions that ArC2 can parse. All registers
    //! are expected to implement the [`ToU32s`][`crate::registers::ToU32s`]
    //! trait that converts them into a serialisable `Vec<u32>`. This can be
    //! then be processed by ArC2.

    pub use crate::registers::channelconf::*;
    pub use crate::registers::opcode::OpCode;
    pub use crate::registers::terminate::Terminate;
    pub use crate::registers::empty::Empty;
    pub use crate::registers::dacmask::DACMask;
    pub use crate::registers::u32mask::U32Mask;
    pub use crate::registers::adcmask::ADCMask;
    pub use crate::registers::iomask::IOMask;
    pub use crate::registers::ioenable::IOEnable;
    pub use crate::registers::dacvoltage::DACVoltage;
    pub use crate::registers::sourceconf::*;

    pub use crate::registers::ToU32s;
}

pub use crate::instrument::*;
pub use crate::instructions::*;
