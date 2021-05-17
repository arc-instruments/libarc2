#[macro_use]
extern crate lazy_static;

#[macro_use]
mod macros;

mod memory;
mod instrument;
pub mod registers;
pub mod instructions;

pub use crate::instrument::*;
