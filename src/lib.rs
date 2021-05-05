//use ndarray::{Array, Ix1};
#[macro_use]
extern crate lazy_static;

#[macro_use]
mod macros;

mod instrument;
pub mod registers;
pub mod instructions;

pub use crate::instrument::*;
