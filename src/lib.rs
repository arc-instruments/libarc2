#![doc(html_logo_url = "../arci.svg")]
#[macro_use]
extern crate lazy_static;

#[macro_use]
mod macros;

mod memory;
mod instrument;
pub mod registers;
pub mod instructions;

pub use crate::instrument::*;

pub const LIBARC2_VERSION: Option<&'static str> = option_env!("CARGO_PKG_VERSION");
