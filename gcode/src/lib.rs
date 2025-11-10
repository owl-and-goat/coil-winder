#![cfg_attr(not(test), no_std)]
#![feature(int_from_ascii)]

mod ast;
mod parser;

pub use ast::{Command, UCoord, UPos};

#[cfg(test)]
mod tests {
    // use super::*;
}
