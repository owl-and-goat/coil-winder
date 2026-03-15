#![cfg_attr(not(test), no_std)]
#![feature(int_from_ascii)]

mod ast;
mod parser;

pub use ast::{Command, DisplayCommand, UCoord, UPos};
use nom::{Parser, character::streaming::newline, sequence::terminated};

pub enum Error {
    ParseFailed,
    Incomplete(nom::Needed),
}

pub fn parse_single_command<const AXES: usize>(
    axis_labels: [char; AXES],
    input: &[u8],
) -> Result<(&[u8], Command<AXES>), Error> {
    match terminated(parser::command(axis_labels), newline).parse(input) {
        Ok((i, res)) => Ok((i, res)),
        Err(nom::Err::Incomplete(needed)) => Err(Error::Incomplete(needed)),
        Err(_) => Err(Error::ParseFailed),
    }
}
