#![allow(dead_code)] // FIXME

use heapless::Vec;
use nom::{
    branch::alt,
    bytes::streaming::tag,
    character::{complete::multispace1, streaming::char},
    combinator::opt,
    error::ErrorKind,
    number::complete::recognize_float,
    IResult, Parser,
};

use crate::ast::{Command, UCoord, UPos};

pub fn ucoord(i: &[u8]) -> IResult<&[u8], UCoord> {
    let (i, txt) = recognize_float(i)?;
    let num = UCoord::from_ascii(txt).unwrap();
    Ok((i, num))
}

pub fn labeled_ucoord(label: char) -> impl Fn(&[u8]) -> IResult<&[u8], UCoord> {
    move |i| {
        let (i, _) = char(label)(i)?;
        ucoord(i)
    }
}

pub fn upos<const AXES: usize>(
    coord_labels: [char; AXES],
) -> impl Fn(&[u8]) -> IResult<&[u8], UPos<AXES>> {
    move |mut i| {
        let mut res = Vec::<_, AXES>::new();
        let mut first = true;
        for c in coord_labels {
            if !first {
                (i, _) = multispace1(i)?;
            }
            let coord;
            (i, coord) = opt(labeled_ucoord(c)).parse(i)?;
            if coord.is_some() {
                first = false;
            }
            res.push(coord).unwrap();
        }
        Ok((i, UPos::from(res.into_array().unwrap())))
    }
}

pub fn non_empty_upos<const AXES: usize>(
    coord_labels: [char; AXES],
) -> impl Fn(&[u8]) -> IResult<&[u8], UPos<AXES>> {
    move |i| {
        let (i, pos) = upos(coord_labels)(i)?;
        if !pos.0.iter().any(Option::is_some) {
            return Err(nom::Err::Error(nom::error::make_error(
                i,
                ErrorKind::NonEmpty,
            )));
        }
        Ok((i, pos))
    }
}

pub fn g(code: &str) -> impl Fn(&[u8]) -> IResult<&[u8], ()> {
    move |i| {
        let (i, _) = char('G')(i)?;
        let (i, _) = tag(code)(i)?;
        Ok((i, ()))
    }
}

pub fn upos_g_command<const AXES: usize>(
    g_code: &str,
    coord_labels: [char; AXES],
    mk_command: impl Fn(UPos<AXES>) -> Command<AXES>,
) -> impl Fn(&[u8]) -> IResult<&[u8], Command<AXES>> {
    move |i| {
        let (i, _) = g(g_code)(i)?;
        let (i, _) = multispace1(i)?;
        let (i, pos) = upos(coord_labels)(i)?;
        Ok((i, mk_command(pos)))
    }
}

pub fn non_empty_upos_g_command<const AXES: usize>(
    g_code: &str,
    coord_labels: [char; AXES],
    mk_command: impl Fn(UPos<AXES>) -> Command<AXES>,
) -> impl Fn(&[u8]) -> IResult<&[u8], Command<AXES>> {
    move |i| {
        let (i, _) = g(g_code)(i)?;
        let (i, _) = multispace1(i)?;
        let (i, pos) = non_empty_upos(coord_labels)(i)?;
        Ok((i, mk_command(pos)))
    }
}

pub fn command<const AXES: usize>(
    coord_labels: [char; AXES],
) -> impl Fn(&[u8]) -> IResult<&[u8], Command<AXES>> {
    move |i| {
        alt((
            non_empty_upos_g_command("0", coord_labels, Command::RapidMove),
            non_empty_upos_g_command("1", coord_labels, Command::LinearMove),
            non_empty_upos_g_command("2", coord_labels, Command::LinearMove),
        ))
        .parse(i)
    }
}

#[cfg(test)]
mod tests {
    use fixed::FixedU32;

    use super::*;

    const XYZ: [char; 3] = ['X', 'Y', 'Z'];
    const XYZF: [char; 4] = ['X', 'Y', 'Z', 'F'];

    #[test]
    fn xyz_upos() {
        let (remaining, res) = upos(XYZ).parse(b"X90.6 Y13.8 Z22").unwrap();
        assert_eq!(remaining, b"");
        assert_eq!(
            res,
            UPos([
                Some(FixedU32::from_str("90.6").unwrap()),
                Some(FixedU32::from_str("13.8").unwrap()),
                Some(FixedU32::from_num(22)),
            ])
        )
    }

    #[test]
    fn z_upos() {
        let (remaining, res) = upos(XYZ).parse(b"Z22").unwrap();
        assert_eq!(remaining, b"");
        assert_eq!(res, UPos([None, None, Some(FixedU32::from_num(22)),]))
    }

    #[test]
    fn non_empty_upos_requires_non_empty_coords() {
        let result = non_empty_upos(XYZ).parse(b"");
        assert!(result.is_err());
    }

    #[test]
    fn g0() {
        let (remaining, res) = command(XYZ)(b"G0 X90.6 Y13.8 Z22.4").unwrap();
        assert_eq!(remaining, b"");
        assert_eq!(
            res,
            Command::RapidMove(UPos([
                Some(FixedU32::from_str("90.6").unwrap()),
                Some(FixedU32::from_str("13.8").unwrap()),
                Some(FixedU32::from_str("22.4").unwrap()),
            ]))
        )
    }

    #[test]
    fn g0_feedrate() {
        let (remaining, res) = command(XYZF)(b"G0 F1500").unwrap();
        assert_eq!(remaining, b"");
        assert_eq!(
            res,
            Command::RapidMove(UPos([None, None, None, Some(FixedU32::from_num(1500))]))
        );
    }
}
