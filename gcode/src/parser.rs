#![allow(dead_code)] // FIXME

use core::time::Duration;

use heapless::Vec;
use nom::{
    branch::alt,
    bytes::{complete::take_while1, streaming::tag},
    character::{complete::multispace1, streaming::char},
    combinator::{map, map_res, opt, value},
    error::ErrorKind,
    number::complete::recognize_float,
    sequence::preceded,
    AsChar, IResult, Parser,
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
        for c in coord_labels {
            let coord;
            (i, coord) = opt(preceded(
                take_while1(|c| c == b' ' || c == b'\t'),
                labeled_ucoord(c),
            ))
            .parse(i)?;
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

pub fn m(code: &str) -> impl Fn(&[u8]) -> IResult<&[u8], ()> {
    move |i| {
        let (i, _) = char('M')(i)?;
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
        let (i, pos) = non_empty_upos(coord_labels)(i)?;
        Ok((i, mk_command(pos)))
    }
}

pub fn dwell<const AXES: usize>(i: &[u8]) -> IResult<&[u8], Command<AXES>> {
    let (i, _) = g("4")(i)?;
    let (i, _) = multispace1(i)?;
    let (i, dur) = alt((
        preceded(
            char('S'),
            map(
                map_res(take_while1(AsChar::is_dec_digit), u64::from_ascii),
                Duration::from_secs,
            ),
        ),
        preceded(
            char('P'),
            map(
                map_res(take_while1(AsChar::is_dec_digit), u64::from_ascii),
                Duration::from_millis,
            ),
        ),
    ))
    .parse(i)?;
    Ok((i, Command::Dwell(dur)))
}

pub fn command<const AXES: usize>(
    coord_labels: [char; AXES],
) -> impl Fn(&[u8]) -> IResult<&[u8], Command<AXES>> {
    move |i| {
        alt((
            non_empty_upos_g_command("0", coord_labels, Command::RapidMove),
            non_empty_upos_g_command("1", coord_labels, Command::LinearMove),
            dwell,
            value(Command::Stop, m("0")),
            value(Command::EnableAllSteppers, m("17")),
            value(Command::DisableAllSteppers, m("18")),
            value(Command::Home, g("28")),
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
    fn g0_incomplete() {
        let (remaining, res) = command(XYZ)(b"G0 X90.6").unwrap();
        assert_eq!(remaining, b"");
        assert_eq!(
            res,
            Command::RapidMove(UPos([
                Some(FixedU32::from_str("90.6").unwrap()),
                None,
                None,
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

    #[test]
    fn g4_secs() {
        let (rem, res) = command(XYZF)(b"G4 S4").unwrap();
        assert_eq!(rem, b"");
        assert_eq!(res, Command::Dwell(Duration::from_secs(4)));
    }

    #[test]
    fn g4_millis() {
        let (rem, res) = command(XYZF)(b"G4 P123").unwrap();
        assert_eq!(rem, b"");
        assert_eq!(res, Command::Dwell(Duration::from_millis(123)));
    }

    #[test]
    fn m0_stop() {
        let (rem, res) = command(XYZF)(b"M0").unwrap();
        assert_eq!(rem, b"");
        assert_eq!(res, Command::Stop);
    }

    #[test]
    fn m17_enable_all_steppers() {
        let (rem, res) = command(XYZF)(b"M17").unwrap();
        assert_eq!(rem, b"");
        assert_eq!(res, Command::EnableAllSteppers);
    }

    #[test]
    fn m18_disable_all_steppers() {
        let (rem, res) = command(XYZF)(b"M18").unwrap();
        assert_eq!(rem, b"");
        assert_eq!(res, Command::DisableAllSteppers);
    }

    #[test]
    fn g28_home() {
        let (rem, res) = command(XYZF)(b"G28").unwrap();
        assert_eq!(rem, b"");
        assert_eq!(res, Command::Home);
    }
}
