#![allow(dead_code)] // FIXME

use core::time::Duration;

use heapless::Vec;
use nom::{
    AsChar, IResult, Parser,
    branch::alt,
    bytes::complete::{tag, take_till, take_while1},
    character::complete::{char, multispace1},
    combinator::{map, map_res, opt, value},
    error::ErrorKind,
    multi::fold_many1,
    number::complete::recognize_float,
    sequence::preceded,
};

use crate::ast::{Command, UCoord, UPos};

pub fn comment(i: &[u8]) -> IResult<&[u8], ()> {
    let (mut i, _) = char('(')(i)?;
    while !(i.is_empty() || i.first().is_some_and(|x| *x == b')')) {
        (i, _) = take_till(|c| [b'(', b')'].contains(&c))(i)?;
        (i, _) = opt(comment).parse(i)?;
    }
    let (i, _) = char(')')(i)?;
    Ok((i, ()))
}

pub fn whitespace1(i: &[u8]) -> IResult<&[u8], ()> {
    fold_many1(alt((comment, multispace1.map(|_| ()))), || (), |(), _| ()).parse(i)
}

pub fn whitespace0(i: &[u8]) -> IResult<&[u8], ()> {
    opt(whitespace1).map(|_| ()).parse(i)
}

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
                fold_many1(
                    alt((char(' ').map(|_| ()), char('\t').map(|_| ()), comment)),
                    || (),
                    |(), _| (),
                ),
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
        let (i, _) = whitespace1(i)?;
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

pub fn home<const AXES: usize>(i: &[u8]) -> IResult<&[u8], Command<AXES>> {
    let (i, _) = g("28")(i)?;
    let (i, f) = opt(preceded(whitespace1, labeled_ucoord('F'))).parse(i)?;
    Ok((i, Command::Home { f }))
}

pub fn dwell<const AXES: usize>(i: &[u8]) -> IResult<&[u8], Command<AXES>> {
    let (i, _) = g("4")(i)?;
    let (i, _) = whitespace1(i)?;
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
        preceded(
            whitespace0,
            alt((
                non_empty_upos_g_command("0", coord_labels, Command::RapidMove),
                non_empty_upos_g_command("1", coord_labels, Command::LinearMove),
                dwell,
                value(Command::Stop, m("0")),
                value(Command::EnableAllSteppers, m("17")),
                value(Command::DisableAllSteppers, m("18")),
                home,
                value(Command::ForceStop, m("112")),
                value(Command::GetCurrentPosition, m("114")),
                value(Command::Pause, m("226")),
            )),
        )
        .parse(i)
    }
}

#[cfg(test)]
mod tests {
    use fixed::FixedU32;

    use super::*;

    const XYZ: [char; 3] = ['X', 'Y', 'Z'];
    const XYZF: [char; 4] = ['X', 'Y', 'Z', 'F'];
    const XZCF: [char; 4] = ['X', 'Z', 'C', 'F'];

    fn round_trip<const N: usize>(cmd: Command<N>, axis_labels: [char; N]) {
        let formatted = cmd.display(axis_labels).to_string();
        let (rem, parsed) = command(axis_labels)(formatted.as_bytes()).unwrap();
        assert_eq!(
            rem,
            b"",
            "unparsed remainder: {:?}",
            core::str::from_utf8(rem)
        );
        assert_eq!(parsed, cmd, "round-trip failed for: {formatted}");
    }

    macro_rules! test_parse {
        ($axis_labels:expr, $input:expr, $expected:expr) => {
            let axis_labels = $axis_labels;
            let (rem, res) = command(axis_labels)($input).unwrap();
            let expected = $expected;
            assert_eq!(
                rem,
                b"",
                "unparsed remainder: {:?}",
                core::str::from_utf8(rem)
            );
            assert_eq!(res, expected);
            round_trip(expected, axis_labels);
        };
    }

    #[test]
    fn non_empty_upos_requires_non_empty_coords() {
        let result = non_empty_upos(XYZ).parse(b"");
        assert!(result.is_err());
    }

    #[test]
    fn g0() {
        test_parse!(
            XYZ,
            b"G0 X90.6 Y13.8 Z22.4",
            Command::RapidMove(UPos([
                Some(FixedU32::from_str("90.6").unwrap()),
                Some(FixedU32::from_str("13.8").unwrap()),
                Some(FixedU32::from_str("22.4").unwrap()),
            ]))
        );
    }

    #[test]
    fn g0_incomplete() {
        test_parse!(
            XYZ,
            b"G0 X90.6",
            Command::RapidMove(UPos([
                Some(FixedU32::from_str("90.6").unwrap()),
                None,
                None,
            ]))
        );
    }

    #[test]
    fn g0_feedrate() {
        test_parse!(
            XYZF,
            b"G0 F1500",
            Command::RapidMove(UPos([None, None, None, Some(FixedU32::from_num(1500))]))
        );
    }

    #[test]
    fn zc_axis() {
        test_parse!(
            XZCF,
            b"G0 Z40 C10 F40",
            Command::RapidMove(UPos([
                None,
                Some(FixedU32::lit("40")),
                Some(FixedU32::lit("10")),
                Some(FixedU32::lit("40"))
            ]))
        );
    }

    #[test]
    fn g4_secs() {
        test_parse!(XYZF, b"G4 S4", Command::Dwell(Duration::from_secs(4)));
    }

    #[test]
    fn g4_millis() {
        test_parse!(XYZF, b"G4 P123", Command::Dwell(Duration::from_millis(123)));
    }

    #[test]
    fn m0_stop() {
        test_parse!(XYZF, b"M0", Command::Stop);
    }

    #[test]
    fn m17_enable_all_steppers() {
        test_parse!(XYZF, b"M17", Command::EnableAllSteppers);
    }

    #[test]
    fn m18_disable_all_steppers() {
        test_parse!(XYZF, b"M18", Command::DisableAllSteppers);
    }

    #[test]
    fn m25_pause() {
        test_parse!(XYZF, b"M226", Command::Pause);
        test_parse!(XYZ, b"M226", Command::Pause);
    }

    #[test]
    fn g28_home() {
        test_parse!(XYZF, b"G28", Command::Home { f: None });
    }

    #[test]
    fn g28_home_trailing_newline() {
        // Tests to make sure we're not accidentally using the streaming parsers
        let (rem, res) = command(XYZF)(b"G28\n").unwrap();
        assert_eq!(res, Command::Home { f: None });
        assert_eq!(rem, b"\n");
    }

    #[test]
    fn g28_home_with_feedrate() {
        test_parse!(
            XYZF,
            b"G28 F123",
            Command::Home {
                f: Some("123".parse().unwrap())
            }
        );
    }

    #[test]
    fn g28_home_with_feedrate_trailing_newline() {
        let (rem, res) = command(XYZF)(b"G28 F123\n").unwrap();
        assert_eq!(
            res,
            Command::Home {
                f: Some("123".parse().unwrap())
            }
        );
        assert_eq!(rem, b"\n");
    }

    #[test]
    fn home_with_feedrate() {
        test_parse!(
            XYZF,
            b"G28 F40",
            Command::Home {
                f: Some(FixedU32::lit("40"))
            }
        );
    }

    #[test]
    fn comment_whole_line() {
        test_parse!(
            XYZ,
            b"(hi, this is a comment!)\nG0 X1 Y1",
            Command::RapidMove(UPos([
                Some("1".parse().unwrap()),
                Some("1".parse().unwrap()),
                None,
            ]))
        );
    }

    #[test]
    fn comment_inside_command() {
        test_parse!(
            XYZ,
            b"G0 X1 Y1 (z should become two!) Z2",
            Command::RapidMove(UPos([
                Some("1".parse().unwrap()),
                Some("1".parse().unwrap()),
                Some("2".parse().unwrap())
            ]))
        );
    }

    #[test]
    fn nested_comment_inside_command() {
        let _ = comment.parse(b"(z should (not) become seven!)").unwrap();
        let _ = preceded(
            fold_many1(
                alt((comment, char(' ').map(|_| ()), char('\t').map(|_| ()))),
                || (),
                |(), _| (),
            ),
            |i| {
                eprintln!("{}", str::from_utf8(i).unwrap());
                labeled_ucoord('Z').parse(i)
            },
        )
        .parse(b" (z should (not) become seven!) Z2")
        .unwrap();
        test_parse!(
            XYZ,
            b"G0 X1 Y1 (z should (not) become seven!) Z2",
            Command::RapidMove(UPos([
                Some("1".parse().unwrap()),
                Some("1".parse().unwrap()),
                Some("2".parse().unwrap())
            ]))
        );
    }

    #[test]
    fn m112_force_stop() {
        test_parse!(XYZF, b"M112", Command::ForceStop);
    }
}
