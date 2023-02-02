use super::method::Method;
use std::convert::TryFrom;
use std::error::Error;
// use std::fmt::Display;
// use std::fmt::Formatter;
// use std::fmt::Result as FmtResult;
use std::fmt::{Debug, Display, Formatter, Result as FmtResult};
use std::str;
use std::str::Utf8Error;

pub struct Request {
    path: String,
    query_string: Option<String>,
    // method: super::method::Method,
    method: Method,
}

impl TryFrom<&[u8]> for Request {
    // type Error = String;
    type Error = ParseError;

    // GET /search?name=abc&sort=1 HTTP/1.1
    fn try_from(buf: &[u8]) -> Result<Self, Self::Error> {
        // Method 1 of Error Handling.
        // match str::from_utf8(buf){
        //     Ok(request) => {},
        //     Err(_) => return Err(ParseError::InvalidEncoding),
        // }

        // Method 2 of Error Handling.
        // match str::from_utf8(buf).or(Err(ParseError::InvalidEncoding)){
        //     Ok(request) => {},
        //     Err(_) => return Err(e),
        // }

        // Method 3 of Error Handling.
        // let request = str::from_utf8(buf).or(Err(ParseError::InvalidEncoding))?;
        let request = str::from_utf8(buf)?;
        unimplemented!()
    }
}

fn get_next_word(request: &str) -> Option<(&str, &str)> {
    // Method 1: Using iterator.
    // let mut iter = request.chars();
    // loop {
    //     let item = iter.next();
    //     match item {
    //         Some(c) => {}
    //         None => break,
    //     }
    // }

    // Method 2: Using for loop.
    for c in request.chars() {
        if c == ' ' {
            // We add i+1 to skip the space which is only one byte.
            return Some((&request[..i], &request[i + 1..]));
        }
        None
    }
}

pub enum ParseError {
    InvalidRequest,
    InvalidEncoding,
    InvalidProtocol,
    InvalidMethod,
}

impl ParseError {
    fn message(&self) -> &str {
        match self {
            Self::InvalidRequest => "Invalid Request",
            Self::InvalidEncoding => "Invalid Encoding",
            Self::InvalidProtocol => "Invalid Protocol",
            Self::InvalidMethod => "Invalid Method",
        }
    }
}

impl From<Utf8Error> for ParseError {
    fn from(_: Utf8Error) -> Self {
        Self::InvalidEncoding
    }
}

impl Display for ParseError {
    fn fmt(&self, f: &mut Formatter) -> FmtResult {
        write!(f, "{}", self.message())
    }
}

impl Debug for ParseError {
    fn fmt(&self, f: &mut Formatter) -> FmtResult {
        write!(f, "{}", self.message())
    }
}

impl Error for ParseError {}
