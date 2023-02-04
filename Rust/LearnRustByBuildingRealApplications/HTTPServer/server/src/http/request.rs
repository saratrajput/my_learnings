use super::method::{Method, MethodError};
use std::convert::TryFrom;
use std::error::Error;
// use std::fmt::Display;
// use std::fmt::Formatter;
// use std::fmt::Result as FmtResult;
use std::fmt::{Debug, Display, Formatter, Result as FmtResult};
use std::str;
use std::str::Utf8Error;

pub struct Request {
    // path: String,
    // query_string: Option<String>,
    path: &str,
    query_string: Option<&str>,
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

        // Method 1 of getting next word.
        // match get_next_word(request) {
        //     Some((method, request)) => {}
        //     None => return Err(ParseError::InvalidRequest),
        // }

        // Method 2 of using .ok_or()
        let (method, request) = get_next_word(request).ok_or(ParseError::InvalidRequest)?;

        // Get the rest of the request.
        let (mut path, request) = get_next_word(request).ok_or(ParseError::InvalidRequest)?;
        let (protocol, _) = get_next_word(request).ok_or(ParseError::InvalidRequest)?;

        if protocol != "HTTP/1.1" {
            return Err(ParseError::InvalidProtocol);
        }

        let method: Method = method.parse()?;

        let mut query_string = None;

        // Method 1 to get the path from /search?name=abc&sort=1
        // match path.find('?') {
        //     Some(i) => {
        //         query_string = Some(&path[i + 1..]);
        //         path = &path[..i];
        //     }
        //     None => {}
        // }

        // Method 2 to avoid using None => {} in Method 1.
        // let q = path.find('?');
        // if q.is_some() {
        //     let i = q.unwrap();
        //     query_string = Some(&path[i + 1..]);
        //     path = &path[..i];
        // }

        // Method 3 using the if-let expression.
        if let Some(i) = path.find('?') {
            // query_string = Some(&path[i + 1..].to_string());
            query_string = Some(&path[i + 1..]);
            path = &path[..i];
        }

        // unimplemented!()
        Ok(Self {
            // path.to_string(),
            path,
            query_string,
            method,
        })
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
    for (i, c) in request.chars().enumerate() {
        if c == ' ' || c == '\r' {
            // We add i+1 to skip the space which is only one byte.
            return Some((&request[..i], &request[i + 1..]));
        }
    }
    None
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

impl From<MethodError> for ParseError {
    fn from(_: MethodError) -> Self {
        Self::InvalidMethod
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
