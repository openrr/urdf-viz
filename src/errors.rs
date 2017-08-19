use std::error::Error as StdError;
use std::fmt;
use std::io;
use std::string;

#[derive(Debug)]
pub enum Error {
    Other(String),
    Io(io::Error),
}

pub type Result<T> = ::std::result::Result<T, Error>;


impl From<io::Error> for Error {
    fn from(err: io::Error) -> Error {
        Error::Io(err)
    }
}

impl From<string::FromUtf8Error> for Error {
    fn from(err: string::FromUtf8Error) -> Error {
        Error::Other(err.description().to_owned())
    }
}


impl<'a> From<&'a str> for Error {
    fn from(err: &'a str) -> Error {
        Error::Other(err.to_owned())
    }
}

impl From<String> for Error {
    fn from(err: String) -> Error {
        Error::Other(err)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Error::Other(ref err) => write!(f, "error : {}", err),
            Error::Io(ref err) => write!(f, "io error : {}", err),
        }
    }
}

impl StdError for Error {
    fn description(&self) -> &str {
        match *self {
            Error::Other(ref err) => &err,
            Error::Io(ref err) => err.description(),
        }
    }
}
