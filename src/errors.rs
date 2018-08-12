/*
   Copyright 2017 Takashi Ogura

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

use std::io;

#[derive(Debug, Fail)]
pub enum Error {
    #[fail(display = "Error: {:?}", error)]
    Other { error: String },
    #[fail(display = "IOError: {:?}", error)]
    IoError { error: io::Error },
}

pub type Result<T> = ::std::result::Result<T, Error>;

impl From<io::Error> for Error {
    fn from(error: io::Error) -> Error {
        Error::IoError { error }
    }
}

impl<'a> From<&'a str> for Error {
    fn from(error: &'a str) -> Error {
        Error::Other {
            error: error.to_owned(),
        }
    }
}

impl From<String> for Error {
    fn from(error: String) -> Error {
        Error::Other { error }
    }
}
