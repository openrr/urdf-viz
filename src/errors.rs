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
use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("Error: {:?}", .0)]
    Other(String),
    #[error("IOError: {:?}", .0)]
    IoError(#[from] io::Error),
    #[error("FromUtf8Error: {:?}", .0)]
    FromUtf8Error(#[from] std::string::FromUtf8Error),
    #[error("UrdfError: {:?}", .0)]
    Urdf(#[from] urdf_rs::UrdfError),
}

pub type Result<T> = ::std::result::Result<T, Error>;

impl<'a> From<&'a str> for Error {
    fn from(error: &'a str) -> Self {
        Error::Other(error.to_owned())
    }
}

impl From<String> for Error {
    fn from(error: String) -> Self {
        Error::Other(error)
    }
}

#[cfg(target_arch = "wasm32")]
impl From<wasm_bindgen::JsValue> for Error {
    fn from(error: wasm_bindgen::JsValue) -> Self {
        Error::Other(format!("{error:?}"))
    }
}

#[cfg(target_arch = "wasm32")]
impl From<Error> for wasm_bindgen::JsValue {
    fn from(error: Error) -> Self {
        Self::from_str(&error.to_string())
    }
}
