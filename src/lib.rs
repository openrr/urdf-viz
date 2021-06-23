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

//! Visualize [URDF(Unified Robot Description Format)](http://wiki.ros.org/urdf) file.
//! `urdf-viz` is written by rust-lang.

#![warn(missing_debug_implementations, rust_2018_idioms)]

#[cfg(feature = "assimp")]
extern crate assimp_crate as assimp;

#[cfg(feature = "assimp")]
mod assimp_utils;

pub mod app;
mod errors;
pub use errors::*;
mod handle;
pub use handle::*;
#[cfg(not(target_arch = "wasm32"))]
mod web_server;
#[cfg(not(target_arch = "wasm32"))]
pub use web_server::*;
mod viewer;
pub use viewer::*;
mod mesh;
pub use mesh::*;
mod urdf;
pub use urdf::*;
pub mod utils;

// re-export
#[doc(no_inline)]
pub use kiss3d::{self, event::*};
