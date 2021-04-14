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

//! urdf-viz
//! ==================
//!
//! Visualize [URDF(Unified Robot Description Format)](http://wiki.ros.org/urdf) file.
//! `urdf-viz` is written by rust-lang.
//!

#[cfg(feature = "assimp")]
extern crate assimp_crate as assimp;

#[cfg(feature = "assimp")]
mod assimp_utils;

mod errors;
pub use errors::*;
mod web_server;
pub use web_server::*;
mod viewer;
pub use viewer::*;
mod mesh;
pub use mesh::*;
mod urdf;
pub use urdf::*;
