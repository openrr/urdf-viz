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

use k::nalgebra as na;
use kiss3d::resource::Mesh;
use std::cell::RefCell;
use std::ffi::CStr;
use std::os::raw::{c_float, c_uint};
use std::rc::Rc;
use std::str;
use tracing::*;

const ASSIMP_DIFFUSE: &CStr = c"$clr.diffuse";

type RefCellMesh = Rc<RefCell<Mesh>>;

fn assimp_material_texture(material: &assimp::Material<'_>) -> Option<String> {
    let mut path = assimp_sys::AiString::default();
    let texture_type = assimp_sys::AiTextureType::Diffuse;
    let mat = &**material;
    let mapping = assimp_sys::AiTextureMapping::UV;
    let mut uv_index: c_uint = 0;
    let mut blend: c_float = 0.0;
    let mut op = assimp_sys::AiTextureOp::Multiply;
    let mut map_mode = assimp_sys::AiTextureMapMode::Wrap;
    let mut flags: c_uint = 0;
    unsafe {
        match assimp_sys::aiGetMaterialTexture(
            mat,
            texture_type,
            0,
            &mut path,
            &mapping,
            &mut uv_index,
            &mut blend,
            &mut op,
            &mut map_mode,
            &mut flags,
        ) {
            assimp_sys::AiReturn::Success => Some(
                // assimp-rs's impl AsRef<str> for AiString does a similar, but it
                // might panic because assimp-rs doesn't handle the case where assimp
                // returns an out-of-range length (which probably means an error).
                str::from_utf8(path.data.get(0..path.length)?)
                    .ok()?
                    .to_owned(),
            ),
            _ => None,
        }
    }
}

fn assimp_material_color(
    material: &assimp::Material<'_>,
    color_type: &'static CStr,
) -> Option<na::Vector3<f32>> {
    let mut assimp_color = assimp_sys::AiColor4D {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 0.0,
    };
    let mat = &**material;
    unsafe {
        match assimp_sys::aiGetMaterialColor(mat, color_type.as_ptr(), 0, 0, &mut assimp_color) {
            assimp_sys::AiReturn::Success => Some(na::Vector3::<f32>::new(
                assimp_color.r,
                assimp_color.g,
                assimp_color.b,
            )),
            _ => None,
        }
    }
}

pub(crate) fn convert_assimp_scene_to_kiss3d_mesh(
    scene: &assimp::Scene<'_>,
) -> (Vec<RefCellMesh>, Vec<String>, Vec<na::Vector3<f32>>) {
    let meshes = scene
        .mesh_iter()
        .map(|mesh| {
            let mut vertices = Vec::new();
            let mut indices = Vec::new();
            vertices.extend(mesh.vertex_iter().map(|v| na::Point3::new(v.x, v.y, v.z)));
            indices.extend(mesh.face_iter().filter_map(|f| {
                if f.num_indices == 3 {
                    // TODO: https://github.com/openrr/urdf-viz/issues/22
                    Some(na::Point3::new(f[0] as u16, f[1] as u16, f[2] as u16))
                } else {
                    debug!("invalid mesh!");
                    None
                }
            }));
            Rc::new(RefCell::new(Mesh::new(
                vertices, indices, None, None, false,
            )))
        })
        .collect();
    let colors = scene
        .material_iter()
        .filter_map(|material| assimp_material_color(&material, ASSIMP_DIFFUSE))
        .collect();
    let texture_files = scene
        .material_iter()
        .filter_map(|material| assimp_material_texture(&material))
        .collect();
    (meshes, texture_files, colors)
}
