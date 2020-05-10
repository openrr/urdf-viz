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

use kiss3d::resource::Mesh;
use log::*;
use nalgebra as na;
use std::cell::RefCell;
use std::rc::Rc;

const ASSIMP_DIFFUSE: &[u8] = b"$clr.diffuse\0";

type RefCellMesh = Rc<RefCell<Mesh>>;

pub fn assimp_material_texture(material: &assimp::Material) -> Option<String> {
    use std::os::raw::{c_float, c_uint};
    let mut path = assimp_sys::AiString::default();
    let texture_type = assimp_sys::AiTextureType::Diffuse;
    let mat = &(**material) as *const assimp_sys::AiMaterial;
    let mut mapping = assimp_sys::AiTextureMapping::UV;
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
            &mut path as *mut assimp_sys::AiString,
            &mut mapping as *const assimp_sys::AiTextureMapping,
            &mut uv_index as *mut c_uint,
            &mut blend as *mut c_float,
            &mut op as *mut assimp_sys::AiTextureOp,
            &mut map_mode as *mut assimp_sys::AiTextureMapMode,
            &mut flags as *mut c_uint,
        ) {
            assimp_sys::AiReturn::Success => Some(path.as_ref().to_owned()),
            _ => None,
        }
    }
}

pub fn assimp_material_color(
    material: &assimp::Material,
    color_type: &'static [u8],
) -> Option<na::Vector3<f32>> {
    let mut assimp_color = assimp_sys::AiColor4D {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 0.0,
    };
    let mat = &(**material) as *const assimp_sys::AiMaterial;
    unsafe {
        match assimp_sys::aiGetMaterialColor(
            mat,
            color_type.as_ptr() as *const i8,
            0,
            0,
            &mut assimp_color,
        ) {
            assimp_sys::AiReturn::Success => Some(na::Vector3::<f32>::new(
                assimp_color.r,
                assimp_color.g,
                assimp_color.b,
            )),
            _ => None,
        }
    }
}

pub fn convert_assimp_scene_to_kiss3d_mesh(
    scene: &assimp::Scene,
) -> (Vec<RefCellMesh>, Vec<String>, Vec<na::Vector3<f32>>) {
    let meshes = scene
        .mesh_iter()
        .map(|mesh| {
            let mut vertices = Vec::new();
            let mut indices = Vec::new();
            vertices.extend(mesh.vertex_iter().map(|v| na::Point3::new(v.x, v.y, v.z)));
            indices.extend(mesh.face_iter().filter_map(|f| {
                if f.num_indices == 3 {
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
