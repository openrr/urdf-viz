use crate::errors::Error;
use crate::errors::Result;
use crate::mesh::load_mesh;
use kiss3d::scene::SceneNode;
use log::*;
use nalgebra as na;
use std::path::Path;

pub fn add_geometry(
    geometry: &urdf_rs::Geometry,
    opt_color: &Option<na::Point3<f32>>,
    base_dir: Option<&Path>,
    group: &mut SceneNode,
    use_texture: bool,
) -> Result<SceneNode> {
    match *geometry {
        urdf_rs::Geometry::Box { ref size } => {
            let mut cube = group.add_cube(size[0] as f32, size[1] as f32, size[2] as f32);
            if let Some(color) = *opt_color {
                cube.set_color(color[0], color[1], color[2]);
            }
            Ok(cube)
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            let mut base = group.add_group();
            let mut cylinder = base.add_cylinder(radius as f32, length as f32);
            cylinder.append_rotation(&na::UnitQuaternion::from_axis_angle(
                &na::Vector3::x_axis(),
                1.57,
            ));
            if let Some(color) = *opt_color {
                base.set_color(color[0], color[1], color[2]);
            }
            Ok(base)
        }
        urdf_rs::Geometry::Capsule { .. } => {
            todo!()
        }
        urdf_rs::Geometry::Sphere { radius } => {
            let mut sphere = group.add_sphere(radius as f32);
            if let Some(color) = *opt_color {
                sphere.set_color(color[0], color[1], color[2]);
            }
            Ok(sphere)
        }
        urdf_rs::Geometry::Mesh {
            ref filename,
            scale,
        } => {
            let scale = scale.unwrap_or(DEFAULT_MESH_SCALE);
            let replaced_filename = urdf_rs::utils::expand_package_path(filename, base_dir);
            let path = Path::new(&replaced_filename);
            if !path.exists() {
                return Err(Error::from(format!("{} not found", replaced_filename)));
            }
            let na_scale = na::Vector3::new(scale[0] as f32, scale[1] as f32, scale[2] as f32);
            if cfg!(feature = "assimp") {
                debug!("filename = {}", replaced_filename);
                load_mesh(path, na_scale, opt_color, group, use_texture)
            } else if path.extension() == Some(std::ffi::OsStr::new("obj")) {
                let mut base = group.add_obj(path, path, na_scale);
                if let Some(color) = *opt_color {
                    base.set_color(color[0], color[1], color[2]);
                }
                Ok(base)
            } else {
                error!(
                    "{:?} is not supported, because assimp feature is disabled",
                    path
                );
                let mut base = group.add_cube(0.05f32, 0.05, 0.05);
                if let Some(color) = *opt_color {
                    base.set_color(color[0], color[1], color[2]);
                }
                Ok(base)
            }
        }
    }
}

// Use material which is defined as root materials if found.
// Root material is used for PR2, but not documented.
pub fn rgba_from_visual(urdf_robot: &urdf_rs::Robot, visual: &urdf_rs::Visual) -> [f64; 4] {
    match urdf_robot
        .materials
        .iter()
        .find(|mat| {
            visual
                .material
                .as_ref()
                .map_or(false, |m| mat.name == m.name)
        })
        .cloned()
    {
        Some(ref material) => material
            .color
            .as_ref()
            .map(|color| color.rgba)
            .unwrap_or_default(),
        None => visual
            .material
            .as_ref()
            .and_then(|material| material.color.as_ref().map(|color| color.rgba))
            .unwrap_or_default(),
    }
}

// https://github.com/openrr/urdf-rs/pull/3/files#diff-0fb2eeea3273a4c9b3de69ee949567f546dc8c06b1e190336870d00b54ea0979L36-L38
const DEFAULT_MESH_SCALE: [f64; 3] = [1.0f64; 3];
