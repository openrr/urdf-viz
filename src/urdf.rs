use crate::errors::{Error, Result};
use crate::mesh::load_mesh;
use crate::utils::is_url;
use k::nalgebra as na;
use kiss3d::scene::SceneNode;
use std::borrow::Cow;
use std::collections::HashMap;
use std::path::Path;
use tracing::*;

pub fn add_geometry(
    geometry: &urdf_rs::Geometry,
    opt_color: &Option<na::Point3<f32>>,
    base_dir: Option<&Path>,
    group: &mut SceneNode,
    use_texture: bool,
    use_assimp: bool,
    package_path: &HashMap<String, String>,
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
        urdf_rs::Geometry::Capsule { radius, length } => {
            let mut base = group.add_group();
            let mut cylinder = base.add_cylinder(radius as f32, length as f32);
            cylinder.append_rotation(&na::UnitQuaternion::from_axis_angle(
                &na::Vector3::x_axis(),
                1.57,
            ));
            let mut sphere1 = base.add_sphere(radius as f32);
            sphere1.append_translation(&na::Translation3::new(0.0, 0.0, length as f32 * 0.5));
            let mut sphere2 = base.add_sphere(radius as f32);
            sphere2.append_translation(&na::Translation3::new(0.0, 0.0, length as f32 * -0.5));

            if let Some(color) = *opt_color {
                cylinder.set_color(color[0], color[1], color[2]);
                sphere1.set_color(color[0], color[1], color[2]);
                sphere2.set_color(color[0], color[1], color[2]);
            }
            Ok(base)
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
            let mut filename = Cow::Borrowed(&**filename);
            if !cfg!(target_family = "wasm") {
                // On WASM, this is handled in utils::load_mesh
                if filename.starts_with("package://") {
                    if let Some(replaced_filename) =
                        crate::utils::replace_package_with_path(&filename, package_path)
                    {
                        filename = replaced_filename.into();
                    }
                };
                let replaced_filename = urdf_rs::utils::expand_package_path(&filename, base_dir)?;
                if !is_url(&replaced_filename) && !Path::new(&*replaced_filename).exists() {
                    return Err(Error::from(format!("{replaced_filename} not found")));
                }
                filename = replaced_filename.into_owned().into();
            }
            let na_scale = na::Vector3::new(scale[0] as f32, scale[1] as f32, scale[2] as f32);
            debug!("filename = {filename}");
            if cfg!(feature = "assimp") {
                load_mesh(
                    &filename,
                    na_scale,
                    opt_color,
                    group,
                    use_texture,
                    use_assimp,
                )
            } else {
                match load_mesh(
                    &filename,
                    na_scale,
                    opt_color,
                    group,
                    use_texture,
                    use_assimp,
                ) {
                    Ok(scene) => Ok(scene),
                    Err(e) => {
                        error!("{e}");
                        let mut base = group.add_cube(0.05f32, 0.05, 0.05);
                        if let Some(color) = *opt_color {
                            base.set_color(color[0], color[1], color[2]);
                        }
                        Ok(base)
                    }
                }
            }
        }
    }
}

// Use material which is defined as root materials if found.
// Root material is used for PR2, but not documented.
pub fn rgba_from_visual(urdf_robot: &urdf_rs::Robot, visual: &urdf_rs::Visual) -> urdf_rs::Vec4 {
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
const DEFAULT_MESH_SCALE: urdf_rs::Vec3 = urdf_rs::Vec3([1.0f64; 3]);
