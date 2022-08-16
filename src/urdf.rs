use crate::errors::{Error, Result};
use crate::mesh::load_mesh;
use k::nalgebra as na;
use kiss3d::scene::SceneNode;
use std::borrow::Cow;
use std::path::Path;
use tracing::*;

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
            let replaced_filename = if cfg!(target_family = "wasm") {
                Cow::Borrowed(filename)
            } else {
                let replaced_filename = urdf_rs::utils::expand_package_path(filename, base_dir);
                if !replaced_filename.starts_with("https://")
                    && !replaced_filename.starts_with("http://")
                    && !Path::new(&replaced_filename).exists()
                {
                    return Err(Error::from(format!("{replaced_filename} not found")));
                }
                // TODO: remove Cow::Owned once https://github.com/openrr/urdf-rs/pull/41
                // is released in the next breaking release of urdf-rs.
                Cow::Owned(replaced_filename)
            };
            let na_scale = na::Vector3::new(scale[0] as f32, scale[1] as f32, scale[2] as f32);
            debug!("filename = {replaced_filename}");
            if cfg!(feature = "assimp") {
                load_mesh(
                    replaced_filename.as_str(),
                    na_scale,
                    opt_color,
                    group,
                    use_texture,
                )
            } else {
                match load_mesh(
                    replaced_filename.as_str(),
                    na_scale,
                    opt_color,
                    group,
                    use_texture,
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
