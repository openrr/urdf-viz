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
extern crate assimp;
#[cfg(feature = "assimp")]
extern crate assimp_sys;
#[cfg(feature = "assimp")]
mod assimp_utils;

extern crate alga;
#[macro_use]
extern crate failure;
extern crate glfw;
extern crate k;
extern crate kiss3d;
#[macro_use]
extern crate log;
extern crate nalgebra as na;
#[macro_use]
extern crate rouille;
#[macro_use]
extern crate serde_derive;
extern crate urdf_rs;

use alga::general::SubsetOf;
use kiss3d::scene::SceneNode;
use std::collections::HashMap;
use std::path::Path;
use std::rc::Rc;

mod errors;
pub use errors::*;
mod arc_ball;
pub use arc_ball::*;
mod web_server;
use assimp_utils::*;
pub use web_server::JointNamesAndPositions;
pub use web_server::WebServer;

use na::Real;

#[cfg(feature = "assimp")]
pub fn load_mesh<P>(
    filename: P,
    scale: na::Vector3<f32>,
    opt_urdf_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    use_texture: bool,
) -> Result<SceneNode>
where
    P: AsRef<Path>,
{
    let mut base = group.add_group();
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    let file_string = filename
        .as_ref()
        .to_str()
        .ok_or("failed to convert file string")?;
    let (meshes, textures, colors) =
        convert_assimp_scene_to_kiss3d_mesh(&importer.read_file(file_string)?);
    info!(
        "num mesh, texture, colors = {} {} {}",
        meshes.len(),
        textures.len(),
        colors.len()
    );
    let mesh_scenes = meshes
        .into_iter()
        .map(|mesh| {
            let mut scene = base.add_mesh(mesh, scale);
            // use urdf color as default
            if let Some(urdf_color) = *opt_urdf_color {
                scene.set_color(urdf_color[0], urdf_color[1], urdf_color[2]);
            }
            scene
        })
        .collect::<Vec<_>>();
    // use texture only for dae (collada)
    let mut is_collada = false;
    if let Some(ext) = filename.as_ref().extension() {
        if ext == "dae" || ext == "DAE" {
            is_collada = true;
        }
    }
    // do not use texture, use only color in urdf file.
    if !use_texture || !is_collada {
        return Ok(base);
    }

    // Size of color and mesh are same, use each color for mesh
    if mesh_scenes.len() == colors.len() {
        let mut count = 0;
        for (mut mesh_scene, color) in mesh_scenes.into_iter().zip(colors.into_iter()) {
            mesh_scene.set_color(color[0], color[1], color[2]);
            // Is this OK?
            if count < textures.len() {
                let mut texture_path = filename.as_ref().to_path_buf();
                texture_path.set_file_name(textures[count].clone());
                debug!("using texture={}", texture_path.display());
                if texture_path.exists() {
                    mesh_scene.set_texture_from_file(&texture_path, texture_path.to_str().unwrap());
                }
            }
            count += 1;
        }
    } else {
        // When size of mesh and color mismatch, use only first color/texture for all meshes.
        // If no color found, use urdf color instead.
        for mut mesh_scene in mesh_scenes {
            if !textures.is_empty() {
                let mut texture_path = filename.as_ref().to_path_buf();
                texture_path.set_file_name(textures[0].clone());
                debug!("texture={}", texture_path.display());
                if texture_path.exists() {
                    mesh_scene.set_texture_from_file(&texture_path, texture_path.to_str().unwrap());
                }
            }
            if !colors.is_empty() {
                let color = colors[0];
                mesh_scene.set_color(color[0], color[1], color[2]);
            }
        }
    }
    Ok(base)
}

#[cfg(not(feature = "assimp"))]
pub fn load_mesh<P>(
    filename: P,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    use_texture: bool,
) -> Result<SceneNode>
where
    P: AsRef<Path>,
{
    Err(Error::from("load mesh is disabled by feature"))
}

fn add_geometry(
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
fn rgba_from_visual(urdf_robot: &urdf_rs::Robot, visual: &urdf_rs::Visual) -> [f64; 4] {
    match urdf_robot
        .materials
        .iter()
        .find(|mat| mat.name == visual.material.name)
        .cloned()
    {
        Some(ref material) => material.color.rgba,
        None => visual.material.color.rgba,
    }
}

pub struct Viewer {
    pub window: kiss3d::window::Window,
    scenes: HashMap<String, SceneNode>,
    arc_ball: ArcBall,
    font_map: HashMap<i32, Rc<kiss3d::text::Font>>,
    font_data: &'static [u8],
    original_colors: HashMap<String, Vec<na::Point3<f32>>>,
    is_texture_enabled: bool,
    link_joint_map: HashMap<String, String>,
}

impl Viewer {
    pub fn new(title: &str) -> Viewer {
        let eye = na::Point3::new(3.0f32, 1.0, 1.0);
        let at = na::Point3::new(0.0f32, 0.0, 0.25);
        let mut window = kiss3d::window::Window::new_with_size(title, 1400, 1000);
        window.set_light(kiss3d::light::Light::StickToCamera);
        window.set_background_color(0.0, 0.0, 0.3);

        Viewer {
            window,
            scenes: HashMap::new(),
            arc_ball: ArcBall::new(eye, at),
            font_map: HashMap::new(),
            font_data: include_bytes!("font/Inconsolata.otf"),
            original_colors: HashMap::new(),
            is_texture_enabled: true,
            link_joint_map: HashMap::new(),
        }
    }
    pub fn disable_texture(&mut self) {
        self.is_texture_enabled = false;
    }
    pub fn enable_texture(&mut self) {
        self.is_texture_enabled = true;
    }

    pub fn add_robot(&mut self, urdf_robot: &urdf_rs::Robot) {
        self.add_robot_with_base_dir(urdf_robot, None);
    }
    pub fn add_robot_with_base_dir(
        &mut self,
        urdf_robot: &urdf_rs::Robot,
        base_dir: Option<&Path>,
    ) {
        self.add_robot_with_base_dir_and_collision_flag(urdf_robot, base_dir, false);
    }
    pub fn add_robot_with_base_dir_and_collision_flag(
        &mut self,
        urdf_robot: &urdf_rs::Robot,
        base_dir: Option<&Path>,
        is_collision: bool,
    ) {
        self.link_joint_map = k::urdf::link_to_joint_map(&urdf_robot);

        for l in &urdf_robot.links {
            let num = if is_collision {
                l.collision.len()
            } else {
                l.visual.len()
            };
            if num == 0 {
                continue;
            }
            let mut scene_group = self.window.add_group();
            let mut colors = Vec::new();
            for i in 0..num {
                let (geom_element, origin_element) = if is_collision {
                    (&l.collision[i].geometry, &l.collision[i].origin)
                } else {
                    (&l.visual[i].geometry, &l.visual[i].origin)
                };
                let mut opt_color = None;
                if l.visual.len() > i {
                    let rgba = rgba_from_visual(urdf_robot, &l.visual[i]);
                    let color = na::Point3::new(rgba[0] as f32, rgba[1] as f32, rgba[2] as f32);
                    if color[0] > 0.001 || color[1] > 0.001 || color[2] > 0.001 {
                        opt_color = Some(color);
                    }
                    colors.push(color);
                }
                if let Ok(mut base_group) = add_geometry(
                    geom_element,
                    &opt_color,
                    base_dir,
                    &mut scene_group,
                    self.is_texture_enabled,
                ) {
                    // set initial origin offset
                    base_group.set_local_transformation(k::urdf::isometry_from(&origin_element));
                } else {
                    error!("failed to create for {:?}", l);
                }
            }
            let joint_name = self.link_joint_map.get(&l.name).expect(&format!("joint for link '{}' not found", l.name));
            self.scenes
                .insert(joint_name.to_owned(), scene_group);
            self.original_colors.insert(joint_name.to_owned(), colors);
        }
    }
    pub fn remove_robot(&mut self, urdf_robot: &urdf_rs::Robot) {
        for l in &urdf_robot.joints {
            if let Some(mut scene) = self.scenes.get_mut(&l.name) {
                self.window.remove(&mut scene);
            }
        }
    }
    pub fn add_axis_cylinders(&mut self, name: &str, size: f32) {
        let mut axis_group = self.window.add_group();
        let mut x = axis_group.add_cylinder(0.01, size);
        x.set_color(0.0, 0.0, 1.0);
        let mut y = axis_group.add_cylinder(0.01, size);
        y.set_color(0.0, 1.0, 0.0);
        let mut z = axis_group.add_cylinder(0.01, size);
        z.set_color(1.0, 0.0, 0.0);
        let rot_x = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), 1.57);
        let rot_y = na::UnitQuaternion::from_axis_angle(&na::Vector3::y_axis(), 1.57);
        let rot_z = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), 1.57);
        x.append_translation(&na::Translation3::new(0.0, 0.0, size * 0.5));
        y.append_translation(&na::Translation3::new(0.0, size * 0.5, 0.0));
        z.append_translation(&na::Translation3::new(size * 0.5, 0.0, 0.0));
        x.set_local_rotation(rot_x);
        y.set_local_rotation(rot_y);
        z.set_local_rotation(rot_z);
        self.scenes.insert(name.to_owned(), axis_group);
    }
    pub fn scene_node(&mut self, name: &str) -> Option<&SceneNode> {
        self.scenes.get(name)
    }
    pub fn scene_node_mut(&mut self, name: &str) -> Option<&mut SceneNode> {
        self.scenes.get_mut(name)
    }
    pub fn render(&mut self) -> bool {
        self.window.render_with_camera(&mut self.arc_ball)
    }
    pub fn update<T>(&mut self, robot: &k::Chain<T>)
    where
        T: Real + alga::general::SubsetOf<f32>,
    {
        robot.update_transforms();
        for link in robot.iter() {
            let trans = link.world_transform().unwrap();
            let link_name = &link.joint().name;
            let trans_f32: na::Isometry3<f32> = na::Isometry3::to_superset(&trans);
            match self.scenes.get_mut(link_name) {
                Some(obj) => {
                    obj.set_local_transformation(trans_f32);
                }
                None => {
                    debug!("{} not found", link_name);
                }
            }
        }
    }
    pub fn draw_text(
        &mut self,
        text: &str,
        size: i32,
        pos: &na::Point2<f32>,
        color: &na::Point3<f32>,
    ) {
        self.window.draw_text(
            text,
            pos,
            self.font_map
                .entry(size)
                .or_insert(kiss3d::text::Font::from_memory(self.font_data, size)),
            color,
        );
    }
    pub fn events(&self) -> kiss3d::window::EventManager {
        self.window.events()
    }
    pub fn set_temporal_color(&mut self, link_name: &str, r: f32, g: f32, b: f32) {
        if let Some(obj) = self.scenes.get_mut(link_name) {
            obj.set_color(r, g, b);
        }
    }
    pub fn reset_temporal_color(&mut self, link_name: &str) {
        if let Some(colors) = self.original_colors.get(link_name) {
            if let Some(obj) = self.scenes.get_mut(link_name) {
                for color in colors {
                    obj.apply_to_scene_nodes_mut(&mut |o| {
                        o.set_color(color[0], color[1], color[2]);
                    });
                }
            }
        }
    }
}
