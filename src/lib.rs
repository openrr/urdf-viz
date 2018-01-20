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

extern crate alga;
extern crate glfw;
extern crate k;
extern crate kiss3d;
#[macro_use]
extern crate log;
extern crate nalgebra as na;
extern crate urdf_rs;

use kiss3d::resource::Mesh;
use kiss3d::scene::SceneNode;
use std::cell::RefCell;
use std::collections::HashMap;
use std::path::Path;
use std::rc::Rc;
use alga::general::SubsetOf;

mod errors;
pub use errors::*;
mod arc_ball;
use arc_ball::*;

use na::Real;

#[cfg(feature = "assimp")]
pub fn load_mesh<P>(filename: P) -> Result<Rc<RefCell<Mesh>>>
where
    P: AsRef<Path>,
{
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    let file_string = filename.as_ref().to_str().ok_or(
        "faild to get string from path",
    )?;
    Ok(convert_assimp_scene_to_kiss3d_mesh(
        importer.read_file(file_string)?,
    ))
}

#[cfg(feature = "assimp")]
fn convert_assimp_scene_to_kiss3d_mesh(scene: assimp::Scene) -> Rc<RefCell<Mesh>> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut last_index: u32 = 0;
    for mesh in scene.mesh_iter() {
        vertices.extend(mesh.vertex_iter().map(|v| na::Point3::new(v.x, v.y, v.z)));
        indices.extend(mesh.face_iter().filter_map(|f| if f.num_indices == 3 {
            Some(na::Point3::new(
                f[0] + last_index,
                f[1] + last_index,
                f[2] + last_index,
            ))
        } else {
            None
        }));
        last_index = vertices.len() as u32;
    }
    Rc::new(RefCell::new(
        Mesh::new(vertices, indices, None, None, false),
    ))
}

#[cfg(not(feature = "assimp"))]
pub fn load_mesh<P>(_filename: P) -> Result<Rc<RefCell<Mesh>>>
where
    P: AsRef<Path>,
{
    Err(Error::from("load mesh is disabled by feature"))
}

fn add_geometry(
    geometry: &urdf_rs::Geometry,
    base_dir: Option<&Path>,
    group: &mut SceneNode,
) -> Option<SceneNode> {
    match *geometry {
        urdf_rs::Geometry::Box { ref size } => Some(group.add_cube(
            size[0] as f32,
            size[1] as f32,
            size[2] as f32,
        )),
        urdf_rs::Geometry::Cylinder { radius, length } => {
            let mut base = group.add_group();
            let mut cylinder = base.add_cylinder(radius as f32, length as f32);
            cylinder.append_rotation(&na::UnitQuaternion::from_axis_angle(
                &na::Vector3::x_axis(),
                1.57,
            ));
            Some(base)
        }
        urdf_rs::Geometry::Sphere { radius } => Some(group.add_sphere(radius as f32)),
        urdf_rs::Geometry::Mesh {
            ref filename,
            scale,
        } => {
            let replaced_filename = urdf_rs::utils::expand_package_path(filename, base_dir);
            let path = Path::new(&replaced_filename);
            if !path.exists() {
                error!("{} not found", replaced_filename);
                return None;
            }
            let na_scale = na::Vector3::new(scale[0] as f32, scale[1] as f32, scale[2] as f32);
            if cfg!(feature = "assimp") {
                if let Ok(mesh) = load_mesh(path) {
                    Some(group.add_mesh(mesh, na_scale))
                } else {
                    None
                }
            } else {
                if path.extension() == Some(std::ffi::OsStr::new("obj")) {
                    Some(group.add_obj(path, path, na_scale))
                } else {
                    error!(
                        "{:?} is not supported, because assimp feature is disabled",
                        path
                    );
                    Some(group.add_cube(0.05f32, 0.05, 0.05))
                }
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
        .map(|mat| mat.clone()) {
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
}

impl Viewer {
    pub fn new(title: &str) -> Viewer {
        let eye = na::Point3::new(3.0f32, 0.0, 1.0);
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
        }
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
                if let Some(mut scene_node) =
                    add_geometry(geom_element, base_dir, &mut scene_group)
                {
                    if l.visual.len() > i {
                        let rgba = rgba_from_visual(urdf_robot, &l.visual[i]);
                        let color = na::Point3::new(rgba[0] as f32, rgba[1] as f32, rgba[2] as f32);
                        scene_node.set_color(color[0], color[1], color[2]);
                        colors.push(color);
                    }
                    let origin = na::Isometry3::from_parts(
                        k::urdf::translation_from(&origin_element.xyz),
                        k::urdf::quaternion_from(&origin_element.rpy),
                    );
                    // set initial origin offset
                    scene_node.set_local_transformation(origin);
                } else {
                    error!("failed to create for {:?}", l);
                }
            }
            self.scenes.insert(l.name.to_string(), scene_group);
            self.original_colors.insert(l.name.to_string(), colors);
        }
    }
    pub fn remove_robot(&mut self, urdf_robot: &urdf_rs::Robot) {
        for l in &urdf_robot.links {
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
    pub fn update<R, T>(&mut self, robot: &R)
    where
        R: k::LinkContainer<T>,
        T: Real + alga::general::SubsetOf<f32>,
    {
        for (trans, link_name) in
            robot.link_transforms().iter().zip(
                robot.link_names().iter(),
            )
        {
            let trans_f32: na::Isometry3<f32> = na::Isometry3::to_superset(&*trans);
            match self.scenes.get_mut(&*link_name) {
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
            self.font_map.entry(size).or_insert(
                kiss3d::text::Font::from_memory(
                    self.font_data,
                    size,
                ),
            ),
            color,
        );
    }
    pub fn events(&self) -> kiss3d::window::EventManager {
        self.window.events()
    }
    pub fn set_temporal_color(&mut self, link_name: &str, r: f32, g: f32, b: f32) {
        self.scenes.get_mut(link_name).map(|obj| {
            obj.set_color(r, g, b);
        });
    }
    pub fn reset_temporal_color(&mut self, link_name: &str) {
        if let Some(colors) = self.original_colors.get(link_name) {
            let mut i = 0;
            self.scenes.get_mut(link_name).map(|obj| {
                obj.apply_to_scene_nodes_mut(&mut |o| {
                    o.set_color(colors[i][0], colors[i][1], colors[i][2])
                });
                i += 1;
            });
        }
    }
}
