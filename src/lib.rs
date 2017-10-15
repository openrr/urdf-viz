//! urdf-viz
//! ==================
//!
//! Visualize [URDF(Unified Robot Description Format)](http://wiki.ros.org/urdf) file.
//! `urdf-viz` is written by rust-lang.
//!

#[cfg(feature = "assimp")]
extern crate assimp;

extern crate glfw;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate k;
extern crate urdf_rs;
#[macro_use]
extern crate log;
extern crate structopt;
#[macro_use]
extern crate structopt_derive;

use kiss3d::resource::Mesh;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use std::cell::RefCell;
use std::collections::HashMap;
use std::path::Path;
use std::rc::Rc;

mod errors;
pub use errors::*;

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

#[cfg(feature = "assimp")]
fn set_verbose() {
    assimp::LogStream::set_verbose_logging(true);
    let mut log_stream = assimp::LogStream::stdout();
    log_stream.attach();
}

#[cfg(not(feature = "assimp"))]
fn set_verbose() {}

fn add_geometry(
    visual: &urdf_rs::Visual,
    base_dir: &Path,
    window: &mut Window,
) -> Option<SceneNode> {
    match visual.geometry {
        urdf_rs::Geometry::Box { ref size } => {
            Some(window.add_cube(
                size[0] as f32,
                size[1] as f32,
                size[2] as f32,
            ))
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            Some(window.add_cylinder(radius as f32, length as f32))
        }
        urdf_rs::Geometry::Sphere { radius } => Some(window.add_sphere(radius as f32)),
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
                    Some(window.add_mesh(mesh, na_scale))
                } else {
                    None
                }
            } else {
                if path.extension() == Some(std::ffi::OsStr::new("obj")) {
                    Some(window.add_obj(path, path, na_scale))
                } else {
                    error!(
                        "{:?} is not supported, because assimp feature is disabled",
                        path
                    );
                    Some(window.add_cube(0.05f32, 0.05, 0.05))
                }
            }
        }
    }
}

// we can remove this if we use group, but it need fix of temporal color
pub struct SceneNodeAndTransform(pub SceneNode, pub na::Isometry3<f32>);

pub struct Viewer<'a> {
    pub window: kiss3d::window::Window,
    pub urdf_robot: &'a urdf_rs::Robot,
    pub scenes: HashMap<String, SceneNodeAndTransform>,
    pub arc_ball: kiss3d::camera::ArcBall,
    font_map: HashMap<i32, Rc<kiss3d::text::Font>>,
    font_data: &'static [u8],
    original_colors: HashMap<String, na::Point3<f32>>,
}

impl<'a> Viewer<'a> {
    pub fn new(urdf_robot: &'a urdf_rs::Robot) -> Viewer {
        let eye = na::Point3::new(0.5f32, 1.0, -3.0);
        let at = na::Point3::new(0.0f32, 0.25, 0.0);
        Viewer {
            window: kiss3d::window::Window::new_with_size("urdf_viewer", 1400, 1000),
            urdf_robot: urdf_robot,
            scenes: HashMap::new(),
            arc_ball: kiss3d::camera::ArcBall::new(eye, at),
            font_map: HashMap::new(),
            font_data: include_bytes!("font/Inconsolata.otf"),
            original_colors: HashMap::new(),
        }
    }
    pub fn setup(&mut self, base_dir: &Path, is_verbose: bool) {
        if is_verbose {
            set_verbose()
        }
        self.window.set_light(kiss3d::light::Light::StickToCamera);

        self.window.set_background_color(0.0, 0.0, 0.3);
        for l in &self.urdf_robot.links {
            if let Some(mut geom) = add_geometry(&l.visual, base_dir, &mut self.window) {
                match self.urdf_robot
                    .materials
                    .iter()
                    .find(|mat| mat.name == l.visual.material.name)
                    .map(|mat| mat.clone()) {
                    Some(ref material) => {
                        geom.set_color(
                            material.color.rgba[0] as f32,
                            material.color.rgba[1] as f32,
                            material.color.rgba[2] as f32,
                        )
                    }
                    None => {
                        let rgba = &l.visual.material.color.rgba;
                        geom.set_color(rgba[0] as f32, rgba[1] as f32, rgba[2] as f32);
                    }
                }
                self.scenes.insert(
                    l.name.to_string(),
                    SceneNodeAndTransform(
                        geom,
                        na::Isometry3::from_parts(
                            k::urdf::translation_from(&l.visual.origin.xyz),
                            k::urdf::quaternion_from(&l.visual.origin.rpy),
                        ),
                    ),
                );
            } else {
                error!("failed to create for {:?}", l.visual);
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
        self.scenes.insert(
            name.to_owned(),
            SceneNodeAndTransform(axis_group, na::Isometry3::identity()),
        );
    }
    pub fn render(&mut self) -> bool {
        self.window.render_with_camera(&mut self.arc_ball)
    }
    pub fn update(&mut self, robot: &k::LinkTree<f32>) {
        for (trans, link_name) in
            robot.calc_link_transforms().iter().zip(
                robot.iter_link().map(
                    |link| link.name.clone(),
                ),
            )
        {
            match self.scenes.get_mut(&link_name) {
                Some(obj) => obj.0.set_local_transformation(*trans * obj.1),
                None => {
                    println!("{} not found", link_name);
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
        let color_opt = self.scenes
            .get_mut(link_name)
            .map(|obj| {
                let orig_color = match obj.0.data().object() {
                    Some(object) => Some(object.data().color().to_owned()),
                    None => None,
                };
                obj.0.set_color(r, g, b);
                orig_color
            })
            .unwrap_or(None);
        if let Some(color) = color_opt {
            self.original_colors.insert(link_name.to_string(), color);
        }
    }
    pub fn reset_temporal_color(&mut self, link_name: &str) {
        if let Some(original_color) = self.original_colors.get(link_name) {
            self.scenes.get_mut(link_name).map(|obj| {
                obj.0.set_color(
                    original_color[0] as f32,
                    original_color[1] as f32,
                    original_color[2] as f32,
                )
            });
        }
    }
}

#[derive(StructOpt, Debug)]
#[structopt(name = "urdf_viz", about = "Option for visualizing urdf")]
pub struct Opt {
    #[structopt(short = "d", long = "dof",
                help = "limit the dof for ik to avoid use fingers as end effectors",
                default_value = "6")]
    pub ik_dof: usize,
    #[structopt(short = "v", long = "verbose", help = "show assimp log")]
    pub verbose: bool,
    #[structopt(help = "Input urdf or xacro")]
    pub input_urdf_or_xacro: String,
}
