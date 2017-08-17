//! # urdf visualization
//!
//! # Limitation
//!
//! ## Mesh
//!
//! Only `.obj` is supported by [kiss3d](https://github.com/sebcrozet/kiss3d),
//! the visualization library used this crate.
//! Other files are converted by `meshlabserver`.
//! If you add `-a` option, `assimp` is used instead of meshlab.
//! You need to install meshlab or assimp anyway.
//!
extern crate alga;
extern crate glfw;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate k;
extern crate regex;
extern crate urdf_rs;
#[macro_use]
extern crate log;
extern crate rayon;
extern crate structopt;
#[macro_use]
extern crate structopt_derive;

use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use rayon::prelude::*;
use regex::Regex;
use std::collections::HashMap;
use std::fs;
use std::io;
use std::path::Path;
use std::path::PathBuf;
use std::process::Command;
use std::rc::Rc;

pub enum MeshConvert {
    Assimp,
    Meshlab,
}

fn get_cache_dir() -> &'static str {
    "/tmp/urdf_vis/"
}

pub fn clean_cahce_dir() -> io::Result<()> {
    fs::remove_dir_all(get_cache_dir())
}

fn create_parent_dir(new_path: &Path) -> Result<(), std::io::Error> {
    let new_parent_dir = new_path.parent().unwrap();
    if !new_parent_dir.is_dir() {
        info!("creating dir {}", new_parent_dir.to_str().unwrap());
        std::fs::create_dir_all(new_parent_dir)?;
    }
    Ok(())
}

pub fn convert_xacro_to_urdf<P>(filename: P, new_path: P) -> Result<(), std::io::Error>
    where P: AsRef<Path>
{
    create_parent_dir(new_path.as_ref())?;
    let output = Command::new("rosrun")
        .args(&["xacro",
                "xacro",
                "--inorder",
                filename.as_ref().to_str().unwrap(),
                "-o",
                new_path.as_ref().to_str().unwrap()])
        .output()
        .expect("failed to execute xacro. install by apt-get install ros-*-xacro");
    if output.status.success() {
        Ok(())
    } else {
        error!("{}", String::from_utf8(output.stderr).unwrap());
        Err(std::io::Error::new(std::io::ErrorKind::Other, "faild to xacro"))
    }
}

pub fn convert_to_obj_file_by_meshlab(filename: &Path,
                                      new_path: &Path)
                                      -> Result<(), std::io::Error> {
    create_parent_dir(new_path)?;
    info!("converting {:?} to {:?}", filename, new_path);
    let output = Command::new("meshlabserver")
        .args(&["-i", filename.to_str().unwrap(), "-o", new_path.to_str().unwrap()])
        .output()
        .expect("failed to execute meshlabserver. install by apt-get install meshlab");
    if output.status.success() {
        Ok(())
    } else {
        Err(std::io::Error::new(std::io::ErrorKind::Other, "faild to meshlab"))
    }
}

pub fn convert_to_obj_file_by_assimp(filename: &Path,
                                     new_path: &Path)
                                     -> Result<(), std::io::Error> {
    create_parent_dir(new_path)?;
    info!("converting {:?} to {:?}", filename, new_path);
    let output =
        Command::new("assimp")
            .args(&["export", filename.to_str().unwrap(), new_path.to_str().unwrap(), "-ptv"])
            .output()
            .expect("failed to execute meshlabserver. install by apt-get install assimp-utils");
    if output.status.success() {
        Ok(())
    } else {
        Err(std::io::Error::new(std::io::ErrorKind::Other, "faild to assimp"))
    }
}


fn rospack_find(package: &str) -> Option<String> {
    let output = Command::new("rospack")
        .arg("find")
        .arg(package)
        .output()
        .expect("rospack find failed");
    if output.status.success() {
        String::from_utf8(output.stdout)
            .map(|string| string.trim().to_string())
            .ok()
    } else {
        None
    }
}


fn expand_package_path(filename: &str, base_dir: &Path) -> String {
    if filename.starts_with("package://") {
        let re = Regex::new("^package://(\\w+)/").unwrap();
        re.replace(filename,
                   |ma: &regex::Captures| match rospack_find(&ma[1]) {
                       Some(found_path) => found_path + "/",
                       None => panic!("failed to find ros package {}", &ma[1]),
                   })
    } else {
        let mut relative_path_from_urdf = base_dir.to_owned();
        relative_path_from_urdf.push(filename);
        relative_path_from_urdf.to_str().unwrap().to_owned()
    }
}

fn get_cache_or_obj_path(path: &Path) -> PathBuf {
    match path.extension() {
        Some(ext) => {
            if ext == "obj" {
                return path.to_path_buf();
            }
        }
        None => panic!("not supported mesh file {:?}", path),
    }
    let cache_path = get_cache_dir().to_string() + &path.with_extension("obj").to_str().unwrap();
    info!("cache obj path = {:?}", cache_path);
    Path::new(&cache_path).to_path_buf()
}

fn convert_mesh_if_needed(filename: &str, mesh_convert: &MeshConvert, base_dir: &Path) {
    let replaced_filename = expand_package_path(filename, base_dir);
    let path = Path::new(&replaced_filename);
    assert!(path.exists(), "{} not found", replaced_filename);
    let new_path = get_cache_or_obj_path(path);
    if !new_path.exists() {
        match *mesh_convert {
                MeshConvert::Assimp => convert_to_obj_file_by_assimp(path, new_path.as_path()),
                MeshConvert::Meshlab => convert_to_obj_file_by_meshlab(path, new_path.as_path()),
            }
            .unwrap_or_else(|err| {
                                panic!("failed to convert mesh {:?}: {}",
                                       new_path.into_os_string(),
                                       err);
                            });
    }
}


fn add_geometry(visual: &urdf_rs::Visual,
                base_dir: &Path,
                window: &mut Window)
                -> Option<SceneNode> {
    let mut geom = match visual.geometry {
        urdf_rs::Geometry::Box { ref size } => {
            Some(window.add_cube(size[0] as f32, size[1] as f32, size[2] as f32))
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            Some(window.add_cylinder(radius as f32, length as f32))
        }
        urdf_rs::Geometry::Sphere { radius } => Some(window.add_sphere(radius as f32)),
        urdf_rs::Geometry::Mesh {
            ref filename,
            scale,
        } => {
            let replaced_filename = expand_package_path(filename, base_dir);
            let path = Path::new(&replaced_filename);
            if !path.exists() {
                error!("{} not found", replaced_filename);
                return None;
            }
            let new_path = get_cache_or_obj_path(path);
            // mtl_path works for only assimp
            let mtl_path = new_path.with_extension("obj.mtl");
            debug!("mtl_path = {}", mtl_path.to_str().unwrap());
            // should be generated in advance
            if !new_path.exists() {
                error!("converted file {:?} not found", new_path.into_os_string());
                return None;
            }
            Some(window.add_obj(new_path.as_path(),
                                mtl_path.as_path(),
                                na::Vector3::new(scale[0] as f32,
                                                 scale[1] as f32,
                                                 scale[2] as f32)))
        }
    };
    let rgba = &visual.material.color.rgba;
    match geom {
        Some(ref mut obj) => obj.set_color(rgba[0] as f32, rgba[1] as f32, rgba[2] as f32),
        None => return None,
    }
    geom
}

pub struct Viewer {
    pub window: kiss3d::window::Window,
    pub urdf_robot: urdf_rs::Robot,
    pub scenes: HashMap<String, SceneNode>,
    pub arc_ball: kiss3d::camera::ArcBall,
    font_map: HashMap<i32, Rc<kiss3d::text::Font>>,
    font_data: &'static [u8],
    original_colors: HashMap<String, na::Point3<f32>>,
}

impl Viewer {
    pub fn new(urdf_robot: urdf_rs::Robot) -> Viewer {
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
    pub fn setup(&mut self, mesh_convert: MeshConvert, base_dir: &Path) {
        self.window
            .set_light(kiss3d::light::Light::StickToCamera);

        self.window.set_background_color(0.0, 0.0, 0.3);
        let _ = self.urdf_robot
            .links
            .par_iter()
            .map(|l| if let urdf_rs::Geometry::Mesh {
                            filename: ref f,
                            scale: _,
                        } = l.visual.geometry {
                     convert_mesh_if_needed(f, &mesh_convert, base_dir);
                 })
            .count();
        for l in &self.urdf_robot.links {
            if let Some(geom) = add_geometry(&l.visual, base_dir, &mut self.window) {
                self.scenes.insert(l.name.to_string(), geom);
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
        self.scenes.insert(name.to_owned(), axis_group);
    }
    pub fn render(&mut self) -> bool {
        self.window.render_with_camera(&mut self.arc_ball)
    }
    pub fn update(&mut self, robot: &mut k::LinkTree<f32>) {
        for (trans, link_name) in
            robot
                .calc_link_transforms()
                .iter()
                .zip(robot.map_link(&|link| link.name.clone())) {
            match self.scenes.get_mut(&link_name) {
                Some(obj) => obj.set_local_transformation(*trans),
                None => {
                    println!("{} not found", link_name);
                }
            }
        }
    }
    pub fn draw_text(&mut self,
                     text: &str,
                     size: i32,
                     pos: &na::Point2<f32>,
                     color: &na::Point3<f32>) {
        self.window
            .draw_text(text,
                       pos,
                       self.font_map
                           .entry(size)
                           .or_insert(kiss3d::text::Font::from_memory(self.font_data, size)),
                       color);
    }
    pub fn events(&self) -> kiss3d::window::EventManager {
        self.window.events()
    }
    pub fn set_temporal_color(&mut self, link_name: &str, r: f32, g: f32, b: f32) {
        let color_opt = self.scenes
            .get_mut(link_name)
            .map(|obj| {
                     let orig_color = match obj.data().object() {
                         Some(object) => Some(object.data().color().to_owned()),
                         None => None,
                     };
                     obj.set_color(r, g, b);
                     orig_color
                 })
            .unwrap_or(None);
        if let Some(color) = color_opt {
            self.original_colors.insert(link_name.to_string(), color);
        }
    }
    pub fn reset_temporal_color(&mut self, link_name: &str) {
        if let Some(original_color) = self.original_colors.get(link_name) {
            self.scenes
                .get_mut(link_name)
                .map(|obj| {
                         obj.set_color(original_color[0] as f32,
                                       original_color[1] as f32,
                                       original_color[2] as f32)
                     });
        }
    }
}

// TODO: Use error chain
pub fn convert_xacro_if_needed_and_get_path(input_path: &Path) -> Result<PathBuf, std::io::Error> {
    if let Some(ext) = input_path.extension() {
        if ext == "xacro" {
            let abs_urdf_path = get_cache_dir().to_string() +
                                input_path.with_extension("urdf").to_str().unwrap();
            let tmp_urdf_path = Path::new(&abs_urdf_path);
            convert_xacro_to_urdf(&input_path, &tmp_urdf_path)?;
            Ok(tmp_urdf_path.to_owned())
        } else {
            Ok(input_path.to_owned())
        }
    } else {
        Err(std::io::Error::new(std::io::ErrorKind::Other, "failed to get extension"))
    }
}

#[derive(StructOpt, Debug)]
#[structopt(name = "urdf_viz", about = "Option for visualizing urdf")]
pub struct Opt {
    #[structopt(short = "a", long = "assimp",
                help = "Use assimp instead of meshlab to convert .dae to .obj for visualization")]
    pub assimp: bool,
    #[structopt(short = "c", long = "clean",
                help = "Clean the caches which is created by assimp or meshlab")]
    pub clean: bool,
    #[structopt(short = "d", long = "dof", help = "max dof for ik", default_value = "6")]
    pub ik_dof: usize,
    #[structopt(help = "Input urdf or xacro")]
    pub input_urdf_or_xacro: String,
}

impl Opt {
    pub fn get_mesh_convert_method(&self) -> MeshConvert {
        if self.assimp {
            MeshConvert::Assimp
        } else {
            MeshConvert::Meshlab
        }
    }
}

#[test]
fn test_func() {
    let input = Path::new("/home/user/robo.urdf");
    assert!(expand_package_path("mesh/aaa.obj", input.parent().unwrap()) ==
            "/home/user/mesh/aaa.obj");

}
