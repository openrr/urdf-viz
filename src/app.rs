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

#![cfg_attr(target_arch = "wasm32", allow(dead_code))]

use k::nalgebra as na;
use k::prelude::*;
use kiss3d::event::{Action, Key, Modifiers, WindowEvent};
use kiss3d::window::{self, Window};
use serde::Deserialize;
use std::path::PathBuf;
use std::sync::{
    atomic::{AtomicBool, Ordering::Relaxed},
    Arc,
};
use structopt::StructOpt;

use crate::{utils::RobotModel, Viewer};
#[cfg(not(target_arch = "wasm32"))]
use crate::{Data, JointNamesAndPositions, RobotOrigin, WebServer};

#[cfg(target_os = "macos")]
static NATIVE_MOD: Modifiers = Modifiers::Super;

#[cfg(not(target_os = "macos"))]
static NATIVE_MOD: Modifiers = Modifiers::Control;

fn move_joint_by_random(robot: &mut k::Chain<f32>) -> Result<(), k::Error> {
    let angles_vec = robot
        .iter_joints()
        .map(|j| match j.limits {
            Some(ref range) => (range.max - range.min) * rand::random::<f32>() + range.min,
            None => (rand::random::<f32>() - 0.5) * 2.0,
        })
        .collect::<Vec<f32>>();
    robot.set_joint_positions(&angles_vec)
}

fn move_joint_to_zero(robot: &mut k::Chain<f32>) -> Result<(), k::Error> {
    let angles_vec = vec![0.0; robot.dof()];
    robot.set_joint_positions(&angles_vec)
}

fn move_joint_by_index(
    index: usize,
    diff_angle: f32,
    robot: &mut k::Chain<f32>,
) -> Result<(), k::Error> {
    let mut angles_vec = robot.joint_positions();
    assert!(index < robot.dof());
    angles_vec[index] += diff_angle;
    robot.set_joint_positions(&angles_vec)
}

struct LoopIndex {
    index: usize,
    size: usize,
}

impl LoopIndex {
    fn new(size: usize) -> Self {
        Self { index: 0, size }
    }
    fn get(&self) -> usize {
        self.index
    }
    fn inc(&mut self) {
        if self.size != 0 {
            self.index += 1;
            self.index %= self.size;
        }
    }
    fn dec(&mut self) {
        if self.size != 0 {
            if self.index == 0 {
                self.index = self.size - 1;
            } else {
                self.index -= 1;
            }
        }
    }
}

const HOW_TO_USE_STR: &str = r#"o:    joint ID +1
p:    joint ID -1
,:    IK target ID +1
.:    IK target ID -1
Up:   joint angle +0.1
Down: joint angle -0.1
Ctrl+Drag: move joint
Shift+Drag: IK (y, z)
Shift+Ctrl+Drag: IK (x, z)
l:    Reload the file
r:    set random angles
z:    reset joint positions and origin
c:    toggle visual/collision
"#;

pub struct UrdfViewerApp {
    input_path: PathBuf,
    urdf_robot: RobotModel,
    needs_reload: Arc<AtomicBool>,
    robot: k::Chain<f32>,
    viewer: Viewer,
    window: Option<Window>,
    arms: Vec<k::SerialChain<f32>>,
    names: Vec<String>,
    input_end_link_names: Vec<String>,
    num_joints: usize,
    index_of_arm: LoopIndex,
    index_of_move_joint: LoopIndex,
    web_server_port: u16,
    is_collision: bool,
    ik_constraints: k::Constraints,
}

impl UrdfViewerApp {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        input_file: &str,
        urdf_robot: urdf_rs::Robot,
        mut end_link_names: Vec<String>,
        is_collision: bool,
        disable_texture: bool,
        web_server_port: u16,
        background_color: (f32, f32, f32),
        tile_color1: (f32, f32, f32),
        tile_color2: (f32, f32, f32),
        ground_height: Option<f32>,
    ) -> Self {
        let input_path = PathBuf::from(input_file);
        let robot: k::Chain<f32> = (&urdf_robot).into();
        println!("{}", robot);
        let (mut viewer, mut window) = Viewer::with_background_color("urdf-viz", background_color);
        if disable_texture {
            viewer.disable_texture();
        }
        viewer.add_robot_with_base_dir_and_collision_flag(
            &mut window,
            &urdf_robot,
            input_path.parent(),
            is_collision,
        );
        viewer.add_axis_cylinders(&mut window, "origin", 1.0);
        if let Some(h) = ground_height {
            viewer.add_ground(&mut window, h, 0.5, 3, tile_color1, tile_color2);
        }
        let input_end_link_names = end_link_names.clone();
        if end_link_names.is_empty() {
            end_link_names = robot
                .iter()
                .filter(|node| node.is_end())
                .map(|node| node.joint().name.clone())
                .collect::<Vec<_>>();
        }
        let arms = end_link_names
            .iter()
            .filter_map(|name| robot.find(name).map(|j| k::SerialChain::from_end(j)))
            .collect::<Vec<_>>();
        println!("end_link_names = {:?}", end_link_names);
        let names = robot
            .iter_joints()
            .map(|j| j.name.clone())
            .collect::<Vec<_>>();
        let num_arms = end_link_names.len();
        let num_joints = names.len();
        println!("DoF={}", num_joints);
        println!("joint names={:?}", names);
        let (urdf_robot, needs_reload) = RobotModel::new(urdf_robot);
        UrdfViewerApp {
            input_path,
            viewer,
            window: Some(window),
            arms,
            input_end_link_names,
            urdf_robot,
            needs_reload,
            robot,
            num_joints,
            names,
            index_of_arm: LoopIndex::new(num_arms),
            index_of_move_joint: LoopIndex::new(num_joints),
            web_server_port,
            is_collision,
            ik_constraints: k::Constraints::default(),
        }
    }
    pub fn set_ik_constraints(&mut self, ik_constraints: k::Constraints) {
        self.ik_constraints = ik_constraints;
    }
    fn has_arms(&self) -> bool {
        !self.arms.is_empty()
    }
    fn has_joints(&self) -> bool {
        self.num_joints > 0
    }
    pub fn init(&mut self) {
        self.update_robot();
        if self.has_arms() {
            let mut window = self.window.as_mut().unwrap();
            self.viewer
                .add_axis_cylinders(&mut window, "ik_target", 0.2);
            self.update_ik_target_marker();
        }
    }
    fn get_arm(&self) -> &k::SerialChain<f32> {
        &self.arms[self.index_of_arm.get()]
    }
    fn get_end_transform(&self) -> na::Isometry3<f32> {
        self.get_arm().end_transform()
    }
    fn update_ik_target_marker(&mut self) {
        if self.has_arms() {
            let pose = self.get_end_transform();

            if let Some(obj) = self.viewer.scene_node_mut("ik_target") {
                obj.set_local_transformation(pose);
            };
        }
    }
    fn update_robot(&mut self) {
        // this is hack to handle invalid mimic joints, like hsr
        let joint_positions = self.robot.joint_positions();
        self.robot
            .set_joint_positions(&joint_positions)
            .unwrap_or_else(|err| println!("failed to update robot joints {}", err));
        self.viewer.update(&self.robot);
        self.update_ik_target_marker();
    }
    fn request_reload(&mut self) {
        self.urdf_robot
            .request_reload(self.input_path.to_str().unwrap());
    }
    fn reload_urdf(&mut self, window: &mut Window) {
        let urdf_robot = self.urdf_robot.get();
        self.viewer.remove_robot(window, urdf_robot);
        self.robot = urdf_robot.into();
        let end_link_names = if self.input_end_link_names.is_empty() {
            self.robot
                .iter()
                .filter(|node| node.is_end())
                .map(|node| node.joint().name.clone())
                .collect::<Vec<_>>()
        } else {
            self.input_end_link_names.clone()
        };
        self.arms = end_link_names
            .iter()
            .filter_map(|name| self.robot.find(name).map(|j| k::SerialChain::from_end(j)))
            .collect::<Vec<_>>();
        self.names = self.robot.iter_joints().map(|j| j.name.clone()).collect();
    }
    fn reload_and_update(&mut self, window: &mut Window) {
        if self.needs_reload.swap(false, Relaxed) {
            self.reload_urdf(window);
            self.viewer.add_robot_with_base_dir_and_collision_flag(
                window,
                &self.urdf_robot.get(),
                self.input_path.parent(),
                self.is_collision,
            );
            self.update_robot();
        }
    }

    /// Handle set_joint_positions request from web server
    #[cfg(not(target_arch = "wasm32"))]
    fn set_joint_positions_from_request(
        &mut self,
        joint_positions: &JointNamesAndPositions,
    ) -> Result<(), k::Error> {
        let mut angles = self.robot.joint_positions();
        for (name, angle) in joint_positions
            .names
            .iter()
            .zip(joint_positions.positions.iter())
        {
            if let Some(index) = self.names.iter().position(|ref n| *n == name) {
                angles[index] = *angle;
            } else {
                println!("{} not found, but continues", name);
            }
        }
        self.robot.set_joint_positions(&angles)
    }

    /// Handle set_origin request from web server
    #[cfg(not(target_arch = "wasm32"))]
    fn set_robot_origin_from_request(&mut self, origin: &RobotOrigin) {
        let pos = origin.position;
        let q = origin.quaternion;
        let pose = na::Isometry3::from_parts(
            na::Translation3::new(pos[0], pos[1], pos[2]),
            na::UnitQuaternion::new_normalize(na::Quaternion::new(q[0], q[1], q[2], q[3])),
        );
        self.robot.set_origin(pose);
    }

    fn increment_move_joint_index(&mut self, is_inc: bool) {
        if self.has_joints() {
            self.viewer
                .reset_temporal_color(&self.names[self.index_of_move_joint.get()]);
            if is_inc {
                self.index_of_move_joint.inc();
            } else {
                self.index_of_move_joint.dec();
            }
            self.viewer.set_temporal_color(
                &self.names[self.index_of_move_joint.get()],
                1.0,
                0.0,
                0.0,
            );
        }
    }
    fn handle_key_press(&mut self, window: &mut Window, code: Key) {
        match code {
            Key::O | Key::LBracket => self.increment_move_joint_index(true),
            Key::P | Key::RBracket => self.increment_move_joint_index(false),
            Key::Period => {
                self.index_of_arm.inc();
                self.update_ik_target_marker();
            }
            Key::Comma => {
                self.index_of_arm.dec();
                self.update_ik_target_marker();
            }
            Key::A => {
                let mut origin = self.robot.origin();
                origin.translation.vector[1] += 0.1;
                self.robot.set_origin(origin);
                self.update_robot();
            }
            Key::S => {
                let mut origin = self.robot.origin();
                origin.translation.vector[0] -= 0.1;
                self.robot.set_origin(origin);
                self.update_robot();
            }
            Key::D => {
                let mut origin = self.robot.origin();
                origin.translation.vector[1] -= 0.1;
                self.robot.set_origin(origin);
                self.update_robot();
            }
            Key::W => {
                let mut origin = self.robot.origin();
                origin.translation.vector[0] += 0.1;
                self.robot.set_origin(origin);
                self.update_robot();
            }
            Key::C => {
                self.viewer.remove_robot(window, self.urdf_robot.get());
                self.is_collision = !self.is_collision;
                self.viewer.add_robot_with_base_dir_and_collision_flag(
                    window,
                    self.urdf_robot.get(),
                    self.input_path.parent(),
                    self.is_collision,
                );
                self.update_robot();
            }
            Key::L => {
                // reload
                self.request_reload();
                self.reload_and_update(window);
            }
            Key::R => {
                if self.has_joints() {
                    move_joint_by_random(&mut self.robot).unwrap_or(());
                    self.update_robot();
                }
            }
            Key::Z => {
                self.robot.set_origin(na::Isometry::identity());
                if self.has_joints() {
                    move_joint_to_zero(&mut self.robot).unwrap_or(());
                }
                self.update_robot();
            }
            Key::Up => {
                if self.has_joints() {
                    move_joint_by_index(self.index_of_move_joint.get(), 0.1, &mut self.robot)
                        .unwrap_or(());
                    self.update_robot();
                }
            }
            Key::Down => {
                if self.has_joints() {
                    move_joint_by_index(self.index_of_move_joint.get(), -0.1, &mut self.robot)
                        .unwrap_or(());
                    self.update_robot();
                }
            }
            _ => {}
        };
    }
    pub fn run(mut self) {
        let window = self.window.take().unwrap();
        let is_ctrl = false;
        let is_shift = false;
        let last_cur_pos_y = 0f64;
        let last_cur_pos_x = 0f64;
        let solver = k::JacobianIkSolver::default();

        #[cfg(not(target_arch = "wasm32"))]
        let data = {
            let web_server = WebServer::new(self.web_server_port);
            let data = web_server.data();
            if let Ok(mut cur_ja) = data.current_joint_positions.lock() {
                cur_ja.names = self.names.clone();
            }

            std::thread::spawn(move || web_server.start());

            ctrlc::set_handler(|| {
                ABORTED.store(true, Relaxed);
            })
            .unwrap();

            data
        };

        let state = AppState {
            app: self,
            #[cfg(not(target_arch = "wasm32"))]
            data,
            solver,
            is_ctrl,
            is_shift,
            last_cur_pos_x,
            last_cur_pos_y,
        };
        window.render_loop(state);
    }
}

#[cfg(not(target_arch = "wasm32"))]
static ABORTED: AtomicBool = AtomicBool::new(false);

struct AppState {
    app: UrdfViewerApp,
    #[cfg(not(target_arch = "wasm32"))]
    data: Arc<Data>,
    solver: k::JacobianIkSolver<f32>,
    is_ctrl: bool,
    is_shift: bool,
    last_cur_pos_y: f64,
    last_cur_pos_x: f64,
}

impl AppState {
    #[cfg(not(target_arch = "wasm32"))]
    fn handle_http_request(&mut self) {
        // Joint positions for web server
        if let Ok(mut ja) = self.data.target_joint_positions.lock() {
            if ja.requested {
                match self
                    .app
                    .set_joint_positions_from_request(&ja.joint_positions)
                {
                    Ok(_) => {
                        self.app.update_robot();
                        ja.requested = false;
                    }
                    Err(err) => {
                        println!("{}", err);
                    }
                }
            }
        }
        if let Ok(mut cur_ja) = self.data.current_joint_positions.lock() {
            cur_ja.positions = self.app.robot.joint_positions();
        }

        // Robot orientation for web server
        if let Ok(mut ro) = self.data.target_robot_origin.lock() {
            if ro.requested {
                self.app.set_robot_origin_from_request(&ro.origin);
                self.app.update_robot();
                ro.requested = false;
            }
        }
        if let Ok(mut cur_ro) = self.data.current_robot_origin.lock() {
            let o = self.app.robot.origin();
            for i in 0..3 {
                cur_ro.position[i] = o.translation.vector[i];
            }
            cur_ro.quaternion[0] = o.rotation.quaternion().w;
            cur_ro.quaternion[1] = o.rotation.quaternion().i;
            cur_ro.quaternion[2] = o.rotation.quaternion().j;
            cur_ro.quaternion[3] = o.rotation.quaternion().k;
        }
    }
}

impl window::State for AppState {
    fn step(&mut self, window: &mut Window) {
        const FONT_SIZE_USAGE: f32 = 60.0;
        const FONT_SIZE_INFO: f32 = 80.0;

        #[cfg(not(target_arch = "wasm32"))]
        if ABORTED.load(Relaxed) {
            window.close();
            return;
        }

        self.app.reload_and_update(window);

        self.app.viewer.draw_text(
            window,
            HOW_TO_USE_STR,
            FONT_SIZE_USAGE,
            &na::Point2::new(2000.0, 10.0),
            &na::Point3::new(1f32, 1.0, 1.0),
        );
        if self.app.has_joints() {
            self.app.viewer.draw_text(
                window,
                &format!(
                    "moving joint name [{}]",
                    self.app.names[self.app.index_of_move_joint.get()]
                ),
                FONT_SIZE_INFO,
                &na::Point2::new(10f32, 20.0),
                &na::Point3::new(0.5f32, 0.5, 1.0),
            );

            #[cfg(not(target_arch = "wasm32"))]
            self.handle_http_request();
        }
        if self.app.has_arms() {
            let name = &self
                .app
                .get_arm()
                .iter()
                .last()
                .unwrap()
                .joint()
                .name
                .to_owned();
            self.app.viewer.draw_text(
                window,
                &format!("IK target name [{}]", name),
                FONT_SIZE_INFO,
                &na::Point2::new(10f32, 100.0),
                &na::Point3::new(0.5f32, 0.8, 0.2),
            );
        }
        if self.is_ctrl && !self.is_shift {
            self.app.viewer.draw_text(
                window,
                "moving joint by drag",
                FONT_SIZE_INFO,
                &na::Point2::new(10f32, 150.0),
                &na::Point3::new(0.9f32, 0.5, 1.0),
            );
        }
        if self.is_shift {
            self.app.viewer.draw_text(
                window,
                "solving ik",
                FONT_SIZE_INFO,
                &na::Point2::new(10f32, 150.0),
                &na::Point3::new(0.9f32, 0.5, 1.0),
            );
        }
        for mut event in window.events().iter() {
            self.app.reload_and_update(window);
            match event.value {
                WindowEvent::MouseButton(_, Action::Press, mods) => {
                    if mods.contains(NATIVE_MOD) {
                        self.is_ctrl = true;
                        event.inhibited = true;
                    }
                    if mods.contains(Modifiers::Shift) {
                        self.is_shift = true;
                        event.inhibited = true;
                    }
                }
                WindowEvent::CursorPos(x, y, _modifiers) => {
                    if self.is_ctrl && !self.is_shift {
                        event.inhibited = true;
                        let move_gain = 0.005;
                        if self.app.has_joints() {
                            move_joint_by_index(
                                self.app.index_of_move_joint.get(),
                                (((x - self.last_cur_pos_x) + (y - self.last_cur_pos_y))
                                    * move_gain) as f32,
                                &mut self.app.robot,
                            )
                            .unwrap_or(());
                            self.app.update_robot();
                        }
                    }
                    if self.is_shift {
                        event.inhibited = true;
                        if self.app.has_arms() {
                            self.app.robot.update_transforms();
                            let mut target = self.app.get_end_transform();
                            let ik_move_gain = 0.002;
                            target.translation.vector[2] -=
                                ((y - self.last_cur_pos_y) * ik_move_gain) as f32;
                            if self.is_ctrl {
                                target.translation.vector[0] +=
                                    ((x - self.last_cur_pos_x) * ik_move_gain) as f32;
                            } else {
                                target.translation.vector[1] +=
                                    ((x - self.last_cur_pos_x) * ik_move_gain) as f32;
                            }

                            self.app.update_ik_target_marker();
                            let orig_angles = self.app.robot.joint_positions();
                            self.solver
                                .solve_with_constraints(
                                    &self.app.get_arm(),
                                    &target,
                                    &self.app.ik_constraints,
                                )
                                .unwrap_or_else(|err| {
                                    self.app.robot.set_joint_positions_unchecked(&orig_angles);
                                    println!("Err: {}", err);
                                });
                            self.app.update_robot();
                        }
                    }
                    self.last_cur_pos_x = x;
                    self.last_cur_pos_y = y;
                }
                WindowEvent::MouseButton(_, Action::Release, _) => {
                    if self.is_ctrl {
                        self.is_ctrl = false;
                        event.inhibited = true;
                    } else if self.is_shift {
                        self.is_shift = false;
                        event.inhibited = true;
                    }
                }
                WindowEvent::Key(code, Action::Press, _modifiers) => {
                    self.app.handle_key_press(window, code);
                    event.inhibited = true;
                }
                _ => {}
            }
        }
    }

    #[allow(clippy::type_complexity)]
    fn cameras_and_effect_and_renderer(
        &mut self,
    ) -> (
        Option<&mut dyn kiss3d::camera::Camera>,
        Option<&mut dyn kiss3d::planar_camera::PlanarCamera>,
        Option<&mut dyn kiss3d::renderer::Renderer>,
        Option<&mut dyn kiss3d::post_processing::PostProcessingEffect>,
    ) {
        (Some(&mut self.app.viewer.arc_ball), None, None, None)
    }
}

/// Option for visualizing urdf
#[derive(StructOpt, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct Opt {
    /// Input urdf or xacro
    #[serde(default, rename = "urdf")]
    pub input_urdf_or_xacro: String,
    /// end link names
    #[structopt(short = "e", long = "end-link-name")]
    #[serde(default)]
    pub end_link_names: Vec<String>,
    /// Show collision element instead of visual
    #[structopt(short = "c", long = "collision")]
    #[serde(default)]
    pub is_collision: bool,
    /// Disable texture rendering
    #[structopt(short = "d", long = "disable-texture")]
    #[serde(default)]
    pub disable_texture: bool,
    /// Port number for web server interface
    #[structopt(short = "p", long = "web-server-port", default_value = "7777")]
    #[serde(default = "default_web_server_port")]
    pub web_server_port: u16,

    #[structopt(long = "ignore-ik-position-x")]
    #[serde(default)]
    pub ignore_ik_position_x: bool,
    #[structopt(long = "ignore-ik-position-y")]
    #[serde(default)]
    pub ignore_ik_position_y: bool,
    #[structopt(long = "ignore-ik-position-z")]
    #[serde(default)]
    pub ignore_ik_position_z: bool,

    #[structopt(long = "ignore-ik-rotation-x")]
    #[serde(default)]
    pub ignore_ik_rotation_x: bool,
    #[structopt(long = "ignore-ik-rotation-y")]
    #[serde(default)]
    pub ignore_ik_rotation_y: bool,
    #[structopt(long = "ignore-ik-rotation-z")]
    #[serde(default)]
    pub ignore_ik_rotation_z: bool,

    #[structopt(long = "bg-color-r", default_value = "0.0")]
    #[serde(default)]
    pub back_ground_color_r: f32,
    #[structopt(long = "bg-color-g", default_value = "0.0")]
    #[serde(default)]
    pub back_ground_color_g: f32,
    #[structopt(long = "bg-color-b", default_value = "0.3")]
    #[serde(default = "default_back_ground_color_b")]
    pub back_ground_color_b: f32,

    #[structopt(long = "tile-color1-r", default_value = "0.1")]
    #[serde(default = "default_tile_color1")]
    pub tile_color1_r: f32,
    #[structopt(long = "tile-color1-g", default_value = "0.1")]
    #[serde(default = "default_tile_color1")]
    pub tile_color1_g: f32,
    #[structopt(long = "tile-color1-b", default_value = "0.1")]
    #[serde(default = "default_tile_color1")]
    pub tile_color1_b: f32,

    #[structopt(long = "tile-color2-r", default_value = "0.8")]
    #[serde(default = "default_tile_color2")]
    pub tile_color2_r: f32,
    #[structopt(long = "tile-color2-g", default_value = "0.8")]
    #[serde(default = "default_tile_color2")]
    pub tile_color2_g: f32,
    #[structopt(long = "tile-color2-b", default_value = "0.8")]
    #[serde(default = "default_tile_color2")]
    pub tile_color2_b: f32,

    #[structopt(long = "ground-height")]
    pub ground_height: Option<f32>,
}

fn default_web_server_port() -> u16 {
    7777
}
fn default_back_ground_color_b() -> f32 {
    0.3
}
fn default_tile_color1() -> f32 {
    0.1
}
fn default_tile_color2() -> f32 {
    0.8
}

impl Opt {
    #[cfg(target_arch = "wasm32")]
    pub fn from_params() -> Result<Self, crate::Error> {
        let href = web_sys::window().unwrap().location().href()?;
        log::debug!("href={}", href);
        let url = url::Url::parse(&href).map_err(|e| e.to_string())?;
        Ok(serde_qs::from_str(url.query().unwrap_or_default()).map_err(|e| e.to_string())?)
    }
}
