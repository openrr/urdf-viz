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

use k::prelude::*;
use kiss3d::event::{Action, Key, Modifiers, WindowEvent};
use nalgebra as na;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
use structopt::StructOpt;

#[cfg(target_os = "macos")]
static NATIVE_MOD: Modifiers = kiss3d::event::Modifiers::Super;

#[cfg(not(target_os = "macos"))]
static NATIVE_MOD: Modifiers = kiss3d::event::Modifiers::Control;

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

struct UrdfViewerApp {
    input_path: PathBuf,
    urdf_robot: urdf_rs::Robot,
    robot: k::Chain<f32>,
    viewer: urdf_viz::Viewer,
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
    fn new(
        input_file: &str,
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
        let urdf_robo = urdf_rs::utils::read_urdf_or_xacro(&input_path).unwrap();
        let robot: k::Chain<f32> = (&urdf_robo).into();
        println!("{}", robot);
        let mut viewer = urdf_viz::Viewer::with_background_color("urdf-viz", background_color);
        if disable_texture {
            viewer.disable_texture();
        }
        viewer.add_robot_with_base_dir_and_collision_flag(
            &urdf_robo,
            input_path.parent(),
            is_collision,
        );
        viewer.add_axis_cylinders("origin", 1.0);
        if let Some(h) = ground_height {
            viewer.add_ground(h, 0.5, 3, tile_color1, tile_color2);
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
        UrdfViewerApp {
            input_path,
            viewer,
            arms,
            input_end_link_names,
            urdf_robot: urdf_robo,
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
    fn init(&mut self) {
        self.update_robot();
        if self.has_arms() {
            self.viewer.add_axis_cylinders("ik_target", 0.2);
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
    fn reload_urdf(&mut self) {
        self.viewer.remove_robot(&self.urdf_robot);
        self.urdf_robot = urdf_rs::utils::read_urdf_or_xacro(&self.input_path).unwrap();
        self.robot = (&self.urdf_robot).into();
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

    /// Handle set_joint_positions request from web server
    fn set_joint_positions_from_request(
        &mut self,
        joint_positions: &urdf_viz::JointNamesAndPositions,
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
    fn set_robot_origin_from_request(
        &mut self,
        origin: &urdf_viz::RobotOrigin,
    ) -> Result<(), k::Error> {
        let pos = origin.position;
        let q = origin.quaternion;
        let pose = na::Isometry3::from_parts(
            na::Translation3::new(pos[0], pos[1], pos[2]),
            na::UnitQuaternion::new_normalize(na::Quaternion::new(q[0], q[1], q[2], q[3])),
        );
        self.robot.set_origin(pose);
        Ok(())
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
    fn handle_key_press(&mut self, code: Key) {
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
                self.viewer.remove_robot(&self.urdf_robot);
                self.is_collision = !self.is_collision;
                self.viewer.add_robot_with_base_dir_and_collision_flag(
                    &self.urdf_robot,
                    self.input_path.parent(),
                    self.is_collision,
                );
                self.update_robot();
            }
            Key::L => {
                // reload
                self.reload_urdf();
                self.viewer.add_robot_with_base_dir_and_collision_flag(
                    &self.urdf_robot,
                    self.input_path.parent(),
                    self.is_collision,
                );
                self.update_robot();
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
    fn run(&mut self) {
        let mut is_ctrl = false;
        let mut is_shift = false;
        let mut last_cur_pos_y = 0f64;
        let mut last_cur_pos_x = 0f64;
        let solver = k::JacobianIkSolver::default();
        let web_server = urdf_viz::WebServer::new(self.web_server_port);
        let (
            target_joint_positions,
            current_joint_positions,
            target_robot_origin,
            current_robot_origin,
        ) = web_server.clone_in_out();
        if let Ok(mut cur_ja) = current_joint_positions.lock() {
            cur_ja.names = self.names.clone();
        }

        std::thread::spawn(move || web_server.start());

        static ABORTED: AtomicBool = AtomicBool::new(false);
        ctrlc::set_handler(|| {
            ABORTED.store(true, Relaxed);
        })
        .unwrap();

        const FONT_SIZE_USAGE: f32 = 60.0;
        const FONT_SIZE_INFO: f32 = 80.0;
        while self.viewer.render() {
            if ABORTED.load(Relaxed) {
                break;
            }
            self.viewer.draw_text(
                HOW_TO_USE_STR,
                FONT_SIZE_USAGE,
                &na::Point2::new(2000.0, 10.0),
                &na::Point3::new(1f32, 1.0, 1.0),
            );
            if self.has_joints() {
                self.viewer.draw_text(
                    &format!(
                        "moving joint name [{}]",
                        self.names[self.index_of_move_joint.get()]
                    ),
                    FONT_SIZE_INFO,
                    &na::Point2::new(10f32, 20.0),
                    &na::Point3::new(0.5f32, 0.5, 1.0),
                );

                // Joint positions for web server
                if let Ok(mut ja) = target_joint_positions.lock() {
                    if ja.requested {
                        match self.set_joint_positions_from_request(&ja.joint_positions) {
                            Ok(_) => {
                                self.update_robot();
                                ja.requested = false;
                            }
                            Err(err) => {
                                println!("{}", err);
                            }
                        }
                    }
                }
                if let Ok(mut cur_ja) = current_joint_positions.lock() {
                    cur_ja.positions = self.robot.joint_positions();
                }

                // Robot orientation for web server
                if let Ok(mut ro) = target_robot_origin.lock() {
                    if ro.requested {
                        match self.set_robot_origin_from_request(&ro.origin) {
                            Ok(_) => {
                                self.update_robot();
                                ro.requested = false;
                            }
                            Err(err) => {
                                println!("{}", err);
                            }
                        }
                    }
                }
                if let Ok(mut cur_ro) = current_robot_origin.lock() {
                    let o = self.robot.origin();
                    for i in 0..3 {
                        cur_ro.position[i] = o.translation.vector[i];
                    }
                    cur_ro.quaternion[0] = o.rotation.quaternion().w;
                    cur_ro.quaternion[1] = o.rotation.quaternion().i;
                    cur_ro.quaternion[2] = o.rotation.quaternion().j;
                    cur_ro.quaternion[3] = o.rotation.quaternion().k;
                }
            }
            if self.has_arms() {
                let name = &self
                    .get_arm()
                    .iter()
                    .last()
                    .unwrap()
                    .joint()
                    .name
                    .to_owned();
                self.viewer.draw_text(
                    &format!("IK target name [{}]", name),
                    FONT_SIZE_INFO,
                    &na::Point2::new(10f32, 100.0),
                    &na::Point3::new(0.5f32, 0.8, 0.2),
                );
            }
            if is_ctrl && !is_shift {
                self.viewer.draw_text(
                    "moving joint by drag",
                    FONT_SIZE_INFO,
                    &na::Point2::new(10f32, 150.0),
                    &na::Point3::new(0.9f32, 0.5, 1.0),
                );
            }
            if is_shift {
                self.viewer.draw_text(
                    "solving ik",
                    FONT_SIZE_INFO,
                    &na::Point2::new(10f32, 150.0),
                    &na::Point3::new(0.9f32, 0.5, 1.0),
                );
            }
            for mut event in self.viewer.events().iter() {
                match event.value {
                    WindowEvent::MouseButton(_, Action::Press, mods) => {
                        if mods.contains(NATIVE_MOD) {
                            is_ctrl = true;
                            event.inhibited = true;
                        }
                        if mods.contains(kiss3d::event::Modifiers::Shift) {
                            is_shift = true;
                            event.inhibited = true;
                        }
                    }
                    WindowEvent::CursorPos(x, y, _modifiers) => {
                        if is_ctrl && !is_shift {
                            event.inhibited = true;
                            let move_gain = 0.005;
                            if self.has_joints() {
                                move_joint_by_index(
                                    self.index_of_move_joint.get(),
                                    (((x - last_cur_pos_x) + (y - last_cur_pos_y)) * move_gain)
                                        as f32,
                                    &mut self.robot,
                                )
                                .unwrap_or(());
                                self.update_robot();
                            }
                        }
                        if is_shift {
                            event.inhibited = true;
                            if self.has_arms() {
                                self.robot.update_transforms();
                                let mut target = self.get_end_transform();
                                let ik_move_gain = 0.002;
                                target.translation.vector[2] -=
                                    ((y - last_cur_pos_y) * ik_move_gain) as f32;
                                if is_ctrl {
                                    target.translation.vector[0] +=
                                        ((x - last_cur_pos_x) * ik_move_gain) as f32;
                                } else {
                                    target.translation.vector[1] +=
                                        ((x - last_cur_pos_x) * ik_move_gain) as f32;
                                }

                                self.update_ik_target_marker();
                                let orig_angles = self.robot.joint_positions();
                                solver
                                    .solve_with_constraints(
                                        &self.get_arm(),
                                        &target,
                                        &self.ik_constraints,
                                    )
                                    .unwrap_or_else(|err| {
                                        self.robot.set_joint_positions_unchecked(&orig_angles);
                                        println!("Err: {}", err);
                                    });
                                self.update_robot();
                            }
                        }
                        last_cur_pos_x = x;
                        last_cur_pos_y = y;
                    }
                    WindowEvent::MouseButton(_, Action::Release, _) => {
                        if is_ctrl {
                            is_ctrl = false;
                            event.inhibited = true;
                        } else if is_shift {
                            is_shift = false;
                            event.inhibited = true;
                        }
                    }
                    WindowEvent::Key(code, Action::Press, _modifiers) => {
                        self.handle_key_press(code);
                        event.inhibited = true;
                    }
                    _ => {}
                }
            }
        }
    }
}

/// Option for visualizing urdf
#[derive(StructOpt, Debug)]
#[structopt(name = "urdf_viz")]
pub struct Opt {
    /// Input urdf or xacro
    pub input_urdf_or_xacro: String,
    /// end link names
    #[structopt(short = "e", long = "end-link-name")]
    pub end_link_names: Vec<String>,
    /// Show collision element instead of visual
    #[structopt(short = "c", long = "collision")]
    pub is_collision: bool,
    /// Disable texture rendering
    #[structopt(short = "d", long = "disable-texture")]
    pub disable_texture: bool,
    /// Port number for web server interface
    #[structopt(short = "p", long = "web-server-port", default_value = "7777")]
    pub web_server_port: u16,
    #[structopt(long = "ignore-ik-position-x")]
    pub ignore_ik_position_x: bool,
    #[structopt(long = "ignore-ik-position-y")]
    pub ignore_ik_position_y: bool,
    #[structopt(long = "ignore-ik-position-z")]
    pub ignore_ik_position_z: bool,
    #[structopt(long = "ignore-ik-rotation-x")]
    pub ignore_ik_rotation_x: bool,
    #[structopt(long = "ignore-ik-rotation-y")]
    pub ignore_ik_rotation_y: bool,
    #[structopt(long = "ignore-ik-rotation-z")]
    pub ignore_ik_rotation_z: bool,

    #[structopt(long = "bg-color-r", default_value = "0.0")]
    pub back_ground_color_r: f32,
    #[structopt(long = "bg-color-g", default_value = "0.0")]
    pub back_ground_color_g: f32,
    #[structopt(long = "bg-color-b", default_value = "0.3")]
    pub back_ground_color_b: f32,

    #[structopt(long = "tile-color1-r", default_value = "0.1")]
    pub tile_color1_r: f32,
    #[structopt(long = "tile-color1-g", default_value = "0.1")]
    pub tile_color1_g: f32,
    #[structopt(long = "tile-color1-b", default_value = "0.1")]
    pub tile_color1_b: f32,

    #[structopt(long = "tile-color2-r", default_value = "0.8")]
    pub tile_color2_r: f32,
    #[structopt(long = "tile-color2-g", default_value = "0.8")]
    pub tile_color2_g: f32,
    #[structopt(long = "tile-color2-b", default_value = "0.8")]
    pub tile_color2_b: f32,

    #[structopt(long = "ground-height")]
    pub ground_height: Option<f32>,
}

fn main() {
    env_logger::init();
    let opt = Opt::from_args();
    let mut app = UrdfViewerApp::new(
        &opt.input_urdf_or_xacro,
        opt.end_link_names,
        opt.is_collision,
        opt.disable_texture,
        opt.web_server_port,
        (
            opt.back_ground_color_r,
            opt.back_ground_color_g,
            opt.back_ground_color_b,
        ),
        (opt.tile_color1_r, opt.tile_color1_g, opt.tile_color1_b),
        (opt.tile_color2_r, opt.tile_color2_g, opt.tile_color2_b),
        opt.ground_height,
    );
    let mut ik_constraints = k::Constraints::default();
    ik_constraints.position_x = !opt.ignore_ik_position_x;
    ik_constraints.position_y = !opt.ignore_ik_position_y;
    ik_constraints.position_z = !opt.ignore_ik_position_z;
    ik_constraints.rotation_x = !opt.ignore_ik_rotation_x;
    ik_constraints.rotation_y = !opt.ignore_ik_rotation_y;
    ik_constraints.rotation_z = !opt.ignore_ik_rotation_z;
    app.set_ik_constraints(ik_constraints);
    app.init();
    app.run();
}
