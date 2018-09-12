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

extern crate env_logger;
extern crate glfw;
extern crate k;
extern crate nalgebra as na;
extern crate rand;
#[macro_use]
extern crate structopt;
extern crate urdf_rs;
extern crate urdf_viz;

use glfw::{Action, Key, Modifiers, WindowEvent};
use k::prelude::*;
use std::path::PathBuf;
use structopt::StructOpt;

#[cfg(target_os = "macos")]
static NATIVE_MOD: Modifiers = glfw::Modifiers::Super;

#[cfg(not(target_os = "macos"))]
static NATIVE_MOD: Modifiers = glfw::Modifiers::Control;

fn move_joint_by_random(robot: &mut k::Chain<f32>) -> Result<(), k::JointError> {
    let angles_vec = robot
        .limits()
        .iter()
        .map(|limit| match limit {
            Some(ref range) => (range.max - range.min) * rand::random::<f32>() + range.min,
            None => (rand::random::<f32>() - 0.5) * 2.0,
        })
        .collect::<Vec<f32>>();
    robot.set_joint_positions(&angles_vec)
}

fn move_joint_by_index(
    index: usize,
    diff_angle: f32,
    robot: &mut k::Chain<f32>,
) -> Result<(), k::JointError> {
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

const HOW_TO_USE_STR: &str = r"
[:    joint ID +1
]:    joint ID -1
,:    IK target ID +1
.:    IK target ID -1
r:    set random angles
Up:   joint angle +0.1
Down: joint angle -0.1
Ctrl+Drag: move joint
Shift+Drag: IK (y, z)
Shift+Ctrl+Drag: IK (x, z)
c:    toggle visual/collision
";

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
}

impl UrdfViewerApp {
    fn new(
        input_file: &str,
        mut end_link_names: Vec<String>,
        is_collision: bool,
        disable_texture: bool,
        web_server_port: u16,
    ) -> Self {
        let input_path = PathBuf::from(input_file);
        let urdf_robo = urdf_rs::utils::read_urdf_or_xacro(&input_path).unwrap();
        let robot: k::Chain<f32> = (&urdf_robo).into();
        println!("{}", robot);
        let mut viewer = urdf_viz::Viewer::new("urdf-viz");
        if disable_texture {
            viewer.disable_texture();
        }
        viewer.add_robot_with_base_dir_and_collision_flag(
            &urdf_robo,
            input_path.parent(),
            is_collision,
        );
        viewer.add_axis_cylinders("origin", 1.0);
        let input_end_link_names = end_link_names.clone();
        if end_link_names.is_empty() {
            end_link_names = robot
                .iter()
                .filter(|node| node.borrow().children.is_empty())
                .map(|node| node.name())
                .collect::<Vec<_>>();
        }
        let arms = end_link_names
            .iter()
            .filter_map(|name| robot.find(name).map(|j| k::SerialChain::from_end(j)))
            .collect::<Vec<_>>();
        println!("end_link_names = {:?}", end_link_names);
        let names = robot.names();
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
        }
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
                .filter(|node| node.borrow().children.is_empty())
                .map(|node| node.name())
                .collect::<Vec<_>>()
        } else {
            self.input_end_link_names.clone()
        };
        self.arms = end_link_names
            .iter()
            .filter_map(|name| self.robot.find(name).map(|j| k::SerialChain::from_end(j)))
            .collect::<Vec<_>>();
        self.names = self.robot.names();
    }

    fn set_joint_positions_from_request(
        &mut self,
        joint_positions: &urdf_viz::JointNamesAndPositions,
    ) -> Result<(), k::JointError> {
        let mut angles = self.robot.joint_positions();
        for (name, angle) in joint_positions.names.iter().zip(joint_positions.positions.iter()) {
            if let Some(index) = self.names.iter().position(|ref n| *n == name) {
                angles[index] = *angle;
            } else {
                println!("{} not found, but continues", name);
            }
        }
        self.robot.set_joint_positions(&angles)
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
            Key::LeftBracket => self.increment_move_joint_index(true),
            Key::RightBracket => self.increment_move_joint_index(false),
            Key::Period => {
                self.index_of_arm.inc();
                self.update_ik_target_marker();
            }
            Key::Comma => {
                self.index_of_arm.dec();
                self.update_ik_target_marker();
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
            Key::R => if self.has_joints() {
                move_joint_by_random(&mut self.robot).unwrap_or(());
                self.update_robot();
            },
            Key::Up => if self.has_joints() {
                move_joint_by_index(self.index_of_move_joint.get(), 0.1, &mut self.robot)
                    .unwrap_or(());
                self.update_robot();
            },
            Key::Down => if self.has_joints() {
                move_joint_by_index(self.index_of_move_joint.get(), -0.1, &mut self.robot)
                    .unwrap_or(());
                self.update_robot();
            },
            _ => {}
        };
    }
    fn run(&mut self) {
        let mut is_ctrl = false;
        let mut is_shift = false;
        let mut last_cur_pos_y = 0f64;
        let mut last_cur_pos_x = 0f64;
        let solver = k::JacobianIKSolverBuilder::new().finalize();
        let web_server = urdf_viz::WebServer::new(self.web_server_port);
        let (target_joint_positions, current_joint_positions) = web_server.clone_in_out();
        if let Ok(mut cur_ja) = current_joint_positions.lock() {
            cur_ja.names = self.names.clone();
        }
        std::thread::spawn(move || web_server.start());

        while self.viewer.render() {
            self.viewer.draw_text(
                HOW_TO_USE_STR,
                40,
                &na::Point2::new(2000.0, 10.0),
                &na::Point3::new(1f32, 1.0, 1.0),
            );
            if self.has_joints() {
                self.viewer.draw_text(
                    &format!(
                        "moving joint name [{}]",
                        self.names[self.index_of_move_joint.get()]
                    ),
                    60,
                    &na::Point2::new(10f32, 20.0),
                    &na::Point3::new(0.5f32, 0.5, 1.0),
                );

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
            }
            if self.has_arms() {
                let name = self.get_arm().iter().last().unwrap().name();
                self.viewer.draw_text(
                    &format!("IK target name [{}]", name),
                    60,
                    &na::Point2::new(10f32, 100.0),
                    &na::Point3::new(0.5f32, 0.8, 0.2),
                );
            }
            if is_ctrl && !is_shift {
                self.viewer.draw_text(
                    "moving joint by drag",
                    60,
                    &na::Point2::new(10f32, 150.0),
                    &na::Point3::new(0.9f32, 0.5, 1.0),
                );
            }
            if is_shift {
                self.viewer.draw_text(
                    "solving ik",
                    60,
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
                        if mods.contains(glfw::Modifiers::Shift) {
                            is_shift = true;
                            event.inhibited = true;
                        }
                    }
                    WindowEvent::CursorPos(x, y) => {
                        if is_ctrl && !is_shift {
                            event.inhibited = true;
                            let move_gain = 0.005;
                            if self.has_joints() {
                                move_joint_by_index(
                                    self.index_of_move_joint.get(),
                                    (((x - last_cur_pos_x) + (y - last_cur_pos_y)) * move_gain)
                                        as f32,
                                    &mut self.robot,
                                ).unwrap_or(());
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
                                {
                                    solver.solve(&self.get_arm(), &target).unwrap_or_else(
                                        |err| {
                                            println!("Err: {}", err);
                                            0.0f32
                                        },
                                    );
                                }
                                self.update_robot();
                            }
                        }
                        last_cur_pos_x = x;
                        last_cur_pos_y = y;
                    }
                    WindowEvent::MouseButton(_, Action::Release, _) => if is_ctrl {
                        is_ctrl = false;
                        event.inhibited = true;
                    } else if is_shift {
                        is_shift = false;
                        event.inhibited = true;
                    },
                    WindowEvent::Key(code, _, Action::Press, _) => {
                        self.handle_key_press(code);
                        event.inhibited = true;
                    }
                    _ => {}
                }
            }
        }
    }
}

#[derive(StructOpt, Debug)]
#[structopt(name = "urdf_viz", about = "Option for visualizing urdf")]
pub struct Opt {
    #[structopt(help = "Input urdf or xacro")]
    pub input_urdf_or_xacro: String,
    #[structopt(short = "e", long = "end-link-name", help = "end link names")]
    pub end_link_names: Vec<String>,
    #[structopt(
        short = "c", long = "collision", help = "Show collision element instead of visual"
    )]
    pub is_collision: bool,
    #[structopt(short = "d", long = "disable-texture", help = "Disable texture rendering")]
    pub disable_texture: bool,
    #[structopt(
        short = "p",
        long = "web-server-port",
        help = "Port number for web server interface",
        default_value = "7777"
    )]
    pub web_server_port: u16,
}

fn main() {
    env_logger::init().unwrap();
    let opt = Opt::from_args();
    let mut app = UrdfViewerApp::new(
        &opt.input_urdf_or_xacro,
        opt.end_link_names,
        opt.is_collision,
        opt.disable_texture,
        opt.web_server_port,
    );
    app.init();
    app.run();
}
