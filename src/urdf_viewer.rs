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
extern crate structopt;
#[macro_use]
extern crate structopt_derive;
extern crate urdf_rs;
extern crate urdf_viz;

use glfw::{Action, Key, WindowEvent};
use k::InverseKinematicsSolver;
use k::KinematicChain;
use k::JointContainer;
use k::urdf::FromUrdf;
use k::ChainContainer;
use std::path::Path;
use structopt::StructOpt;

#[cfg(target_os = "macos")]
static NATIVE_MOD: glfw::modifiers::Modifiers = glfw::modifiers::Super;

#[cfg(not(target_os = "macos"))]
static NATIVE_MOD: glfw::modifiers::Modifiers = glfw::modifiers::Control;

fn move_joint_by_random(robot: &mut k::LinkTree<f32>) -> Result<(), k::JointError> {
    let angles_vec = robot
        .iter_joints_link()
        .map(|link| match link.joint.limits {
            Some(ref range) => (range.max - range.min) * rand::random::<f32>() + range.min,
            None => (rand::random::<f32>() - 0.5) * 2.0,
        })
        .collect::<Vec<f32>>();
    robot.set_joint_angles(&angles_vec)
}

fn move_joint_by_index(
    index: usize,
    diff_angle: f32,
    robot: &mut k::LinkTree<f32>,
) -> Result<(), k::JointError> {
    let mut angles_vec = robot.get_joint_angles();
    assert!(index < robot.dof());
    angles_vec[index] += diff_angle;
    robot.set_joint_angles(&angles_vec)
}

struct LoopIndex {
    index: usize,
    size: usize,
}

impl LoopIndex {
    fn new(size: usize) -> Self {
        Self {
            index: 0,
            size: size,
        }
    }
    fn get(&self) -> usize {
        self.index
    }
    fn inc(&mut self) {
        self.index += 1;
        self.index %= self.size;
    }
    fn dec(&mut self) {
        if self.index == 0 {
            self.index = self.size - 1;
        } else {
            self.index -= 1;
        }
    }
}

const HOW_TO_USE_STR: &'static str = r"
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

struct UrdfViewerApp<'a> {
    urdf_robot: urdf_rs::Robot,
    robot: k::LinkTree<f32>,
    viewer: urdf_viz::Viewer,
    arms: Vec<k::LinkChain<f32>>,
    joint_names: Vec<String>,
    link_names: Vec<String>,
    num_arms: usize,
    num_joints: usize,
    index_of_arm: LoopIndex,
    index_of_move_joint: LoopIndex,
    base_dir: Option<&'a Path>,
}

impl<'a> UrdfViewerApp<'a> {
    fn new(
        urdf_robo: urdf_rs::Robot,
        base_dir: Option<&'a Path>,
        mut end_link_names: Vec<String>,
        is_collision: bool,
    ) -> Self {
        let robot = k::LinkTree::<f32>::from_urdf_robot(&urdf_robo);
        let mut viewer = urdf_viz::Viewer::new("urdf-viz");
        viewer.add_robot_with_base_dir_and_collision_flag(&urdf_robo, base_dir, is_collision);
        viewer.add_axis_cylinders("origin", 1.0);
        if end_link_names.is_empty() {
            end_link_names = robot
                .iter_joints()
                .filter(|node| node.borrow().children.is_empty())
                .map(|node| node.borrow().data.name.to_owned())
                .collect::<Vec<_>>();
        }
        let arms = end_link_names
            .iter()
            .filter_map(|end_name| robot.get_chain(&end_name))
            .collect::<Vec<_>>();
        let joint_names = robot.get_joint_names();
        let num_arms = arms.len();
        let dof = robot.dof();
        let link_names = robot
            .iter_joints_link()
            .map(|link| link.name.to_string())
            .collect();
        UrdfViewerApp {
            viewer,
            arms,
            link_names,
            urdf_robot: urdf_robo,
            base_dir,
            robot,
            num_arms,
            num_joints: joint_names.len(),
            joint_names,
            index_of_arm: LoopIndex::new(num_arms),
            index_of_move_joint: LoopIndex::new(dof),
        }
    }
    fn has_arms(&self) -> bool {
        self.num_arms > 0
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
    fn get_arm(&mut self) -> &mut k::LinkChain<f32> {
        &mut self.arms[self.index_of_arm.get()]
    }
    fn update_ik_target_marker(&mut self) {
        let trans = self.get_arm().calc_end_transform();
        if let Some(obj) = self.viewer.scenes.get_mut("ik_target") {
            obj.0.set_local_transformation(trans);
        }
    }
    fn update_robot(&mut self) {
        self.viewer.update(&self.robot);
        self.update_ik_target_marker();
    }
    fn run(&mut self) {
        let mut is_ctrl = false;
        let mut is_shift = false;
        let mut is_collision = false;
        let mut last_cur_pos_y = 0f64;
        let mut last_cur_pos_x = 0f64;
        let solver = k::JacobianIKSolverBuilder::new().finalize();

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
                        self.joint_names[self.index_of_move_joint.get()]
                    ),
                    60,
                    &na::Point2::new(10f32, 20.0),
                    &na::Point3::new(0.5f32, 0.5, 1.0),
                );
            }
            if self.has_arms() {
                let name = self.get_arm().name.to_owned();
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
                        if mods.contains(glfw::modifiers::Shift) {
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
                                let mut target = self.get_arm().calc_end_transform();
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
                                    let arm = self.get_arm();
                                    solver.solve(arm, &target).unwrap_or_else(|err| {
                                        println!("Err: {}", err);
                                        0.0f32
                                    });
                                }
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
                    WindowEvent::Key(code, _, Action::Press, _) => {
                        match code {
                            Key::LeftBracket => {
                                self.viewer.reset_temporal_color(
                                    &self.link_names[self.index_of_move_joint.get()],
                                );
                                self.index_of_move_joint.inc();
                                self.viewer.set_temporal_color(
                                    &self.link_names[self.index_of_move_joint.get()],
                                    1.0,
                                    0.0,
                                    0.0,
                                );
                            }
                            Key::RightBracket => {
                                self.viewer.reset_temporal_color(
                                    &self.link_names[self.index_of_move_joint.get()],
                                );
                                self.index_of_move_joint.dec();
                                self.viewer.set_temporal_color(
                                    &self.link_names[self.index_of_move_joint.get()],
                                    1.0,
                                    0.0,
                                    0.0,
                                );
                            }
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
                                is_collision = !is_collision;
                                self.viewer.add_robot_with_base_dir_and_collision_flag(
                                    &self.urdf_robot,
                                    self.base_dir,
                                    is_collision,
                                );
                                self.update_robot();
                            }
                            Key::R => {
                                if self.has_joints() {
                                    move_joint_by_random(&mut self.robot).unwrap_or(());
                                    self.update_robot();
                                }
                            }
                            Key::Up => {
                                if self.has_joints() {
                                    move_joint_by_index(
                                        self.index_of_move_joint.get(),
                                        0.1,
                                        &mut self.robot,
                                    ).unwrap_or(());
                                    self.update_robot();
                                }
                            }
                            Key::Down => {
                                if self.has_joints() {
                                    move_joint_by_index(
                                        self.index_of_move_joint.get(),
                                        0.1,
                                        &mut self.robot,
                                    ).unwrap_or(());
                                    self.update_robot();
                                }
                            }
                            _ => {}
                        };
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
    #[structopt(short = "c", long = "collision", help = "Show collision element instead of visual")]
    pub is_collision: bool,
}

fn main() {
    env_logger::init().unwrap();
    let opt = Opt::from_args();
    let input_path = Path::new(&opt.input_urdf_or_xacro);
    let base_dir = input_path.parent();
    let urdf_robo = urdf_rs::utils::read_urdf_or_xacro(input_path).unwrap();
    let mut app = UrdfViewerApp::new(urdf_robo, base_dir, opt.end_link_names, opt.is_collision);
    app.init();
    app.run();
}
