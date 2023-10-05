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

use k::nalgebra as na;
use k::prelude::*;
use kiss3d::event::{Action, Key, Modifiers, WindowEvent};
use kiss3d::window::{self, Window};
use serde::Deserialize;
use std::collections::HashMap;
use std::fmt;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
use std::sync::Arc;
use structopt::StructOpt;
use tracing::*;

#[cfg(not(target_family = "wasm"))]
use std::sync::atomic::AtomicBool;

use crate::{
    handle::{JointNamesAndPositions, RobotOrigin, RobotStateHandle},
    point_cloud::PointCloudRenderer,
    utils::RobotModel,
    Error, Viewer, ROBOT_OBJECT_ID,
};

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

#[derive(Debug)]
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
f:    toggle show link frames
m:    toggle show menu
"#;

const FRAME_ARROW_SIZE: f32 = 0.2;

fn node_to_frame_name(node: &k::Node<f32>) -> String {
    format!("{}_frame", node.joint().name)
}

pub struct UrdfViewerApp {
    input_path: PathBuf,
    urdf_robot: RobotModel,
    robot: k::Chain<f32>,
    viewer: Viewer,
    window: Option<Window>,
    arms: Vec<k::SerialChain<f32>>,
    names: Vec<String>,
    input_end_link_names: Vec<String>,
    index_of_arm: LoopIndex,
    index_of_move_joint: LoopIndex,
    handle: Arc<RobotStateHandle>,
    is_collision: bool,
    show_frames: bool,
    ik_constraints: k::Constraints,
    point_size: f32,
    package_path: HashMap<String, String>,
    hide_menu: bool,
    axis_scale: f32,
    move_base_diff_unit: f32,
    move_joint_diff_unit: f32,
}

impl UrdfViewerApp {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mut urdf_robot: RobotModel,
        mut end_link_names: Vec<String>,
        is_collision: bool,
        disable_texture: bool,
        disable_assimp: bool,
        background_color: (f32, f32, f32),
        tile_color1: (f32, f32, f32),
        tile_color2: (f32, f32, f32),
        ground_height: Option<f32>,
        hide_menu: bool,
        axis_scale: f32,
        move_base_diff_unit: f32,
        move_joint_diff_unit: f32,
    ) -> Result<Self, Error> {
        let input_path = PathBuf::from(&urdf_robot.path);
        let package_path = urdf_robot.take_package_path_map();
        let robot: k::Chain<f32> = urdf_robot.get().into();
        println!("{robot}");
        let (mut viewer, mut window) = Viewer::with_background_color("urdf-viz", background_color);
        if disable_texture {
            viewer.disable_texture();
        }
        if disable_assimp {
            viewer.disable_assimp();
        }
        viewer.add_robot_with_base_dir_and_collision_flag(
            &mut window,
            urdf_robot.get(),
            input_path.parent(),
            is_collision,
            &package_path,
        );
        viewer.add_axis_cylinders_with_scale(&mut window, "origin", 1.0, axis_scale);
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
            .filter_map(|name| robot.find(name).map(k::SerialChain::from_end))
            .collect::<Vec<_>>();
        println!("end_link_names = {end_link_names:?}");
        let names = robot
            .iter_joints()
            .map(|j| j.name.clone())
            .collect::<Vec<_>>();
        let num_arms = end_link_names.len();
        let num_joints = names.len();
        println!("DoF={num_joints}");
        println!("joint names={names:?}");
        let mut handle = RobotStateHandle::default();
        handle.current_joint_positions.lock().names = names.clone();
        handle.urdf_text = Some(urdf_robot.urdf_text.clone());
        Ok(UrdfViewerApp {
            input_path,
            viewer,
            window: Some(window),
            arms,
            input_end_link_names,
            urdf_robot,
            robot,
            names,
            index_of_arm: LoopIndex::new(num_arms),
            index_of_move_joint: LoopIndex::new(num_joints),
            handle: Arc::new(handle),
            is_collision,
            show_frames: false,
            ik_constraints: k::Constraints::default(),
            point_size: 10.0,
            package_path,
            hide_menu,
            axis_scale,
            move_base_diff_unit,
            move_joint_diff_unit,
        })
    }
    pub fn handle(&self) -> Arc<RobotStateHandle> {
        self.handle.clone()
    }
    pub fn set_ik_constraints(&mut self, ik_constraints: k::Constraints) {
        self.ik_constraints = ik_constraints;
    }
    pub fn set_point_size(&mut self, point_size: f32) {
        self.point_size = point_size;
    }
    fn has_arms(&self) -> bool {
        !self.arms.is_empty()
    }
    fn has_joints(&self) -> bool {
        !self.names.is_empty()
    }
    pub fn init(&mut self) {
        self.update_robot();
        if self.has_arms() {
            let window = self.window.as_mut().unwrap();
            self.viewer
                .add_axis_cylinders_with_scale(window, "ik_target", 0.2, self.axis_scale);
            self.update_ik_target_marker();
        }
        let window = self.window.as_mut().unwrap();
        self.robot.iter().for_each(|n| {
            self.viewer.add_axis_cylinders_with_scale(
                window,
                &node_to_frame_name(n),
                FRAME_ARROW_SIZE,
                self.axis_scale,
            );
        });
        self.update_frame_markers();
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
    fn update_frame_markers(&mut self) {
        for n in self.robot.iter() {
            let name = node_to_frame_name(n);
            if let Some(obj) = self.viewer.scene_node_mut(&name) {
                if self.show_frames {
                    obj.set_local_transformation(n.world_transform().unwrap());
                }
                obj.set_visible(self.show_frames);
            }
        }
    }
    fn remove_frame_markers(&mut self, window: &mut Window) {
        for n in self.robot.iter() {
            let name = node_to_frame_name(n);
            if let Some(obj) = self.viewer.scene_node_mut(&name) {
                window.remove_node(obj);
            }
        }
    }

    fn update_robot(&mut self) {
        // this is hack to handle invalid mimic joints, like hsr
        let joint_positions = self.robot.joint_positions();
        self.robot
            .set_joint_positions(&joint_positions)
            .unwrap_or_else(|err| error!("failed to update robot joints {err}"));
        self.viewer.update(&self.robot);
        self.update_ik_target_marker();
        self.update_frame_markers();
    }
    fn reload(&mut self, window: &mut Window, reload_fn: impl FnOnce(&mut RobotModel)) {
        // remove previous robot
        self.viewer.remove_robot(window, self.urdf_robot.get());
        self.remove_frame_markers(window);
        // update urdf_robot
        reload_fn(&mut self.urdf_robot);

        // update robot based on new urdf_robot
        let urdf_robot = self.urdf_robot.get();
        self.robot = urdf_robot.into();
        self.input_path = PathBuf::from(&self.urdf_robot.path);
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
            .filter_map(|name| self.robot.find(name).map(k::SerialChain::from_end))
            .collect::<Vec<_>>();
        let names: Vec<_> = self.robot.iter_joints().map(|j| j.name.clone()).collect();
        if self.names != names {
            let dof = names.len();
            self.names = names.clone();
            let mut current_joint_positions = self.handle.current_joint_positions.lock();
            current_joint_positions.names = names;
            current_joint_positions.positions = vec![0.0; dof];
        }

        self.viewer.add_robot_with_base_dir_and_collision_flag(
            window,
            self.urdf_robot.get(),
            self.input_path.parent(),
            self.is_collision,
            &self.package_path,
        );
        const FRAME_ARROW_SIZE: f32 = 0.2;
        self.robot.iter().for_each(|n| {
            self.viewer.add_axis_cylinders_with_scale(
                window,
                &node_to_frame_name(n),
                FRAME_ARROW_SIZE,
                self.axis_scale,
            );
        });
        self.update_robot();
    }

    /// Handle set_joint_positions request from server
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
            if let Some(index) = self.names.iter().position(|n| n == name) {
                angles[index] = *angle;
            } else {
                warn!("{name} not found, but continues");
            }
        }
        self.robot.set_joint_positions(&angles)
    }

    /// Handle set_origin request from server
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
    fn handle_key_press(&mut self, window: &mut Window, code: Key, modifiers: Modifiers) {
        let is_ctrl = modifiers.contains(NATIVE_MOD);
        if is_ctrl {
            // Heuristic for avoiding conflicts with browser keyboard shortcuts.
            return;
        }
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
                origin.translation.vector[1] += self.move_base_diff_unit;
                self.robot.set_origin(origin);
                self.update_robot();
            }
            Key::S => {
                let mut origin = self.robot.origin();
                origin.translation.vector[0] -= self.move_base_diff_unit;
                self.robot.set_origin(origin);
                self.update_robot();
            }
            Key::D => {
                let mut origin = self.robot.origin();
                origin.translation.vector[1] -= self.move_base_diff_unit;
                self.robot.set_origin(origin);
                self.update_robot();
            }
            Key::W => {
                let mut origin = self.robot.origin();
                origin.translation.vector[0] += self.move_base_diff_unit;
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
                    &self.package_path,
                );
                self.update_robot();
            }
            Key::F => {
                self.show_frames = !self.show_frames;
                self.update_frame_markers();
            }
            #[cfg(not(target_family = "wasm"))]
            Key::L => {
                // reload
                self.reload(window, |urdf_robot| urdf_robot.reload());
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
                    move_joint_by_index(
                        self.index_of_move_joint.get(),
                        self.move_joint_diff_unit,
                        &mut self.robot,
                    )
                    .unwrap_or(());
                    self.update_robot();
                }
            }
            Key::Down => {
                if self.has_joints() {
                    move_joint_by_index(
                        self.index_of_move_joint.get(),
                        -self.move_joint_diff_unit,
                        &mut self.robot,
                    )
                    .unwrap_or(());
                    self.update_robot();
                }
            }
            Key::M => {
                self.hide_menu = !self.hide_menu;
            }
            _ => {}
        };
    }
    pub fn run(mut self) {
        #[cfg(not(target_family = "wasm"))]
        ctrlc::set_handler(|| {
            ABORTED.store(true, Relaxed);
        })
        .unwrap();

        let window = self.window.take().unwrap();
        let state = AppState {
            point_cloud_renderer: PointCloudRenderer::new(self.point_size),
            app: self,
            solver: k::JacobianIkSolver::default(),
            is_ctrl: false,
            is_shift: false,
            last_cur_pos_x: 0.0,
            last_cur_pos_y: 0.0,
        };
        window.render_loop(state);
    }
}

impl fmt::Debug for UrdfViewerApp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // kiss3d::window::Window doesn't implement Debug.
        f.debug_struct("UrdfViewerApp")
            .field("input_path", &self.input_path)
            .field("urdf_robot", &self.urdf_robot)
            .field("robot", &self.robot)
            .field("viewer", &self.viewer)
            .field("arms", &self.arms)
            .field("names", &self.names)
            .field("input_end_link_names", &self.input_end_link_names)
            .field("index_of_arm", &self.index_of_arm)
            .field("index_of_move_joint", &self.index_of_move_joint)
            .field("handle", &self.handle)
            .field("is_collision", &self.is_collision)
            .field("ik_constraints", &self.ik_constraints)
            .finish()
    }
}

#[cfg(not(target_family = "wasm"))]
static ABORTED: AtomicBool = AtomicBool::new(false);

struct AppState {
    app: UrdfViewerApp,
    solver: k::JacobianIkSolver<f32>,
    is_ctrl: bool,
    is_shift: bool,
    last_cur_pos_y: f64,
    last_cur_pos_x: f64,
    point_cloud_renderer: PointCloudRenderer,
}

impl AppState {
    fn handle_request(&mut self, window: &mut Window) {
        let handle = self.app.handle();

        if let Some(robot) = handle.take_robot() {
            self.app.reload(window, |urdf_robot| *urdf_robot = robot);
        }

        #[cfg(not(target_family = "wasm"))]
        if handle.take_reload_request() {
            self.app.reload(window, |urdf_robot| urdf_robot.reload());
        }

        while let Some(points) = handle.point_cloud.pop() {
            if points.points.len() == points.colors.len() {
                self.point_cloud_renderer
                    .insert(points.id, &points.points, &points.colors);
            } else {
                error!(
                    "point cloud length mismatch points={}, colors={}",
                    points.points.len(),
                    points.colors.len()
                );
            }
        }

        while let Some(cube) = handle.cube.pop() {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let id = &if let Some(extent) = cube.extent {
                let id = cube
                    .id
                    .unwrap_or_else(|| format!("__cube{}", COUNTER.fetch_add(1, Relaxed)));
                if let Some(scene) = self.app.viewer.scene_node_mut(&id) {
                    // remove pre-existent node
                    scene.unlink();
                }
                self.app
                    .viewer
                    .add_cube(window, &id, extent[0], extent[1], extent[2]);
                id
            } else if let Some(id) = cube.id {
                id
            } else {
                error!("must be at least one of cube.extent or cube.id specified");
                break; // jump
            };
            let scene = self.app.viewer.scene_node_mut(id).unwrap();
            if let Some(color) = cube.color {
                scene.set_color(color[0], color[1], color[2]);
            }
            if let Some(p) = cube.position {
                scene.set_local_translation(na::Translation3::new(p[0], p[1], p[2]));
            }
            if let Some(q) = cube.quaternion {
                scene.set_local_rotation(na::UnitQuaternion::new_normalize(na::Quaternion::new(
                    q[0], q[1], q[2], q[3],
                )));
            }
        }

        while let Some(capsule) = handle.capsule.pop() {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let id = &capsule
                .id
                .unwrap_or_else(|| format!("__capsule{}", COUNTER.fetch_add(1, Relaxed)));
            if let Some(scene) = self.app.viewer.scene_node_mut(id) {
                // remove pre-existent node
                scene.unlink();
            }
            self.app
                .viewer
                .add_capsule(window, id, capsule.radius, capsule.height);
            let scene = self.app.viewer.scene_node_mut(id).unwrap();
            if let Some(color) = capsule.color {
                scene.set_color(color[0], color[1], color[2]);
            }
            if let Some(p) = capsule.position {
                scene.set_local_translation(na::Translation3::new(p[0], p[1], p[2]));
            }
            if let Some(q) = capsule.quaternion {
                scene.set_local_rotation(na::UnitQuaternion::new_normalize(na::Quaternion::new(
                    q[0], q[1], q[2], q[3],
                )));
            }
        }

        while let Some(axis_marker) = handle.axis_marker.pop() {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let id = &axis_marker
                .id
                .unwrap_or_else(|| format!("__axis_marker{}", COUNTER.fetch_add(1, Relaxed)));
            if let Some(scene) = self.app.viewer.scene_node_mut(id) {
                // remove pre-existent node
                scene.unlink();
            }
            self.app.viewer.add_axis_cylinders_with_scale(
                window,
                id,
                axis_marker.size,
                self.app.axis_scale,
            );
            let scene = self.app.viewer.scene_node_mut(id).unwrap();
            if let Some(p) = axis_marker.position {
                scene.set_local_translation(na::Translation3::new(p[0], p[1], p[2]));
            }
            if let Some(q) = axis_marker.quaternion {
                scene.set_local_rotation(na::UnitQuaternion::new_normalize(na::Quaternion::new(
                    q[0], q[1], q[2], q[3],
                )));
            }
        }

        while let Some(relationship) = handle.relationship.pop() {
            if relationship.parent != relationship.child {
                if let (Some(mut parent), Some(mut child)) = (
                    self.app.viewer.scene_node(&relationship.parent).cloned(),
                    self.app.viewer.scene_node(&relationship.child).cloned(),
                ) {
                    let p = relationship.position;
                    let t = parent.data().local_translation();
                    child.set_local_translation(na::Translation3::new(
                        t.vector[0] + p[0],
                        t.vector[1] + p[1],
                        t.vector[2] + p[2],
                    ));
                    let q = relationship.quaternion;
                    let r = parent.data().local_rotation();
                    child.set_local_rotation(na::UnitQuaternion::new_normalize(
                        na::Quaternion::new(r.w + q[0], r.i + q[1], r.j + q[2], r.k + q[3]),
                    ));
                    child.unlink();
                    parent.add_child(child);
                }
            }
        }

        if self.app.has_joints() {
            // Joint positions for server
            if let Some(ja) = handle.take_target_joint_positions() {
                match self.app.set_joint_positions_from_request(&ja) {
                    Ok(_) => {
                        self.app.update_robot();
                    }
                    Err(err) => {
                        error!("{err}");
                    }
                }
            }
            handle.current_joint_positions.lock().positions = self.app.robot.joint_positions();
        }

        // Robot orientation for server
        while let Some(origin) = handle.pop_target_object_origin() {
            if origin.id == ROBOT_OBJECT_ID {
                self.app.set_robot_origin_from_request(&RobotOrigin {
                    position: origin.position,
                    quaternion: origin.quaternion,
                });
                self.app.update_robot();
            } else if let Some(scene) = self.app.viewer.scene_node_mut(&origin.id) {
                let p = origin.position;
                scene.set_local_translation(na::Translation3::new(p[0], p[1], p[2]));
                let q = origin.quaternion;
                scene.set_local_rotation(na::UnitQuaternion::new_normalize(na::Quaternion::new(
                    q[0], q[1], q[2], q[3],
                )));
            } else {
                error!("object '{}' not found", origin.id);
            }
        }
        let mut cur_origin = handle.current_robot_origin.lock();
        let origin = self.app.robot.origin();
        for i in 0..3 {
            cur_origin.position[i] = origin.translation.vector[i];
        }
        cur_origin.quaternion[0] = origin.rotation.quaternion().w;
        cur_origin.quaternion[1] = origin.rotation.quaternion().i;
        cur_origin.quaternion[2] = origin.rotation.quaternion().j;
        cur_origin.quaternion[3] = origin.rotation.quaternion().k;
    }
}

impl window::State for AppState {
    fn step(&mut self, window: &mut Window) {
        const FONT_SIZE_USAGE: f32 = 60.0;
        const FONT_SIZE_INFO: f32 = 80.0;

        #[cfg(not(target_family = "wasm"))]
        if ABORTED.load(Relaxed) {
            window.close();
            return;
        }

        if !self.app.hide_menu {
            self.app.viewer.draw_text(
                window,
                HOW_TO_USE_STR,
                FONT_SIZE_USAGE,
                &na::Point2::new(2000.0, 10.0),
                &na::Point3::new(1f32, 1.0, 1.0),
            );
        }
        self.handle_request(window);
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
                &format!("IK target name [{name}]"),
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
            match event.value {
                WindowEvent::MouseButton(_, Action::Press, modifiers) => {
                    if modifiers.contains(NATIVE_MOD) {
                        self.is_ctrl = true;
                        event.inhibited = true;
                    }
                    if modifiers.contains(Modifiers::Shift) {
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
                                    self.app.get_arm(),
                                    &target,
                                    &self.app.ik_constraints,
                                )
                                .unwrap_or_else(|err| {
                                    self.app.robot.set_joint_positions_unchecked(&orig_angles);
                                    error!("{err}");
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
                WindowEvent::Key(code, Action::Press, modifiers) => {
                    self.app.handle_key_press(window, code, modifiers);
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
        (
            Some(&mut self.app.viewer.arc_ball),
            None,
            Some(&mut self.point_cloud_renderer),
            None,
        )
    }
}

/// Option for visualizing urdf
#[derive(StructOpt, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
#[non_exhaustive]
pub struct Opt {
    /// Input urdf or xacro
    #[serde(default, rename = "urdf")]
    pub input_urdf_or_xacro: String,
    /// Xacro arguments
    #[structopt(long = "xacro-args", value_name = "ARGUMENT=VALUE", parse(try_from_str = parse_xacro_argument))]
    #[serde(default)]
    pub xacro_arguments: Vec<(String, String)>,
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
    #[cfg(feature = "assimp")]
    /// Disable using assimp for loading mesh
    #[structopt(long = "disable-assimp")]
    #[serde(default)]
    pub disable_assimp: bool,
    /// Port number for web server interface (default to 7777)
    #[structopt(short = "p", long = "web-server-port")]
    pub web_server_port: Option<u16>,

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

    /// Replace `package://PACKAGE` in mesh tag with PATH.
    #[structopt(long = "package-path", value_name = "PACKAGE=PATH")]
    #[serde(default)]
    pub package_path: Vec<String>,

    /// Hide the menu by default.
    #[structopt(short = "m", long = "hide-menu")]
    #[serde(default)]
    pub hide_menu: bool,

    #[structopt(short = "s", long = "axis-scale", default_value = "1.0")]
    #[serde(default = "default_axis_scale")]
    pub axis_scale: f32,

    #[structopt(short = "b", long = "move-base-diff-unit", default_value = "0.1")]
    #[serde(default = "default_diff_unit")]
    pub move_base_diff_unit: f32,

    #[structopt(short = "j", long = "move-joint-diff-unit", default_value = "0.1")]
    #[serde(default = "default_diff_unit")]
    pub move_joint_diff_unit: f32,
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
fn default_axis_scale() -> f32 {
    1.0
}
fn default_diff_unit() -> f32 {
    0.1
}

fn parse_xacro_argument(xacro_argument: &str) -> Result<(String, String), Error> {
    xacro_argument
        .split_once('=')
        .ok_or_else(|| {
            Error::Other(format!(
                "Invalid xacro argument key=value: no `=` found in {xacro_argument}"
            ))
        })
        .map(|s| (s.0.to_owned(), s.1.to_owned()))
}

impl Opt {
    pub fn create_ik_constraints(&self) -> k::Constraints {
        k::Constraints {
            position_x: !self.ignore_ik_position_x,
            position_y: !self.ignore_ik_position_y,
            position_z: !self.ignore_ik_position_z,
            rotation_x: !self.ignore_ik_rotation_x,
            rotation_y: !self.ignore_ik_rotation_y,
            rotation_z: !self.ignore_ik_rotation_z,
            ..Default::default()
        }
    }

    pub fn create_package_path_map(&self) -> Result<HashMap<String, String>, Error> {
        let mut map = HashMap::with_capacity(self.package_path.len());
        for replace in &self.package_path {
            let (package_name, path) = replace
                .split_once('=')
                .filter(|(k, _)| !k.is_empty())
                .ok_or_else(|| {
                    format!(
                        "--package-path may only accept PACKAGE=PATH format, but found '{replace}'"
                    )
                })?;
            map.insert(package_name.to_owned(), path.to_owned());
        }
        Ok(map)
    }

    #[cfg(target_family = "wasm")]
    pub fn from_params() -> Result<Self, Error> {
        let href = crate::utils::window()?.location().href()?;
        debug!("href={href}");
        let url = url::Url::parse(&href).map_err(|e| e.to_string())?;
        Ok(serde_qs::from_str(url.query().unwrap_or_default()).map_err(|e| e.to_string())?)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;

    #[test]
    fn test_create_package_path_map() {
        let opt: Opt = StructOpt::from_iter(["urdf-viz", "a.urdf", "--package-path", "a=b"]);
        assert_eq!(
            opt.create_package_path_map()
                .unwrap()
                .into_iter()
                .collect::<Vec<_>>(),
            vec![("a".to_owned(), "b".to_owned())]
        );
        let opt: Opt = StructOpt::from_iter([
            "urdf-viz",
            "a.urdf",
            "--package-path",
            "a=b",
            "--package-path",
            "c=d=e",
        ]);
        assert_eq!(
            opt.create_package_path_map()
                .unwrap()
                .into_iter()
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect::<Vec<_>>(),
            vec![
                ("a".to_owned(), "b".to_owned()),
                ("c".to_owned(), "d=e".to_owned())
            ]
        );
        let opt: Opt = StructOpt::from_iter(["urdf-viz", "a.urdf", "--package-path", "a="]);
        assert_eq!(
            opt.create_package_path_map()
                .unwrap()
                .into_iter()
                .collect::<Vec<_>>(),
            vec![("a".to_owned(), "".to_owned())]
        );
        let opt: Opt = StructOpt::from_iter(["urdf-viz", "a.urdf", "--package-path", "=a"]);
        assert!(opt.create_package_path_map().is_err());
        let opt: Opt = StructOpt::from_iter(["urdf-viz", "a.urdf", "--package-path", "a"]);
        assert!(opt.create_package_path_map().is_err());
    }

    #[test]
    fn test_params() {
        let url = url::Url::parse("http://localhost:8080/?urdf=https://raw.githubusercontent.com/openrr/urdf-viz/main/sample.urdf").unwrap();
        let _: Opt = serde_qs::from_str(url.query().unwrap_or_default()).unwrap();
    }
}
