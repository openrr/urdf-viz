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

use bevy::prelude::shape::Plane;
use bevy::prelude::*;
use bevy_stl::StlPlugin;
use k::nalgebra as na;
use k::prelude::*;
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

pub struct BevyUrdfViewerApp {
    app: App,
    input_path: PathBuf,
    urdf_robot: RobotModel,
    robot: k::Chain<f32>,
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

impl BevyUrdfViewerApp {
    pub fn new(
        mut urdf_robot: RobotModel,
        mut end_link_names: Vec<String>,
        is_collision: bool,
        disable_texture: bool,
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

        let user_plugin = DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "urdf-viz".to_owned(),
                resolution: (800., 600.).into(),
                ..Default::default()
            }),
            ..Default::default()
        });
        let mut app = App::new();
        app.add_plugins(user_plugin)
            .add_plugins(StlPlugin)
            .add_systems(Startup, bevy_app_startup_system);

        Ok(BevyUrdfViewerApp {
            app,
            input_path,
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
    pub fn run(&mut self) {
        self.app.run();
    }
}

fn bevy_app_startup_system(
    mut commands: Commands<'_, '_>,
    asset_server: Res<'_, AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
) {
    commands.spawn(PbrBundle {
        mesh: asset_server.load(format!("{}/object.stl", env!("CARGO_MANIFEST_DIR"))),
        material: materials.add(Color::YELLOW.into()),
        transform: Transform::from_rotation(Quat::from_rotation_x(f32::to_radians(-90.0))),
        ..Default::default()
    });
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            ..default()
        },
        transform: Transform::from_xyz(150.0, 150.0, 150.0),
        ..default()
    });
    commands.spawn(PbrBundle {
        mesh: meshes.add(shape::Plane::from_size(500.0).into()),
        material: materials.add(Color::WHITE.into()),
        ..Default::default()
    });
    commands.spawn(Camera3dBundle {
        transform: Transform::from_translation(Vec3::new(150.0, 150.0, 150.0))
            .looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

impl fmt::Debug for BevyUrdfViewerApp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        todo!()
    }
}

#[cfg(not(target_family = "wasm"))]
static ABORTED: AtomicBool = AtomicBool::new(false);

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
    #[serde(default)]
    pub axis_scale: f32,

    #[structopt(short = "b", long = "move-base-diff-unit", default_value = "0.1")]
    pub move_base_diff_unit: f32,

    #[structopt(short = "j", long = "move-joint-diff-unit", default_value = "0.1")]
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
