use parking_lot::{Mutex, MutexGuard};
use serde::{Deserialize, Serialize};
use std::{ops, sync::Arc};

use crate::utils::RobotModel;

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct JointNamesAndPositions {
    pub names: Vec<String>,
    pub positions: Vec<f32>,
}

#[derive(Deserialize, Serialize, Debug, Clone)]
pub struct RobotOrigin {
    pub position: [f32; 3],
    pub quaternion: [f32; 4],
}

impl Default for RobotOrigin {
    fn default() -> Self {
        Self {
            position: [0.0; 3],
            quaternion: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

#[derive(Deserialize, Serialize, Debug, Clone)]
pub struct ObjectOrigin {
    pub id: String,
    pub position: [f32; 3],
    pub quaternion: [f32; 4],
}

#[derive(Deserialize, Serialize, Debug, Clone)]
pub struct PointsAndColors {
    pub id: Option<String>,
    pub points: Vec<[f32; 3]>,
    pub colors: Vec<[f32; 3]>,
}

#[derive(Deserialize, Serialize, Debug, Clone)]
pub struct Cube {
    pub id: Option<String>,
    pub extent: Option<[f32; 3]>,
    pub color: Option<[f32; 3]>,
    pub position: Option<[f32; 3]>,
    pub quaternion: Option<[f32; 4]>,
}

#[derive(Deserialize, Serialize, Debug, Clone)]
pub struct Capsule {
    pub id: Option<String>,
    pub height: f32,
    pub radius: f32,
    pub color: Option<[f32; 3]>,
    pub position: Option<[f32; 3]>,
    pub quaternion: Option<[f32; 4]>,
}

#[derive(Deserialize, Serialize, Debug, Clone)]
pub struct AxisMarker {
    pub id: Option<String>,
    pub size: f32,
    pub position: Option<[f32; 3]>,
    pub quaternion: Option<[f32; 4]>,
}

/// Handle to get and modify the state of the robot.
#[derive(Debug, Default)]
pub struct RobotStateHandle {
    target_joint_positions: Mutex<Option<JointNamesAndPositions>>,
    pub(crate) current_joint_positions: Mutex<JointNamesAndPositions>,
    target_object_origin: Mutex<Option<ObjectOrigin>>,
    pub(crate) current_robot_origin: Mutex<RobotOrigin>,
    point_cloud: Mutex<Option<PointsAndColors>>,
    cube: Mutex<Option<Cube>>,
    capsule: Mutex<Option<Capsule>>,
    axis_marker: Mutex<Option<AxisMarker>>,
    pub(crate) urdf_text: Option<Arc<Mutex<String>>>,
    robot: Mutex<Option<RobotModel>>,
}

// Wrapper type to prevent parking_lot updates from becoming a breaking change.
macro_rules! impl_guard {
    ($guard_name:ident($target:ident)) => {
        #[derive(Debug)]
        pub struct $guard_name<'a>(MutexGuard<'a, $target>);
        impl ops::Deref for $guard_name<'_> {
            type Target = $target;
            fn deref(&self) -> &Self::Target {
                &self.0
            }
        }
        impl ops::DerefMut for $guard_name<'_> {
            fn deref_mut(&mut self) -> &mut Self::Target {
                &mut self.0
            }
        }
    };
}
impl_guard!(JointNamesAndPositionsLockGuard(JointNamesAndPositions));
impl_guard!(RobotOriginLockGuard(RobotOrigin));
impl_guard!(UrdfTextLockGuard(String));

pub const ROBOT_OBJECT_ID: &str = "robot";

impl RobotStateHandle {
    pub fn current_joint_positions(&self) -> JointNamesAndPositionsLockGuard<'_> {
        JointNamesAndPositionsLockGuard(self.current_joint_positions.lock())
    }

    pub fn current_robot_origin(&self) -> RobotOriginLockGuard<'_> {
        RobotOriginLockGuard(self.current_robot_origin.lock())
    }

    pub fn urdf_text(&self) -> Option<UrdfTextLockGuard<'_>> {
        Some(UrdfTextLockGuard(self.urdf_text.as_ref()?.lock()))
    }

    pub fn set_target_joint_positions(&self, joint_positions: JointNamesAndPositions) {
        *self.target_joint_positions.lock() = Some(joint_positions);
    }

    pub fn set_target_robot_origin(&self, robot_origin: RobotOrigin) {
        *self.target_object_origin.lock() = Some(ObjectOrigin {
            id: ROBOT_OBJECT_ID.to_owned(),
            position: robot_origin.position,
            quaternion: robot_origin.quaternion,
        });
    }

    pub fn set_target_object_origin(&self, object_origin: ObjectOrigin) {
        *self.target_object_origin.lock() = Some(object_origin);
    }

    pub fn set_point_cloud(&self, points_and_colors: PointsAndColors) {
        *self.point_cloud.lock() = Some(points_and_colors);
    }

    pub fn set_cube(&self, cube: Cube) {
        *self.cube.lock() = Some(cube);
    }

    pub fn set_capsule(&self, capsule: Capsule) {
        *self.capsule.lock() = Some(capsule);
    }

    pub fn set_axis_marker(&self, axis_marker: AxisMarker) {
        *self.axis_marker.lock() = Some(axis_marker);
    }

    pub fn set_robot(&self, robot: RobotModel) {
        // set_robot may change name or number of joints, so reset target_joint_positions.
        *self.target_joint_positions.lock() = None;
        *self.robot.lock() = Some(robot);
    }

    pub fn take_target_joint_positions(&self) -> Option<JointNamesAndPositions> {
        self.target_joint_positions.lock().take()
    }

    pub fn take_target_object_origin(&self) -> Option<ObjectOrigin> {
        self.target_object_origin.lock().take()
    }

    pub(crate) fn take_point_cloud(&self) -> Option<PointsAndColors> {
        self.point_cloud.lock().take()
    }

    pub(crate) fn take_cube(&self) -> Option<Cube> {
        self.cube.lock().take()
    }

    pub(crate) fn take_capsule(&self) -> Option<Capsule> {
        self.capsule.lock().take()
    }

    pub(crate) fn take_axis_marker(&self) -> Option<AxisMarker> {
        self.axis_marker.lock().take()
    }

    pub(crate) fn take_robot(&self) -> Option<RobotModel> {
        self.robot.lock().take()
    }
}
