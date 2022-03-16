use crossbeam_queue::ArrayQueue;
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
#[derive(Debug)]
pub struct RobotStateHandle {
    pub(crate) target_joint_positions: Mutex<Option<JointNamesAndPositions>>,
    pub(crate) current_joint_positions: Mutex<JointNamesAndPositions>,
    pub(crate) target_object_origin: ArrayQueue<ObjectOrigin>,
    pub(crate) current_robot_origin: Mutex<RobotOrigin>,
    pub(crate) point_cloud: ArrayQueue<PointsAndColors>,
    pub(crate) cube: ArrayQueue<Cube>,
    pub(crate) capsule: ArrayQueue<Capsule>,
    pub(crate) axis_marker: ArrayQueue<AxisMarker>,
    pub(crate) urdf_text: Option<Arc<Mutex<String>>>,
    pub(crate) robot: Mutex<Option<RobotModel>>,
}

impl Default for RobotStateHandle {
    fn default() -> Self {
        const QUEUE_CAP: usize = 8;
        Self {
            target_joint_positions: Mutex::default(),
            current_joint_positions: Mutex::default(),
            target_object_origin: ArrayQueue::new(QUEUE_CAP),
            current_robot_origin: Mutex::default(),
            point_cloud: ArrayQueue::new(QUEUE_CAP),
            cube: ArrayQueue::new(QUEUE_CAP),
            capsule: ArrayQueue::new(QUEUE_CAP),
            axis_marker: ArrayQueue::new(QUEUE_CAP),
            urdf_text: None,
            robot: Mutex::default(),
        }
    }
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
        self.target_object_origin.force_push(ObjectOrigin {
            id: ROBOT_OBJECT_ID.to_owned(),
            position: robot_origin.position,
            quaternion: robot_origin.quaternion,
        });
    }

    pub fn set_target_object_origin(&self, object_origin: ObjectOrigin) {
        self.target_object_origin.force_push(object_origin);
    }

    pub fn set_point_cloud(&self, points_and_colors: PointsAndColors) {
        self.point_cloud.force_push(points_and_colors);
    }

    pub fn set_cube(&self, cube: Cube) {
        self.cube.force_push(cube);
    }

    pub fn set_capsule(&self, capsule: Capsule) {
        self.capsule.force_push(capsule);
    }

    pub fn set_axis_marker(&self, axis_marker: AxisMarker) {
        self.axis_marker.force_push(axis_marker);
    }

    pub fn set_robot(&self, robot: RobotModel) {
        // set_robot may change name or number of joints, so reset target_joint_positions.
        *self.target_joint_positions.lock() = None;
        *self.robot.lock() = Some(robot);
    }

    pub fn take_target_joint_positions(&self) -> Option<JointNamesAndPositions> {
        self.target_joint_positions.lock().take()
    }

    pub fn pop_target_object_origin(&self) -> Option<ObjectOrigin> {
        self.target_object_origin.pop()
    }

    pub(crate) fn take_robot(&self) -> Option<RobotModel> {
        self.robot.lock().take()
    }
}
