use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex, RwLock, RwLockReadGuard};

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct JointNamesAndPositions {
    pub names: Vec<String>,
    pub positions: Vec<f32>,
}

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct RobotOrigin {
    pub position: [f32; 3],
    pub quaternion: [f32; 4],
}

#[derive(Deserialize, Serialize, Debug, Clone)]
pub struct PointsAndColors {
    pub id: Option<String>,
    pub points: Vec<[f32; 3]>,
    pub colors: Vec<[f32; 3]>,
}

/// Handle to get and modify the state of the robot.
#[derive(Debug, Default)]
pub struct RobotStateHandle {
    target_joint_positions: Mutex<Option<JointNamesAndPositions>>,
    pub(crate) current_joint_positions: RwLock<JointNamesAndPositions>,
    target_robot_origin: Mutex<Option<RobotOrigin>>,
    pub(crate) current_robot_origin: RwLock<RobotOrigin>,
    point_cloud: Mutex<Option<PointsAndColors>>,
    pub(crate) urdf_text: Option<Arc<RwLock<String>>>,
}

impl RobotStateHandle {
    pub fn current_joint_positions(&self) -> RwLockReadGuard<'_, JointNamesAndPositions> {
        self.current_joint_positions.read().unwrap()
    }

    pub fn current_robot_origin(&self) -> RwLockReadGuard<'_, RobotOrigin> {
        self.current_robot_origin.read().unwrap()
    }

    pub fn urdf_text(&self) -> Option<RwLockReadGuard<'_, String>> {
        Some(self.urdf_text.as_ref()?.read().unwrap())
    }

    pub fn set_target_joint_positions(&self, joint_positions: JointNamesAndPositions) {
        *self.target_joint_positions.lock().unwrap() = Some(joint_positions);
    }

    pub fn set_target_robot_origin(&self, robot_origin: RobotOrigin) {
        *self.target_robot_origin.lock().unwrap() = Some(robot_origin);
    }

    pub fn set_point_cloud(&self, points_and_colors: PointsAndColors) {
        *self.point_cloud.lock().unwrap() = Some(points_and_colors);
    }

    pub(crate) fn take_target_joint_positions(&self) -> Option<JointNamesAndPositions> {
        self.target_joint_positions.lock().unwrap().take()
    }

    pub(crate) fn take_target_robot_origin(&self) -> Option<RobotOrigin> {
        self.target_robot_origin.lock().unwrap().take()
    }

    pub(crate) fn take_point_cloud(&self) -> Option<PointsAndColors> {
        self.point_cloud.lock().unwrap().take()
    }
}
