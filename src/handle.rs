use serde::{Deserialize, Serialize};
use std::sync::Mutex;

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

/// Handle to get and modify the state of the robot.
#[derive(Debug)]
pub struct RobotStateHandle {
    pub target_joint_positions: Mutex<Option<JointNamesAndPositions>>,
    pub current_joint_positions: Mutex<JointNamesAndPositions>,
    pub target_robot_origin: Mutex<Option<RobotOrigin>>,
    pub current_robot_origin: Mutex<RobotOrigin>,
}

impl Default for RobotStateHandle {
    fn default() -> Self {
        Self {
            target_joint_positions: Mutex::new(None),
            current_joint_positions: Mutex::new(JointNamesAndPositions::default()),
            target_robot_origin: Mutex::new(None),
            current_robot_origin: Mutex::new(RobotOrigin::default()),
        }
    }
}
