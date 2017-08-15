extern crate alga;
extern crate env_logger;
extern crate glfw;
extern crate k;
extern crate nalgebra as na;
extern crate rand;
extern crate structopt;
extern crate urdf_rs;
extern crate urdf_viz;

use glfw::{Action, WindowEvent, Key};
use k::InverseKinematicsSolver;
use k::KinematicChain;
use std::path::Path;

#[cfg(target_os = "macos")]
static NATIVE_MOD: glfw::Modifiers = glfw::Super;

#[cfg(not(target_os = "macos"))]
static NATIVE_MOD: glfw::Modifiers = glfw::Control;

fn move_joint_by_random(robot: &mut k::LinkTree<f32>) -> Result<(), k::JointError> {
    let angles_vec = robot.map_for_joints_link(&|link| match link.joint.limits {
                                                    Some(ref range) => {
                                                        (range.max - range.min) *
                                                        rand::random::<f32>() +
                                                        range.min
                                                    }
                                                    None => (rand::random::<f32>() - 0.5) * 2.0,
                                                });
    robot.set_joint_angles(&angles_vec)
}

fn move_joint_by_index(index: usize,
                       diff_angle: f32,
                       robot: &mut k::LinkTree<f32>)
                       -> Result<(), k::JointError> {
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

fn main() {
    use structopt::StructOpt;

    env_logger::init().unwrap();
    let opt = urdf_viz::Opt::from_args();
    if opt.clean {
        urdf_viz::clean_cahce_dir().unwrap();
    }
    let mesh_convert = opt.get_mesh_convert_method();
    let ik_dof = opt.ik_dof;
    let input_path = Path::new(&opt.input_urdf_or_xacro);
    let urdf_path = urdf_viz::convert_xacro_if_needed_and_get_path(input_path).unwrap();
    let urdf_robo = urdf_rs::read_file(&urdf_path).unwrap();
    let mut robot = k::urdf::create_tree::<f32>(&urdf_robo);
    let mut viewer = urdf_viz::Viewer::new(urdf_robo);
    viewer.setup(mesh_convert);
    let base_transform =
        na::Isometry3::from_parts(na::Translation3::new(0.0, 0.0, 0.0),
                                  na::UnitQuaternion::from_euler_angles(0.0, 1.57, 1.57));
    robot.set_root_transform(base_transform);
    let mut arms = k::create_kinematic_chains_with_dof_limit(&robot, ik_dof);
    let num_arms = arms.len();
    println!("num_arms = {}", num_arms);
    let solver = k::JacobianIKSolverBuilder::new().finalize();
    let dof = robot.dof();
    let mut index_of_move_joint = LoopIndex::new(dof);
    let mut index_of_arm = LoopIndex::new(num_arms);
    let mut is_ctrl = false;
    let mut is_shift = false;
    let mut last_cur_pos_y = 0f64;
    let mut last_cur_pos_x = 0f64;
    let joint_names = robot.get_joint_names();
    let num_joints = joint_names.len();
    viewer.update(&mut robot);
    while viewer.render() {
        viewer.draw_text(r"
[:    joint ID +1
]:    joint ID -1
,:    IK target ID +1
.:    IK target ID -1
r:    set random angles
Up:   joint angle +0.1
Down: joint angle -0.1
Ctrl+Drag: move joint
Shift+Drag: IK (y, z)
Shift+Ctrl+Drag: IK (y, x)
",
                         40,
                         &na::Point2::new(2000.0, 10.0),
                         &na::Point3::new(1f32, 1.0, 1.0));
        if num_joints > 0 {
            viewer.draw_text(&format!("moving joint name [{}]",
                                      joint_names[index_of_move_joint.get()]),
                             60,
                             &na::Point2::new(10f32, 20.0),
                             &na::Point3::new(0.5f32, 0.5, 1.0));
        }
        if num_arms > 0 {
            viewer.draw_text(&format!("IK target name [{}]", arms[index_of_arm.get()].name),
                             60,
                             &na::Point2::new(10f32, 100.0),
                             &na::Point3::new(0.5f32, 0.8, 0.2));
        }
        if is_ctrl && !is_shift {
            viewer.draw_text("moving joint by drag",
                             60,
                             &na::Point2::new(10f32, 150.0),
                             &na::Point3::new(0.9f32, 0.5, 1.0));
        }
        if is_shift {
            viewer.draw_text("solving ik",
                             60,
                             &na::Point2::new(10f32, 150.0),
                             &na::Point3::new(0.9f32, 0.5, 1.0));
        }
        for mut event in viewer.events().iter() {
            match event.value {
                WindowEvent::MouseButton(_, Action::Press, mods) => {
                    if mods.contains(NATIVE_MOD) {
                        is_ctrl = true;
                        event.inhibited = true;
                    }
                    if mods.contains(glfw::Shift) {
                        is_shift = true;
                        event.inhibited = true;
                    }
                }
                WindowEvent::CursorPos(x, y) => {
                    if is_ctrl && !is_shift {
                        event.inhibited = true;
                        let move_gain = 0.005;
                        if num_joints > 0 {
                            move_joint_by_index(index_of_move_joint.get(),
                                                (((x - last_cur_pos_x) + (y - last_cur_pos_y)) *
                                                 move_gain) as
                                                f32,
                                                &mut robot)
                                    .unwrap_or(());
                            viewer.update(&mut robot);
                        }
                    }
                    if is_shift {
                        event.inhibited = true;
                        if num_arms > 0 {
                            let mut target = arms[index_of_arm.get()].calc_end_transform();
                            let ik_move_gain = 0.002;
                            // [0]: y
                            // [1]: z
                            // [2]: x
                            target.translation.vector[0] -= ((x - last_cur_pos_x) * ik_move_gain) as
                                                            f32;
                            if is_ctrl {
                                target.translation.vector[2] +=
                                    ((y - last_cur_pos_y) * ik_move_gain) as f32;
                            } else {
                                target.translation.vector[1] -=
                                    ((y - last_cur_pos_y) * ik_move_gain) as f32;
                            }
                            solver
                                .solve(&mut arms[index_of_arm.get()], &target)
                                .unwrap_or_else(|err| {
                                                    println!("Err: {}", err);
                                                    0.0f32
                                                });
                            viewer.update(&mut robot);
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
                        Key::LeftBracket => index_of_move_joint.inc(),
                        Key::RightBracket => index_of_move_joint.dec(),
                        Key::Period => index_of_arm.inc(),
                        Key::Comma => index_of_arm.dec(),
                        Key::R => {
                            if num_joints > 0 {
                                move_joint_by_random(&mut robot).unwrap_or(());
                                viewer.update(&mut robot)
                            }
                        }
                        Key::Up => {
                            if num_joints > 0 {
                                move_joint_by_index(index_of_move_joint.get(), 0.1, &mut robot)
                                    .unwrap_or(());
                                viewer.update(&mut robot);
                            }
                        }
                        Key::Down => {
                            if num_joints > 0 {
                                move_joint_by_index(index_of_move_joint.get(), 0.1, &mut robot)
                                    .unwrap_or(());
                                viewer.update(&mut robot);
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
