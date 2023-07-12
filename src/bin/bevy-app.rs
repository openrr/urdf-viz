use structopt::StructOpt;
use tracing::debug;
use urdf_viz::bevy_app::*;

#[tokio::main]
async fn main() -> urdf_viz::Result<()> {
    tracing_subscriber::fmt::init();
    let opt = Opt::from_args();
    debug!(?opt);
    let package_path = opt.create_package_path_map()?;
    let urdf_robot = urdf_viz::utils::RobotModel::new(
        &opt.input_urdf_or_xacro,
        package_path,
        &opt.xacro_arguments,
    )?;

    let mut app = BevyUrdfViewerApp::new(
        urdf_robot,
        opt.end_link_names,
        opt.is_collision,
        opt.disable_texture,
        (
            opt.back_ground_color_r,
            opt.back_ground_color_g,
            opt.back_ground_color_b,
        ),
        (opt.tile_color1_r, opt.tile_color1_g, opt.tile_color1_b),
        (opt.tile_color2_r, opt.tile_color2_g, opt.tile_color2_b),
        opt.ground_height,
        opt.hide_menu,
        opt.axis_scale,
        opt.move_base_diff_unit,
        opt.move_joint_diff_unit,
    )?;

    app.run();
    Ok(())
}
