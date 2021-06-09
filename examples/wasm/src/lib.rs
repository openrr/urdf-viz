#![cfg(target_arch = "wasm32")]

use urdf_viz::app::*;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(start)]
pub async fn main() -> Result<(), JsValue> {
    console_error_panic_hook::set_once();
    wasm_logger::init(wasm_logger::Config::default());

    run().await
}

const SAMPLE_URDF_PATH: &str = "sample.urdf";

async fn run() -> Result<(), JsValue> {
    let mut opt = Opt::from_params()?;
    if opt.input_urdf_or_xacro.is_empty() {
        opt.input_urdf_or_xacro = SAMPLE_URDF_PATH.to_string();
    }
    let urdf_robot = urdf_viz::utils::read_urdf(&opt.input_urdf_or_xacro).await?;
    let mut app = UrdfViewerApp::new(
        &opt.input_urdf_or_xacro,
        urdf_robot,
        opt.end_link_names,
        opt.is_collision,
        opt.disable_texture,
        opt.web_server_port,
        (
            opt.back_ground_color_r,
            opt.back_ground_color_g,
            opt.back_ground_color_b,
        ),
        (opt.tile_color1_r, opt.tile_color1_g, opt.tile_color1_b),
        (opt.tile_color2_r, opt.tile_color2_g, opt.tile_color2_b),
        opt.ground_height,
    );
    let mut ik_constraints = k::Constraints::default();
    ik_constraints.position_x = !opt.ignore_ik_position_x;
    ik_constraints.position_y = !opt.ignore_ik_position_y;
    ik_constraints.position_z = !opt.ignore_ik_position_z;
    ik_constraints.rotation_x = !opt.ignore_ik_rotation_x;
    ik_constraints.rotation_y = !opt.ignore_ik_rotation_y;
    ik_constraints.rotation_z = !opt.ignore_ik_rotation_z;
    app.set_ik_constraints(ik_constraints);
    app.init();
    app.run();

    Ok(())
}
