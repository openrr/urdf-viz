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

#![warn(rust_2018_idioms)]

use structopt::StructOpt;
use tracing::{debug, warn};
use urdf_viz::{app::*, WebServer};

const DEFAULT_WEB_SERVER_PORT: u16 = 7777;

#[tokio::main]
async fn main() -> urdf_viz::Result<()> {
    tracing_subscriber::fmt::init();
    let opt = Opt::from_args();
    debug!(?opt);
    let package_path = opt.create_package_path_map()?;
    let urdf_robot = urdf_viz::utils::RobotModel::new(&opt.input_urdf_or_xacro, package_path)?;
    let ik_constraints = opt.create_ik_constraints();
    let mut app = UrdfViewerApp::new(
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
    )?;
    app.set_ik_constraints(ik_constraints);
    app.init();

    match WebServer::new(
        opt.web_server_port.unwrap_or(DEFAULT_WEB_SERVER_PORT),
        app.handle(),
    )
    .bind()
    {
        Ok(server) => {
            tokio::spawn(async move { server.await.unwrap() });
        }
        Err(e) => {
            if opt.web_server_port.is_none() {
                // If the user didn't specify a port, the user may not need web server, so treat as a warning.
                warn!("failed to start web server with default port: {e}");
            } else {
                // If the user specified a port, the user intends to use web server, so treat as an error.
                return Err(e);
            }
        }
    }

    app.run();
    Ok(())
}
