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
use urdf_viz::{app::*, WebServer};

fn main() -> urdf_viz::Result<()> {
    env_logger::init();
    let opt = Opt::from_args();
    let urdf_robot = urdf_rs::utils::read_urdf_or_xacro(&opt.input_urdf_or_xacro)?;
    let ik_constraints = opt.create_ik_constraints();
    let mut app = UrdfViewerApp::new(
        &opt.input_urdf_or_xacro,
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
    );
    app.set_ik_constraints(ik_constraints);
    app.init();
    let web_server = WebServer::new(opt.web_server_port, app.handle());
    std::thread::spawn(move || web_server.start());
    app.run();
    Ok(())
}
