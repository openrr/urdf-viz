use std::{io, sync::Arc};

use actix_web::*;
use serde::{Deserialize, Serialize};

use crate::handle::{JointNamesAndPositions, RobotOrigin, RobotStateHandle};

type Handle = web::Data<Arc<RobotStateHandle>>;

#[derive(Deserialize, Serialize, Debug, Clone)]
struct ResultResponse {
    is_ok: bool,
    reason: String,
}

#[derive(Debug)]
pub struct WebServer {
    port: u16,
    handle: Arc<RobotStateHandle>,
}

impl WebServer {
    pub fn new(port: u16, handle: Arc<RobotStateHandle>) -> Self {
        Self { port, handle }
    }

    pub fn handle(&self) -> Arc<RobotStateHandle> {
        self.handle.clone()
    }

    #[actix_web::main]
    pub async fn start(self) -> io::Result<()> {
        let handle = self.handle();

        HttpServer::new(move || {
            App::new()
                .data(handle.clone())
                .service(set_joint_positions)
                .service(set_robot_origin)
                .service(options_set_joint_positions)
                .service(options_set_robot_origin)
                .service(get_joint_positions)
                .service(get_robot_origin)
                .service(get_urdf_text)
                .service(options_get_joint_positions)
                .service(options_get_robot_origin)
                .service(options_get_urdf_text)
        })
        .bind(("0.0.0.0", self.port))?
        .run()
        .await
    }
}

#[post("set_joint_positions")]
async fn set_joint_positions(
    json: web::Json<JointNamesAndPositions>,
    handle: Handle,
) -> HttpResponse {
    let jp: JointNamesAndPositions = json.into_inner();
    if jp.names.len() != jp.positions.len() {
        HttpResponse::Ok().json(&ResultResponse {
            is_ok: false,
            reason: format!(
                "names and positions size mismatch ({} != {})",
                jp.names.len(),
                jp.positions.len()
            ),
        })
    } else {
        handle.set_target_joint_positions(jp);
        HttpResponse::Ok().json(&ResultResponse {
            is_ok: true,
            reason: "".to_string(),
        })
    }
}

#[post("set_robot_origin")]
async fn set_robot_origin(json: web::Json<RobotOrigin>, handle: Handle) -> HttpResponse {
    handle.set_target_robot_origin(json.into_inner());
    HttpResponse::Ok().json(&ResultResponse {
        is_ok: true,
        reason: "".to_string(),
    })
}

#[options("set_joint_positions")]
async fn options_set_joint_positions() -> HttpResponse {
    HttpResponse::NoContent()
        .header("Allow", "OPTIONS, POST")
        .header("Access-Control-Allow-Methods", "POST")
        .header("Access-Control-Allow-Origin", "*")
        .header("Access-Control-Allow-Headers", "authorization,content-type")
        .header("Access-Control-Max-Age", "86400")
        .finish()
}

#[options("set_robot_origin")]
async fn options_set_robot_origin() -> HttpResponse {
    HttpResponse::NoContent()
        .header("Allow", "OPTIONS, POST")
        .header("Access-Control-Allow-Methods", "POST")
        .header("Access-Control-Allow-Origin", "*")
        .header("Access-Control-Allow-Headers", "authorization,content-type")
        .header("Access-Control-Max-Age", "86400")
        .finish()
}

#[get("get_joint_positions")]
async fn get_joint_positions(handle: Handle) -> HttpResponse {
    let json = handle.current_joint_positions();
    HttpResponse::Ok().json(&*json)
}

#[get("get_robot_origin")]
async fn get_robot_origin(handle: Handle) -> HttpResponse {
    let origin = handle.current_robot_origin();
    HttpResponse::Ok().json(&*origin)
}

#[get("get_urdf_text")]
async fn get_urdf_text(handle: Handle) -> HttpResponse {
    match handle.urdf_text() {
        Some(text) => HttpResponse::from(&*text),
        None => HttpResponse::InternalServerError().finish(),
    }
}

#[options("get_joint_positions")]
async fn options_get_joint_positions() -> HttpResponse {
    HttpResponse::NoContent()
        .header("Allow", "OPTIONS, GET")
        .header("Access-Control-Allow-Methods", "GET")
        .header("Access-Control-Allow-Origin", "*")
        .header("Access-Control-Allow-Headers", "authorization")
        .header("Access-Control-Max-Age", "86400")
        .finish()
}

#[options("get_robot_origin")]
async fn options_get_robot_origin() -> HttpResponse {
    HttpResponse::NoContent()
        .header("Allow", "OPTIONS, GET")
        .header("Access-Control-Allow-Methods", "GET")
        .header("Access-Control-Allow-Origin", "*")
        .header("Access-Control-Allow-Headers", "authorization")
        .header("Access-Control-Max-Age", "86400")
        .finish()
}

#[options("get_urdf_text")]
async fn options_get_urdf_text() -> HttpResponse {
    HttpResponse::NoContent()
        .header("Allow", "OPTIONS, GET")
        .header("Access-Control-Allow-Methods", "GET")
        .header("Access-Control-Allow-Origin", "*")
        .header("Access-Control-Allow-Headers", "authorization")
        .header("Access-Control-Max-Age", "86400")
        .finish()
}
