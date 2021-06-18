use std::{io, sync::Arc};

use actix_web::*;
use serde::{Deserialize, Serialize};

use crate::handle::{JointNamesAndPositions, RobotOrigin, RobotStateHandle};

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
                .service(options_get_joint_positions)
                .service(options_get_robot_origin)
        })
        .bind(("0.0.0.0", self.port))?
        .run()
        .await
    }
}

#[post("set_joint_positions")]
async fn set_joint_positions(
    json: web::Json<JointNamesAndPositions>,
    handle: web::Data<Arc<RobotStateHandle>>,
) -> HttpResponse {
    let mut jp_request = handle.target_joint_positions.lock().unwrap();
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
        *jp_request = Some(jp);
        HttpResponse::Ok().json(&ResultResponse {
            is_ok: true,
            reason: "".to_string(),
        })
    }
}

#[post("set_robot_origin")]
async fn set_robot_origin(
    json: web::Json<RobotOrigin>,
    handle: web::Data<Arc<RobotStateHandle>>,
) -> HttpResponse {
    let mut pose_request = handle.target_robot_origin.lock().unwrap();
    *pose_request = Some(json.into_inner());
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
async fn get_joint_positions(handle: web::Data<Arc<RobotStateHandle>>) -> HttpResponse {
    let json = handle.current_joint_positions.lock().unwrap();
    HttpResponse::Ok().json(&*json)
}

#[get("get_robot_origin")]
async fn get_robot_origin(handle: web::Data<Arc<RobotStateHandle>>) -> HttpResponse {
    let origin = handle.current_robot_origin.lock().unwrap();
    HttpResponse::Ok().json(&*origin)
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
