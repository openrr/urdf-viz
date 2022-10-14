use std::{future::Future, sync::Arc};

use axum::{
    extract::Extension,
    http::StatusCode,
    routing::{get, post},
    Json, Router,
};
use serde::{Deserialize, Serialize};

use crate::handle::{JointNamesAndPositions, RobotOrigin, RobotStateHandle};

type Handle = Arc<RobotStateHandle>;

#[derive(Deserialize, Serialize, Debug, Clone)]
struct ResultResponse {
    is_ok: bool,
    reason: String,
}

impl ResultResponse {
    const SUCCESS: Self = ResultResponse {
        is_ok: true,
        reason: String::new(),
    };
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

    pub fn bind(self) -> crate::errors::Result<impl Future<Output = hyper::Result<()>> + Send> {
        let app = app(self.handle());

        let addr = ([0, 0, 0, 0], self.port).into();
        Ok(axum::Server::try_bind(&addr)
            .map_err(|e| format!("error binding to {addr}: {e}"))?
            .serve(app.into_make_service()))
    }
}

fn app(handle: Handle) -> Router {
    Router::new()
        .route("/set_joint_positions", post(set_joint_positions))
        .route("/set_robot_origin", post(set_robot_origin))
        .route("/set_reload_request", post(set_reload_request))
        .route("/get_joint_positions", get(get_joint_positions))
        .route("/get_robot_origin", get(get_robot_origin))
        .route("/get_urdf_text", get(get_urdf_text))
        .layer(Extension(handle))
        .layer(tower_http::trace::TraceLayer::new_for_http())
}

async fn set_reload_request(Extension(handle): Extension<Handle>) -> Json<ResultResponse> {
    handle.set_reload_request();
    Json(ResultResponse::SUCCESS)
}

async fn set_joint_positions(
    Json(jp): Json<JointNamesAndPositions>,
    Extension(handle): Extension<Handle>,
) -> Json<ResultResponse> {
    if jp.names.len() != jp.positions.len() {
        Json(ResultResponse {
            is_ok: false,
            reason: format!(
                "names and positions size mismatch ({} != {})",
                jp.names.len(),
                jp.positions.len()
            ),
        })
    } else {
        handle.set_target_joint_positions(jp);
        Json(ResultResponse::SUCCESS)
    }
}

async fn set_robot_origin(
    Json(robot_origin): Json<RobotOrigin>,
    Extension(handle): Extension<Handle>,
) -> Json<ResultResponse> {
    handle.set_target_robot_origin(robot_origin);
    Json(ResultResponse::SUCCESS)
}

async fn get_joint_positions(Extension(handle): Extension<Handle>) -> Json<JointNamesAndPositions> {
    Json(handle.current_joint_positions().clone())
}

async fn get_robot_origin(Extension(handle): Extension<Handle>) -> Json<RobotOrigin> {
    Json(handle.current_robot_origin().clone())
}

async fn get_urdf_text(Extension(handle): Extension<Handle>) -> Result<String, StatusCode> {
    match handle.urdf_text() {
        Some(text) => Ok(text.clone()),
        None => Err(StatusCode::NOT_FOUND),
    }
}
