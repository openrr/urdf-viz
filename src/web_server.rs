use std::sync::Arc;

use serde::{Deserialize, Serialize};
use warp::{Filter, Rejection};

use crate::handle::{JointNamesAndPositions, RobotOrigin, RobotStateHandle};

type Handle = Arc<RobotStateHandle>;

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

    #[tokio::main]
    pub async fn start(self) {
        let handle = self.handle();

        let routes = set_joint_positions(handle.clone())
            .or(set_robot_origin(handle.clone()))
            .or(get_joint_positions(handle.clone()))
            .or(get_robot_origin(handle.clone()))
            .or(get_urdf_text(handle.clone()));
        warp::serve(routes).run(([0, 0, 0, 0], self.port)).await;
    }
}

fn set_joint_positions(
    handle: Handle,
) -> impl Filter<Extract = (warp::reply::Json,), Error = Rejection> + Clone {
    warp::post()
        .and(warp::path("set_joint_positions"))
        .and(warp::body::json())
        .and_then(move |jp: JointNamesAndPositions| {
            let handle = handle.clone();
            async move {
                if jp.names.len() != jp.positions.len() {
                    Ok::<_, Rejection>(warp::reply::json(&ResultResponse {
                        is_ok: false,
                        reason: format!(
                            "names and positions size mismatch ({} != {})",
                            jp.names.len(),
                            jp.positions.len()
                        ),
                    }))
                } else {
                    handle.set_target_joint_positions(jp);
                    Ok(warp::reply::json(&ResultResponse {
                        is_ok: true,
                        reason: "".to_string(),
                    }))
                }
            }
        })
}

fn set_robot_origin(
    handle: Handle,
) -> impl Filter<Extract = (warp::reply::Json,), Error = Rejection> + Clone {
    warp::post()
        .and(warp::path("set_robot_origin"))
        .and(warp::body::json())
        .and_then(move |robot_origin: RobotOrigin| {
            let handle = handle.clone();
            async move {
                handle.set_target_robot_origin(robot_origin);
                Ok::<_, Rejection>(warp::reply::json(&ResultResponse {
                    is_ok: true,
                    reason: "".to_string(),
                }))
            }
        })
}

fn get_joint_positions(
    handle: Handle,
) -> impl Filter<Extract = (warp::reply::Json,), Error = Rejection> + Clone {
    warp::get()
        .and(warp::path("get_joint_positions"))
        .and_then(move || {
            let handle = handle.clone();
            async move {
                let json = handle.current_joint_positions();
                Ok::<_, Rejection>(warp::reply::json(&*json))
            }
        })
}

fn get_robot_origin(
    handle: Handle,
) -> impl Filter<Extract = (warp::reply::Json,), Error = Rejection> + Clone {
    warp::get()
        .and(warp::path("get_robot_origin"))
        .and_then(move || {
            let handle = handle.clone();
            async move {
                let robot_origin = handle.current_robot_origin();
                Ok::<_, Rejection>(warp::reply::json(&*robot_origin))
            }
        })
}

fn get_urdf_text(
    handle: Handle,
) -> impl Filter<Extract = (warp::reply::Json,), Error = Rejection> + Clone {
    warp::get()
        .and(warp::path("get_urdf_text"))
        .and_then(move || {
            let handle = handle.clone();
            async move {
                match handle.urdf_text() {
                    Some(text) => Ok(warp::reply::json(&*text)),
                    None => Err(warp::reject::not_found()),
                }
            }
        })
}
