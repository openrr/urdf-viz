use rouille;
use rouille::Response;
use std::sync::{Arc, Mutex};

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct JointNamesAndPositions {
    pub names: Vec<String>,
    pub positions: Vec<f32>,
}

#[derive(Debug, Clone)]
pub struct JointNamesAndPositionsRequest {
    pub joint_positions: JointNamesAndPositions,
    pub requested: bool,
}

#[derive(Deserialize, Serialize, Debug, Clone)]
struct ResultResponse {
    is_ok: bool,
    reason: String,
}

#[derive(Debug)]
pub struct WebServer {
    pub port: u16,
    pub target_joint_positions: Arc<Mutex<JointNamesAndPositionsRequest>>,
    pub current_joint_positions: Arc<Mutex<JointNamesAndPositions>>,
}

impl WebServer {
    pub fn new(port: u16) -> Self {
        Self {
            port,
            target_joint_positions: Arc::new(Mutex::new(JointNamesAndPositionsRequest {
                joint_positions: JointNamesAndPositions::default(),
                requested: false,
            })),
            current_joint_positions: Arc::new(Mutex::new(JointNamesAndPositions::default())),
        }
    }

    pub fn clone_in_out(
        &self,
    ) -> (
        Arc<Mutex<JointNamesAndPositionsRequest>>,
        Arc<Mutex<JointNamesAndPositions>>,
    ) {
        (
            self.target_joint_positions.clone(),
            self.current_joint_positions.clone(),
        )
    }
    pub fn start(self) {
        rouille::start_server(("0.0.0.0", self.port), move |request| {
            router!(request,
                    (POST) (/set_joint_positions) => {
                        let json: JointNamesAndPositions = try_or_400!(rouille::input::json_input(request));
                        let mut ja = try_or_404!(self.target_joint_positions.lock());
                        ja.joint_positions = json;
                        if ja.joint_positions.names.len() != ja.joint_positions.positions.len() {
                            Response::json(&ResultResponse { is_ok: false,
                                                     reason: format!("names and positions size mismatch ({} != {})",
                                                        ja.joint_positions.names.len(), ja.joint_positions.positions.len()) })
                        } else {
                            ja.requested = true;
                            Response::json(&ResultResponse { is_ok: true, reason: "".to_string() })
                        }
                    },
                    (OPTIONS) (/set_joint_angles) => {
                        Response::empty_204()
                            .with_additional_header("Allow", "OPTIONS, POST")
                            .with_additional_header("Access-Control-Allow-Methods", "POST")
                            .with_additional_header("Access-Control-Allow-Origin", "*")
                            .with_additional_header("Access-Control-Allow-Headers", "authorization,content-type")
                            .with_additional_header("Access-Control-Max-Age", "86400")
                    },
                    (GET) (/get_joint_angles) => {
                        let ja = try_or_404!(self.current_joint_angles.lock());
                        Response::json(&*ja)
                    },
                    (OPTIONS) (/get_joint_angles) => {
                        Response::empty_204()
                            .with_additional_header("Allow", "OPTIONS, GET")
                            .with_additional_header("Access-Control-Allow-Methods", "GET")
                            .with_additional_header("Access-Control-Allow-Origin", "*")
                            .with_additional_header("Access-Control-Allow-Headers", "authorization")
                            .with_additional_header("Access-Control-Max-Age", "86400")
                    },

                    _ => Response::empty_404(),
                )
        });
    }
}
