use rouille;
use rouille::Response;
use std::sync::{Arc, Mutex};

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct JointNamesAndAngles {
    pub names: Vec<String>,
    pub angles: Vec<f32>,
}

#[derive(Debug, Clone)]
pub struct JointNamesAndAnglesRequest {
    pub joint_angles: JointNamesAndAngles,
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
    pub target_joint_angles: Arc<Mutex<JointNamesAndAnglesRequest>>,
    pub current_joint_angles: Arc<Mutex<JointNamesAndAngles>>,
}

impl WebServer {
    pub fn new(port: u16) -> Self {
        Self {
            port,
            target_joint_angles: Arc::new(Mutex::new(JointNamesAndAnglesRequest {
                joint_angles: JointNamesAndAngles::default(),
                requested: false,
            })),
            current_joint_angles: Arc::new(Mutex::new(JointNamesAndAngles::default())),
        }
    }

    pub fn clone_in_out(
        &self,
    ) -> (
        Arc<Mutex<JointNamesAndAnglesRequest>>,
        Arc<Mutex<JointNamesAndAngles>>,
    ) {
        (
            self.target_joint_angles.clone(),
            self.current_joint_angles.clone(),
        )
    }
    pub fn start(self) {
        rouille::start_server(("0.0.0.0", self.port), move |request| {
            router!(request,
                    (POST) (/set_joint_angles) => {
                        let json: JointNamesAndAngles = try_or_400!(rouille::input::json_input(request));
                        let mut ja = try_or_404!(self.target_joint_angles.lock());
                        ja.joint_angles = json;
                        if ja.joint_angles.names.len() != ja.joint_angles.angles.len() {
                            Response::json(&ResultResponse { is_ok: false,
                                                     reason: format!("names and angles size mismatch ({} != {})",
                                                        ja.joint_angles.names.len(), ja.joint_angles.angles.len()) })
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
