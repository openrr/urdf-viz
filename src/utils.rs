#[cfg(not(target_arch = "wasm32"))]
pub(crate) use native::*;
#[cfg(target_arch = "wasm32")]
pub use wasm::*;

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use std::sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    };

    use log::error;

    pub(crate) struct RobotModel {
        robot: urdf_rs::Robot,
        needs_reload: Arc<AtomicBool>,
    }

    impl RobotModel {
        pub(crate) fn new(robot: urdf_rs::Robot) -> (Self, Arc<AtomicBool>) {
            let needs_reload = Arc::new(AtomicBool::new(false));
            (
                Self {
                    robot,
                    needs_reload: needs_reload.clone(),
                },
                needs_reload,
            )
        }

        pub(crate) fn get(&mut self) -> &urdf_rs::Robot {
            &self.robot
        }

        pub(crate) fn request_reload(&mut self, input_file: impl AsRef<str>) {
            match urdf_rs::utils::read_urdf_or_xacro(input_file.as_ref()) {
                Ok(robot) => {
                    self.robot = robot;
                    self.needs_reload.store(true, Ordering::Relaxed);
                }
                Err(e) => {
                    error!("{}", e);
                }
            }
        }
    }
}

#[cfg(target_arch = "wasm32")]
mod wasm {
    use std::{
        path::Path,
        sync::{
            atomic::{AtomicBool, Ordering},
            Arc,
        },
    };

    use futures::future::FutureExt;
    use log::{debug, error};
    use serde::{Deserialize, Serialize};
    use tokio::sync::{mpsc, watch};
    use wasm_bindgen::JsCast;
    use wasm_bindgen_futures::JsFuture;
    use web_sys::Response;

    use crate::{Error, Result};

    #[derive(Serialize, Deserialize)]
    pub(crate) struct Mesh {
        pub(crate) kind: MeshKind,
        pub(crate) path: String,
        pub(crate) data: String,
    }

    #[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub(crate) enum MeshKind {
        Obj,
        Other,
    }

    pub(crate) fn window() -> Result<web_sys::Window> {
        Ok(web_sys::window().ok_or("failed to get window")?)
    }

    pub(crate) async fn read_to_string(input_file: impl AsRef<str>) -> Result<String> {
        let promise = window()?.fetch_with_str(input_file.as_ref());

        let response: Response = JsFuture::from(promise).await?.dyn_into().unwrap();

        let s = JsFuture::from(response.text()?).await?;

        Ok(s.as_string().unwrap())
    }

    pub async fn read_urdf(input_file: impl AsRef<str>) -> Result<urdf_rs::Robot> {
        let s = read_to_string(input_file).await?;
        Ok(urdf_rs::read_from_string(&s)?)
    }

    pub async fn load_mesh(robot: &mut urdf_rs::Robot, urdf_path: impl AsRef<Path>) -> Result<()> {
        let urdf_path = urdf_path.as_ref();
        for geometry in robot.links.iter_mut().flat_map(|link| {
            link.visual
                .iter_mut()
                .map(|v| &mut v.geometry)
                .chain(link.collision.iter_mut().map(|c| &mut c.geometry))
        }) {
            if let urdf_rs::Geometry::Mesh { filename, .. } = geometry {
                let input_file =
                    if filename.starts_with("https://") || filename.starts_with("http://") {
                        filename.clone()
                    } else if filename.starts_with("package://") {
                        return Err(Error::from(format!(
                            "ros package ({}) is not supported in wasm",
                            filename
                        )));
                    } else {
                        // We don't use url::Url::path/set_path here, because
                        // urdf_path may be a relative path to a file bundled
                        // with the server. Path::with_file_name works for wasm
                        // where the separator is /, so we use it.
                        urdf_path
                            .with_file_name(&filename)
                            .to_str()
                            .unwrap()
                            .to_string()
                    };
                let (kind, data) = if input_file.ends_with(".obj") {
                    debug!("loading {}", input_file);
                    let data = read_to_string(&input_file).await?;
                    (MeshKind::Obj, data)
                } else {
                    (MeshKind::Other, String::new())
                };
                let new = serde_json::to_string(&Mesh {
                    kind,
                    path: filename.clone(),
                    data,
                })
                .unwrap();
                *filename = new;
            }
        }
        Ok(())
    }

    pub(crate) struct RobotModel {
        request_sender: mpsc::UnboundedSender<String>,
        response_receiver: watch::Receiver<Arc<urdf_rs::Robot>>,
        cache: Arc<urdf_rs::Robot>,
    }

    impl RobotModel {
        pub(crate) fn new(robot: urdf_rs::Robot) -> (Self, Arc<AtomicBool>) {
            let robot = Arc::new(robot);
            let (request_sender, mut request_receiver) = mpsc::unbounded_channel();
            let (response_sender, response_receiver) = watch::channel(robot.clone());
            let needs_reload = Arc::new(AtomicBool::new(false));
            let needs_reload_clone = needs_reload.clone();
            wasm_bindgen_futures::spawn_local(async move {
                while let Some(input_file) = request_receiver.recv().await {
                    match read_urdf(&input_file).await {
                        Ok(mut robot) => {
                            if let Err(e) = load_mesh(&mut robot, input_file).await {
                                error!("{}", e);
                            } else if response_sender.send(Arc::new(robot)).is_ok() {
                                needs_reload_clone.store(true, Ordering::Relaxed);
                            }
                        }
                        Err(e) => {
                            error!("{}", e);
                        }
                    }
                }
            });
            (
                Self {
                    request_sender,
                    response_receiver,
                    cache: robot,
                },
                needs_reload,
            )
        }

        pub(crate) fn get(&mut self) -> &urdf_rs::Robot {
            // Update cache
            // https://github.com/tokio-rs/tokio/issues/3666
            if let Some(Ok(())) = self.response_receiver.changed().now_or_never() {
                self.cache = self.response_receiver.borrow().clone();
            }
            &self.cache
        }

        pub(crate) fn request_reload(&self, input_file: impl AsRef<str>) {
            let _ = self.request_sender.send(input_file.as_ref().to_owned());
        }
    }
}
