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
    use futures::future::FutureExt;
    use log::error;
    use std::sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    };
    use tokio::sync::{mpsc, watch};
    use wasm_bindgen::JsCast;
    use wasm_bindgen_futures::JsFuture;
    use web_sys::Response;

    use crate::Result;

    pub(crate) fn window() -> Result<web_sys::Window> {
        Ok(web_sys::window().ok_or("failed to get window")?)
    }

    pub async fn read_urdf(input_file: impl AsRef<str>) -> Result<urdf_rs::Robot> {
        let promise = window()?.fetch_with_str(input_file.as_ref());

        let response: Response = JsFuture::from(promise).await?.dyn_into().unwrap();

        let s = JsFuture::from(response.text()?).await?;

        Ok(urdf_rs::read_from_string(&s.as_string().unwrap())?)
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
                    match read_urdf(input_file).await {
                        Ok(robot) => {
                            if response_sender.send(Arc::new(robot)).is_ok() {
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
