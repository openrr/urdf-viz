#[cfg(not(target_arch = "wasm32"))]
pub use native::*;
#[cfg(target_arch = "wasm32")]
pub use wasm::*;

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use std::{ffi::OsStr, fs, path::Path, sync::Arc};

    use parking_lot::Mutex;
    use tracing::error;

    use crate::Result;

    fn read_urdf(path: impl AsRef<Path>) -> Result<(urdf_rs::Robot, String)> {
        let urdf_text = if path.as_ref().extension().and_then(OsStr::to_str) == Some("xacro") {
            urdf_rs::utils::convert_xacro_to_urdf(path)?
        } else {
            fs::read_to_string(path)?
        };
        let robot = urdf_rs::read_from_string(&urdf_text)?;
        Ok((robot, urdf_text))
    }

    #[derive(Debug)]
    pub struct RobotModel {
        pub(crate) path: String,
        pub(crate) urdf_text: Arc<Mutex<String>>,
        robot: urdf_rs::Robot,
    }

    impl RobotModel {
        pub fn new(path: impl Into<String>) -> Result<Self> {
            let path = path.into();
            let (robot, urdf_text) = read_urdf(&path)?;
            Ok(Self {
                path,
                urdf_text: Arc::new(Mutex::new(urdf_text)),
                robot,
            })
        }

        pub async fn from_text(
            path: impl Into<String>,
            urdf_text: impl Into<String>,
        ) -> Result<Self> {
            let path = path.into();
            let urdf_text = urdf_text.into();
            let robot = urdf_rs::read_from_string(&urdf_text)?;
            Ok(Self {
                path,
                urdf_text: Arc::new(Mutex::new(urdf_text)),
                robot,
            })
        }

        pub(crate) fn get(&mut self) -> &urdf_rs::Robot {
            &self.robot
        }

        pub(crate) fn reload(&mut self) {
            match read_urdf(&self.path) {
                Ok((robot, text)) => {
                    self.robot = robot;
                    *self.urdf_text.lock() = text;
                }
                Err(e) => {
                    error!("{e}");
                }
            }
        }
    }
}

#[cfg(target_arch = "wasm32")]
mod wasm {
    use std::{path::Path, str, sync::Arc};

    use js_sys::Uint8Array;
    use parking_lot::Mutex;
    use serde::{Deserialize, Serialize};
    use tracing::debug;
    use wasm_bindgen::JsCast;
    use wasm_bindgen_futures::JsFuture;
    use web_sys::Response;

    use crate::{Error, Result};

    #[derive(Serialize, Deserialize)]
    pub(crate) struct Mesh {
        pub(crate) kind: MeshKind,
        pub(crate) path: String,
        data: MeshData,
    }

    impl Mesh {
        pub(crate) fn decode(data: &str) -> Result<Self> {
            let mut mesh: Self = serde_json::from_str(data).map_err(|e| e.to_string())?;
            match &mesh.data {
                MeshData::None => {}
                MeshData::Base64(s) => {
                    mesh.data = MeshData::Bytes(base64::decode(s).map_err(|e| e.to_string())?);
                }
                MeshData::Bytes(_) => unreachable!(),
            }
            Ok(mesh)
        }

        pub(crate) fn bytes(&self) -> Option<&[u8]> {
            match &self.data {
                MeshData::None => None,
                MeshData::Bytes(bytes) => Some(bytes),
                MeshData::Base64(_) => unreachable!(),
            }
        }

        pub(crate) fn string(&self) -> Option<&str> {
            match &self.data {
                MeshData::None => None,
                MeshData::Bytes(s) => str::from_utf8(s).ok(),
                MeshData::Base64(..) => unreachable!(),
            }
        }
    }

    #[derive(Serialize, Deserialize)]
    enum MeshData {
        Base64(String),
        Bytes(Vec<u8>),
        None,
    }

    #[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub(crate) enum MeshKind {
        Obj,
        Stl,
        Dae,
        Other,
    }

    pub fn window() -> Result<web_sys::Window> {
        Ok(web_sys::window().ok_or("failed to get window")?)
    }

    async fn fetch(input_file: &str) -> Result<Response> {
        let promise = window()?.fetch_with_str(input_file);

        let response = JsFuture::from(promise)
            .await?
            .dyn_into::<Response>()
            .unwrap();

        Ok(response)
    }

    pub async fn read_to_string(input_file: impl AsRef<str>) -> Result<String> {
        let promise = fetch(input_file.as_ref()).await?.text()?;

        let s = JsFuture::from(promise).await?;

        Ok(s.as_string()
            .ok_or_else(|| format!("{} is not string", input_file.as_ref()))?)
    }

    pub async fn read(input_file: impl AsRef<str>) -> Result<Vec<u8>> {
        let promise = fetch(input_file.as_ref()).await?.array_buffer()?;

        let bytes = JsFuture::from(promise).await?;

        Ok(Uint8Array::new(&bytes).to_vec())
    }

    async fn read_urdf(input_file: impl AsRef<str>) -> Result<(urdf_rs::Robot, String)> {
        let s = read_to_string(input_file).await?;
        let robot = urdf_rs::read_from_string(&s)?;
        Ok((robot, s))
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
                let input_file = if filename.starts_with("https://")
                    || filename.starts_with("http://")
                    || filename.starts_with("file://")
                {
                    filename.clone()
                } else if filename.starts_with("package://") {
                    return Err(Error::from(format!(
                        "ros package ({filename}) is not supported in wasm",
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

                let kind = if input_file.ends_with(".obj") || input_file.ends_with(".OBJ") {
                    MeshKind::Obj
                } else if input_file.ends_with(".stl") || input_file.ends_with(".STL") {
                    MeshKind::Stl
                } else if input_file.ends_with(".dae") || input_file.ends_with(".DAE") {
                    MeshKind::Dae
                } else {
                    MeshKind::Other
                };

                let data = if kind != MeshKind::Other {
                    debug!("loading {input_file}");
                    MeshData::Base64(base64::encode(read(&input_file).await?))
                } else {
                    MeshData::None
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

    #[derive(Debug)]
    pub struct RobotModel {
        pub(crate) path: String,
        pub(crate) urdf_text: Arc<Mutex<String>>,
        robot: urdf_rs::Robot,
    }

    impl RobotModel {
        pub async fn new(path: impl Into<String>) -> Result<Self> {
            let path = path.into();
            let (mut robot, urdf_text) = read_urdf(&path).await?;
            load_mesh(&mut robot, &path).await?;
            Ok(Self {
                path,
                urdf_text: Arc::new(Mutex::new(urdf_text)),
                robot,
            })
        }

        pub async fn from_text(
            path: impl Into<String>,
            urdf_text: impl Into<String>,
        ) -> Result<Self> {
            let path = path.into();
            let urdf_text = urdf_text.into();
            let mut robot = urdf_rs::read_from_string(&urdf_text)?;
            load_mesh(&mut robot, &path).await?;
            Ok(Self {
                path,
                urdf_text: Arc::new(Mutex::new(urdf_text)),
                robot,
            })
        }

        pub(crate) fn get(&mut self) -> &urdf_rs::Robot {
            &self.robot
        }
    }
}
