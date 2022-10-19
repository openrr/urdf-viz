use std::collections::HashMap;

#[cfg(not(target_family = "wasm"))]
pub use native::*;
#[cfg(target_family = "wasm")]
pub use wasm::*;

pub(crate) fn replace_package_with_path(
    filename: &str,
    package_path: &HashMap<String, String>,
) -> Option<String> {
    let path = filename.strip_prefix("package://")?;
    let (package_name, path) = path.split_once('/').unwrap_or((path, ""));
    let package_path = package_path.get(package_name)?;
    Some(format!(
        "{}/{path}",
        package_path.strip_suffix('/').unwrap_or(package_path),
    ))
}

#[cfg(not(target_family = "wasm"))]
mod native {
    use std::{collections::HashMap, ffi::OsStr, fs, mem, path::Path, sync::Arc};

    use parking_lot::Mutex;
    use tracing::error;

    use crate::Result;

    fn read_urdf(path: &str) -> Result<(urdf_rs::Robot, String)> {
        let urdf_text = if Path::new(path).extension().and_then(OsStr::to_str) == Some("xacro") {
            urdf_rs::utils::convert_xacro_to_urdf(path)?
        } else if path.starts_with("https://") || path.starts_with("http://") {
            ureq::get(path)
                .call()
                .map_err(|e| crate::Error::Other(e.to_string()))?
                .into_string()?
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
        package_path: HashMap<String, String>,
    }

    impl RobotModel {
        pub fn new(path: impl Into<String>, package_path: HashMap<String, String>) -> Result<Self> {
            let path = path.into();
            let (robot, urdf_text) = read_urdf(&path)?;
            Ok(Self {
                path,
                urdf_text: Arc::new(Mutex::new(urdf_text)),
                robot,
                package_path,
            })
        }

        pub async fn from_text(
            path: impl Into<String>,
            urdf_text: impl Into<String>,
            package_path: HashMap<String, String>,
        ) -> Result<Self> {
            let path = path.into();
            let urdf_text = urdf_text.into();
            let robot = urdf_rs::read_from_string(&urdf_text)?;
            Ok(Self {
                path,
                urdf_text: Arc::new(Mutex::new(urdf_text)),
                robot,
                package_path,
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

        pub(crate) fn take_package_path_map(&mut self) -> HashMap<String, String> {
            mem::take(&mut self.package_path)
        }
    }
}

#[cfg(target_family = "wasm")]
mod wasm {
    use std::{collections::HashMap, mem, path::Path, str, sync::Arc};

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

    pub async fn load_mesh(
        robot: &mut urdf_rs::Robot,
        urdf_path: impl AsRef<Path>,
        package_path: &HashMap<String, String>,
    ) -> Result<()> {
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
                {
                    filename.clone()
                } else if filename.starts_with("package://") {
                    crate::utils::replace_package_with_path(filename, package_path).ok_or_else(||
                        format!(
                            "ros package ({filename}) is not supported in wasm; consider using `package-path[]` URL parameter",
                        ))?
                } else if filename.starts_with("file://") {
                    return Err(Error::from(format!(
                        "local file ({filename}) is not supported in wasm",
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
        package_path: HashMap<String, String>,
    }

    impl RobotModel {
        pub async fn new(
            path: impl Into<String>,
            package_path: HashMap<String, String>,
        ) -> Result<Self> {
            let path = path.into();
            let urdf_text = read_to_string(&path).await?;
            Self::from_text(path, urdf_text, package_path).await
        }

        pub async fn from_text(
            path: impl Into<String>,
            urdf_text: impl Into<String>,
            package_path: HashMap<String, String>,
        ) -> Result<Self> {
            let path = path.into();
            let urdf_text = urdf_text.into();
            let mut robot = urdf_rs::read_from_string(&urdf_text)?;
            load_mesh(&mut robot, &path, &package_path).await?;
            Ok(Self {
                path,
                urdf_text: Arc::new(Mutex::new(urdf_text)),
                robot,
                package_path,
            })
        }

        pub(crate) fn get(&mut self) -> &urdf_rs::Robot {
            &self.robot
        }

        pub(crate) fn take_package_path_map(&mut self) -> HashMap<String, String> {
            mem::take(&mut self.package_path)
        }
    }
}
