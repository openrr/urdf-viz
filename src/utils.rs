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
    let (package_name, path) = path.split_once('/')?;
    let package_path = package_path.get(package_name)?;
    Some(format!(
        "{}/{path}",
        package_path.strip_suffix('/').unwrap_or(package_path),
    ))
}

pub(crate) fn is_url(path: &str) -> bool {
    path.starts_with("https://") || path.starts_with("http://")
}

#[cfg(not(target_family = "wasm"))]
mod native {
    use std::{
        collections::HashMap,
        ffi::OsStr,
        fs, mem,
        path::Path,
        sync::{Arc, Mutex},
    };

    use tracing::error;

    use crate::{utils::is_url, Result};

    fn read_urdf(path: &str, xacro_args: &[(String, String)]) -> Result<(urdf_rs::Robot, String)> {
        let urdf_text = if Path::new(path).extension().and_then(OsStr::to_str) == Some("xacro") {
            urdf_rs::utils::convert_xacro_to_urdf_with_args(path, xacro_args)?
        } else if is_url(path) {
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
        xacro_args: Vec<(String, String)>,
    }

    impl RobotModel {
        pub fn new(
            path: impl Into<String>,
            package_path: HashMap<String, String>,
            xacro_args: &[(String, String)],
        ) -> Result<Self> {
            let path = path.into();
            let (robot, urdf_text) = read_urdf(&path, xacro_args)?;
            Ok(Self {
                path,
                urdf_text: Arc::new(Mutex::new(urdf_text)),
                robot,
                package_path,
                xacro_args: xacro_args.to_owned(),
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
                xacro_args: Vec::new(),
            })
        }

        pub(crate) fn get(&mut self) -> &urdf_rs::Robot {
            &self.robot
        }

        pub(crate) fn reload(&mut self) {
            match read_urdf(&self.path, &self.xacro_args) {
                Ok((robot, text)) => {
                    self.robot = robot;
                    *self.urdf_text.lock().unwrap() = text;
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

    #[cfg(feature = "assimp")]
    /// http request -> write to tempfile -> return that file
    pub(crate) fn fetch_tempfile(url: &str) -> Result<tempfile::NamedTempFile> {
        use std::io::{Read, Write};

        const RESPONSE_SIZE_LIMIT: usize = 10 * 1_024 * 1_024;

        let mut buf: Vec<u8> = vec![];
        ureq::get(url)
            .call()
            .map_err(|e| crate::Error::Other(e.to_string()))?
            .into_reader()
            .take((RESPONSE_SIZE_LIMIT + 1) as u64)
            .read_to_end(&mut buf)?;
        if buf.len() > RESPONSE_SIZE_LIMIT {
            return Err(crate::Error::Other(format!("{url} is too big")));
        }
        let mut file = tempfile::NamedTempFile::new()?;
        file.write_all(&buf)?;
        Ok(file)
    }
}

#[cfg(target_family = "wasm")]
mod wasm {
    use std::{
        collections::HashMap,
        mem,
        path::Path,
        str,
        sync::{Arc, Mutex},
    };

    use base64::{engine::general_purpose::STANDARD as BASE64, Engine};
    use js_sys::Uint8Array;
    use serde::{Deserialize, Serialize};
    use tracing::debug;
    use wasm_bindgen::JsCast;
    use wasm_bindgen_futures::JsFuture;
    use web_sys::Response;

    use crate::{utils::is_url, Error, Result};

    #[derive(Serialize, Deserialize)]
    pub(crate) struct Mesh {
        pub(crate) path: String,
        data: MeshData,
    }

    impl Mesh {
        pub(crate) fn decode(data: &str) -> Result<Self> {
            let mut mesh: Self = serde_json::from_str(data).map_err(|e| e.to_string())?;
            match &mesh.data {
                MeshData::None => {}
                MeshData::Base64(s) => {
                    mesh.data = MeshData::Bytes(BASE64.decode(s).map_err(|e| e.to_string())?);
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
    }

    #[derive(Serialize, Deserialize)]
    enum MeshData {
        Base64(String),
        Bytes(Vec<u8>),
        None,
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
                let input_file = if is_url(filename) {
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

                debug!("loading {input_file}");
                let data = MeshData::Base64(BASE64.encode(read(&input_file).await?));

                let new = serde_json::to_string(&Mesh {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_replace_package_with_path() {
        let mut package_path = HashMap::new();
        package_path.insert("a".to_owned(), "path".to_owned());
        assert_eq!(
            replace_package_with_path("package://a/b/c", &package_path),
            Some("path/b/c".to_owned())
        );
        assert_eq!(
            replace_package_with_path("package://a", &package_path),
            None
        );
        assert_eq!(
            replace_package_with_path("package://b/b/c", &package_path),
            None
        );
        assert_eq!(replace_package_with_path("a", &package_path), None);
    }
}
