use crate::errors::Result;
use k::nalgebra as na;
use kiss3d::scene::SceneNode;
use std::{cell::RefCell, rc::Rc};
use tracing::*;

#[cfg(feature = "assimp")]
#[cfg(not(target_arch = "wasm32"))]
fn load_mesh_assimp(
    file_string: &str,
    scale: na::Vector3<f32>,
    opt_urdf_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    use_texture: bool,
) -> Result<SceneNode> {
    use crate::assimp_utils::*;
    use std::{ffi::OsStr, path::Path};

    let filename = Path::new(file_string);

    let mut base = group.add_group();
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    let (meshes, textures, colors) =
        convert_assimp_scene_to_kiss3d_mesh(&importer.read_file(file_string)?);
    info!(
        "num mesh, texture, colors = {} {} {}",
        meshes.len(),
        textures.len(),
        colors.len()
    );
    let mesh_scenes = meshes
        .into_iter()
        .map(|mesh| {
            let mut scene = base.add_mesh(mesh, scale);
            // use urdf color as default
            if let Some(urdf_color) = *opt_urdf_color {
                scene.set_color(urdf_color[0], urdf_color[1], urdf_color[2]);
            }
            scene
        })
        .collect::<Vec<_>>();
    // use texture only for dae (collada)
    let is_collada = matches!(
        filename.extension().and_then(OsStr::to_str),
        Some("dae" | "DAE")
    );
    // do not use texture, use only color in urdf file.
    if !use_texture || !is_collada {
        return Ok(base);
    }

    // Size of color and mesh are same, use each color for mesh
    if mesh_scenes.len() == colors.len() {
        for (count, (mut mesh_scene, color)) in
            mesh_scenes.into_iter().zip(colors.into_iter()).enumerate()
        {
            mesh_scene.set_color(color[0], color[1], color[2]);
            // Is this OK?
            if count < textures.len() {
                let mut texture_path = filename.to_path_buf();
                texture_path.set_file_name(textures[count].clone());
                debug!("using texture={}", texture_path.display());
                if texture_path.exists() {
                    mesh_scene.set_texture_from_file(&texture_path, texture_path.to_str().unwrap());
                }
            }
        }
    } else {
        // When size of mesh and color mismatch, use only first color/texture for all meshes.
        // If no color found, use urdf color instead.
        for mut mesh_scene in mesh_scenes {
            if !textures.is_empty() {
                let mut texture_path = filename.to_path_buf();
                texture_path.set_file_name(textures[0].clone());
                debug!("texture={}", texture_path.display());
                if texture_path.exists() {
                    mesh_scene.set_texture_from_file(&texture_path, texture_path.to_str().unwrap());
                }
            }
            if !colors.is_empty() {
                let color = colors[0];
                mesh_scene.set_color(color[0], color[1], color[2]);
            }
        }
    }
    Ok(base)
}

#[cfg(not(target_arch = "wasm32"))]
pub fn load_mesh(
    filename: impl AsRef<str>,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    #[allow(unused_variables)] use_texture: bool,
) -> Result<SceneNode> {
    use std::{ffi::OsStr, path::Path};

    let file_string = filename.as_ref();

    #[cfg(feature = "assimp")]
    if !file_string.starts_with("https://") && !file_string.starts_with("http://") {
        return load_mesh_assimp(file_string, scale, opt_color, group, use_texture);
    }

    let filename = Path::new(file_string);

    match filename.extension().and_then(OsStr::to_str) {
        Some("obj" | "OBJ") => {
            if !file_string.starts_with("https://") && !file_string.starts_with("http://") {
                let mtl_path = filename.parent().unwrap_or_else(|| Path::new("."));
                debug!(
                    "load obj: path = {file_string}, mtl_path = {}",
                    mtl_path.display()
                );
                let mut base = group.add_obj(filename, mtl_path, scale);
                if let Some(color) = *opt_color {
                    base.set_color(color[0], color[1], color[2]);
                }
                Ok(base)
            } else {
                Ok(load_obj(
                    &String::from_utf8(fetch_or_read(file_string)?)?,
                    file_string,
                    scale,
                    opt_color,
                    group,
                ))
            }
        }
        Some("stl" | "STL") => {
            debug!("load stl: path = {file_string}");
            load_stl(&fetch_or_read(file_string)?, scale, opt_color, group)
        }
        Some("dae" | "DAE") => {
            debug!("load dae: path = {file_string}");
            load_collada(
                &String::from_utf8(fetch_or_read(file_string)?)?,
                scale,
                opt_color,
                group,
            )
        }
        _ => Err(crate::errors::Error::from(format!(
            "{file_string} is not supported, because assimp feature is disabled"
        ))),
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn fetch_or_read(filename: &str) -> Result<Vec<u8>> {
    use std::io::Read;

    const RESPONSE_SIZE_LIMIT: usize = 10 * 1_024 * 1_024;

    if filename.starts_with("https://") || filename.starts_with("http://") {
        let mut buf: Vec<u8> = vec![];
        ureq::get(filename)
            .call()
            .map_err(|e| crate::Error::Other(e.to_string()))?
            .into_reader()
            .take((RESPONSE_SIZE_LIMIT + 1) as u64)
            .read_to_end(&mut buf)?;
        if buf.len() > RESPONSE_SIZE_LIMIT {
            return Err(crate::errors::Error::Other(format!(
                "{filename} is too big"
            )));
        }
        Ok(buf)
    } else {
        Ok(std::fs::read(filename)?)
    }
}

/// NOTE: Unlike other platforms, the first argument should be the data loaded
/// by [`utils::load_mesh`](crate::utils::load_mesh), not the path.
#[cfg(target_arch = "wasm32")]
pub fn load_mesh(
    data: impl AsRef<str>,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    _use_texture: bool,
) -> Result<SceneNode> {
    use crate::utils::MeshKind;

    let data = crate::utils::Mesh::decode(data.as_ref())?;

    match data.kind {
        MeshKind::Obj => {
            debug!("load obj: path = {}", data.path);
            Ok(load_obj(
                data.string().unwrap(),
                &data.path,
                scale,
                opt_color,
                group,
            ))
        }
        MeshKind::Stl => {
            debug!("load stl: path = {}", data.path);
            load_stl(data.bytes().unwrap(), scale, opt_color, group)
        }
        MeshKind::Dae => {
            debug!("load dae: path = {}", data.path);
            load_collada(data.string().unwrap(), scale, opt_color, group)
        }
        MeshKind::Other => Err(crate::errors::Error::from(format!(
            "{} is not supported, because assimp feature is disabled",
            data.path
        ))),
    }
}

// Refs: https://github.com/sebcrozet/kiss3d/blob/73ff15dc40aaf994f3e8e240c23bb660be71a6cd/src/scene/scene_node.rs#L807-L866
fn load_obj(
    s: &str,
    filename: &str,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
) -> SceneNode {
    let tex = kiss3d::resource::TextureManager::get_global_manager(|tm| tm.get_default());
    let mat = kiss3d::resource::MaterialManager::get_global_manager(|mm| mm.get_default());

    // TODO: mtl
    let objs = kiss3d::loader::obj::parse(s, ".".as_ref(), filename);
    let mut root;

    let self_root = objs.len() == 1;
    let child_scale;

    if self_root {
        root = group.clone();
        child_scale = scale;
    } else {
        root = SceneNode::new(scale, na::one(), None);
        group.add_child(root.clone());
        child_scale = na::Vector3::from_element(1.0);
    }

    let mut last = None;
    for (_, mesh, _mtl) in objs {
        let mesh = Rc::new(RefCell::new(mesh));
        let object = kiss3d::scene::Object::new(mesh, 1.0, 1.0, 1.0, tex.clone(), mat.clone());

        // TODO: mtl

        last = Some(root.add_object(child_scale, na::one(), object));
    }

    let mut base = if self_root {
        last.expect("there was nothing on this obj file")
    } else {
        root
    };
    if let Some(color) = *opt_color {
        base.set_color(color[0], color[1], color[2]);
    }
    base
}

fn load_stl(
    bytes: &[u8],
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
) -> Result<SceneNode> {
    let stl = mesh_loader::stl::from_slice(bytes)?;
    let mesh = Rc::new(RefCell::new(kiss3d::resource::Mesh::new(
        stl.vertices.into_iter().map(Into::into).collect(),
        stl.faces
            .into_iter()
            .map(|f| {
                na::Point3::new(
                    f[0].try_into().unwrap(),
                    f[1].try_into().unwrap(),
                    f[2].try_into().unwrap(),
                )
            })
            .collect(),
        None,
        None,
        false,
    )));
    let mut base = group.add_mesh(mesh, scale);
    if let Some(color) = *opt_color {
        base.set_color(color[0], color[1], color[2]);
    }
    Ok(base)
}

fn load_collada(
    s: &str,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
) -> Result<SceneNode> {
    let mut base = group.add_group();
    let collada =
        mesh_loader::collada::from_str(s).map_err(|e| crate::Error::Other(e.to_string()))?;
    for mesh in collada.meshes {
        debug!(
            "name={},vertices={},normals={},texcoords0={},texcoords1={},faces={}",
            mesh.name,
            mesh.vertices.len(),
            mesh.normals.len(),
            mesh.texcoords[0].len(),
            mesh.texcoords[1].len(),
            mesh.faces.len()
        );
        let positions = mesh.vertices.iter().map(|&v| na::Point3::from(v)).collect();
        let faces = mesh
            .faces
            .iter()
            .map(|v| na::Point3::new(v[0] as u16, v[1] as u16, v[2] as u16))
            .collect();
        let mut scene = base.add_mesh(
            Rc::new(RefCell::new(kiss3d::resource::Mesh::new(
                positions, faces, None, None, false,
            ))),
            scale,
        );
        if let Some(color) = *opt_color {
            scene.set_color(color[0], color[1], color[2]);
        }

        // TODO: material
    }
    Ok(base)
}
