use crate::{errors::Result, utils::is_url};
use k::nalgebra as na;
use kiss3d::scene::SceneNode;
use std::{cell::RefCell, ffi::OsStr, io, path::Path, rc::Rc};
use tracing::*;

#[cfg(feature = "assimp")]
#[cfg(not(target_family = "wasm"))]
fn load_mesh_assimp(
    file_string: &str,
    scale: na::Vector3<f32>,
    opt_urdf_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    use_texture: bool,
) -> Result<SceneNode> {
    use crate::assimp_utils::*;

    let filename = Path::new(file_string);

    let mut base = group.add_group();
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    importer.triangulate(true);
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

#[cfg(not(target_family = "wasm"))]
pub fn load_mesh(
    filename: impl AsRef<str>,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    use_texture: bool,
    #[allow(unused_variables)] use_assimp: bool,
) -> Result<SceneNode> {
    let file_string = filename.as_ref();

    #[cfg(feature = "assimp")]
    if use_assimp {
        if is_url(file_string) {
            let file = crate::utils::fetch_tempfile(file_string)?;
            let path = file.path().to_str().unwrap();
            return load_mesh_assimp(path, scale, opt_color, group, use_texture);
        }
        return load_mesh_assimp(file_string, scale, opt_color, group, use_texture);
    }

    let ext = Path::new(file_string).extension().and_then(OsStr::to_str);
    debug!("load {ext:?}: path = {file_string}");
    load_with_mesh_loader(
        &fetch_or_read(file_string)?,
        file_string,
        scale,
        opt_color,
        group,
        use_texture,
    )
}

#[cfg(not(target_family = "wasm"))]
fn fetch_or_read(filename: &str) -> Result<Vec<u8>> {
    use std::io::Read;

    const RESPONSE_SIZE_LIMIT: usize = 10 * 1_024 * 1_024;

    if is_url(filename) {
        let mut buf = Vec::with_capacity(128);
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
#[cfg(target_family = "wasm")]
pub fn load_mesh(
    data: impl AsRef<str>,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    _use_texture: bool,
    _use_assimp: bool,
) -> Result<SceneNode> {
    let data = crate::utils::Mesh::decode(data.as_ref())?;
    let ext = Path::new(&data.path).extension().and_then(OsStr::to_str);
    debug!("load {ext:?}: path = {}", data.path);
    let use_texture = false;
    load_with_mesh_loader(
        data.bytes().unwrap(),
        &data.path,
        scale,
        opt_color,
        group,
        use_texture,
    )
}

fn load_with_mesh_loader(
    bytes: &[u8],
    file_string: &str,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    mut use_texture: bool,
) -> Result<SceneNode> {
    let mut base = group.add_group();
    let mut loader = mesh_loader::Loader::default();
    use_texture &= !is_url(file_string);
    if use_texture {
        // TODO: Using fetch_or_read can support remote materials, but loading becomes slow.
        // #[cfg(not(target_family = "wasm"))]
        // {
        //     loader = loader
        //         .custom_reader(|p| fetch_or_read(p.to_str().unwrap()).map_err(io::Error::other));
        // }
    } else {
        loader = loader.custom_reader(|_| Err(io::Error::other("texture rendering disabled")));
    }
    let scene = loader.load_from_slice(bytes, file_string).map_err(|e| {
        if e.kind() == io::ErrorKind::Unsupported {
            crate::errors::Error::from(format!(
                "{file_string} is not supported, because assimp feature is disabled"
            ))
        } else {
            e.into()
        }
    })?;

    for (mesh, material) in scene.meshes.into_iter().zip(scene.materials) {
        let coords = mesh.vertices.into_iter().map(Into::into).collect();
        let faces = mesh
            .faces
            .into_iter()
            .map(|f| na::Point3::new(f[0], f[1], f[2]))
            .collect();

        let kiss3d_mesh = Rc::new(RefCell::new(kiss3d::resource::Mesh::new(
            coords, faces, None, None, false,
        )));
        let mut kiss3d_scene = base.add_mesh(kiss3d_mesh, scale);
        if use_texture {
            if let Some(color) = material.color.diffuse {
                kiss3d_scene.set_color(color[0], color[1], color[2]);
            }
            if let Some(path) = &material.texture.diffuse {
                let path_string = path.to_str().unwrap();
                // TODO: Using fetch_or_read can support remote materials, but loading becomes slow.
                // let buf = fetch_or_read(path_string)?;
                // kiss3d_scene.set_texture_from_memory(&buf, path_string);
                kiss3d_scene.set_texture_from_file(path, path_string);
            }
            if let Some(path) = &material.texture.ambient {
                let path_string = path.to_str().unwrap();
                // TODO: Using fetch_or_read can support remote materials, but loading becomes slow.
                // let buf = fetch_or_read(path_string)?;
                // kiss3d_scene.set_texture_from_memory(&buf, path_string);
                kiss3d_scene.set_texture_from_file(path, path_string);
            }
        }
    }
    if let Some(color) = *opt_color {
        base.set_color(color[0], color[1], color[2]);
    }
    Ok(base)
}
