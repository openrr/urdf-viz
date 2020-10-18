#[cfg(feature = "assimp")]
use crate::assimp_utils::*;
use crate::errors::Result;
use kiss3d::scene::SceneNode;
use nalgebra as na;
use std::path::Path;

#[cfg(feature = "assimp")]
pub fn load_mesh<P>(
    filename: P,
    scale: na::Vector3<f32>,
    opt_urdf_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    use_texture: bool,
) -> Result<SceneNode>
where
    P: AsRef<Path>,
{
    use log::*;
    let mut base = group.add_group();
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    let file_string = filename
        .as_ref()
        .to_str()
        .ok_or("failed to convert file string")?;
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
    let mut is_collada = false;
    if let Some(ext) = filename.as_ref().extension() {
        if ext == "dae" || ext == "DAE" {
            is_collada = true;
        }
    }
    // do not use texture, use only color in urdf file.
    if !use_texture || !is_collada {
        return Ok(base);
    }

    // Size of color and mesh are same, use each color for mesh
    if mesh_scenes.len() == colors.len() {
        let mut count = 0;
        for (mut mesh_scene, color) in mesh_scenes.into_iter().zip(colors.into_iter()) {
            mesh_scene.set_color(color[0], color[1], color[2]);
            // Is this OK?
            if count < textures.len() {
                let mut texture_path = filename.as_ref().to_path_buf();
                texture_path.set_file_name(textures[count].clone());
                debug!("using texture={}", texture_path.display());
                if texture_path.exists() {
                    mesh_scene.set_texture_from_file(&texture_path, texture_path.to_str().unwrap());
                }
            }
            count += 1;
        }
    } else {
        // When size of mesh and color mismatch, use only first color/texture for all meshes.
        // If no color found, use urdf color instead.
        for mut mesh_scene in mesh_scenes {
            if !textures.is_empty() {
                let mut texture_path = filename.as_ref().to_path_buf();
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

#[cfg(not(feature = "assimp"))]
pub fn load_mesh<P>(
    filename: P,
    _scale: na::Vector3<f32>,
    _opt_color: &Option<na::Point3<f32>>,
    _group: &mut SceneNode,
    _use_texture: bool,
) -> Result<SceneNode>
where
    P: AsRef<Path>,
{
    Err(crate::errors::Error::from(format!(
        "load mesh is disabled by feature: {}",
        filename.as_ref().display()
    )))
}
