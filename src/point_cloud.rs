// Adapted from https://github.com/sebcrozet/kiss3d/blob/v0.30.0/examples/persistent_point_cloud.rs.

use std::{collections::HashMap, ops::Range};

use kiss3d::camera::Camera;
use kiss3d::context::Context;
use kiss3d::nalgebra as na;
use kiss3d::renderer::Renderer;
use kiss3d::resource::{
    AllocationType, BufferType, Effect, GPUVec, ShaderAttribute, ShaderUniform,
};
use na::{Matrix4, Point3};

pub(crate) struct PointCloudRenderer {
    shader: Effect,
    pos: ShaderAttribute<Point3<f32>>,
    color: ShaderAttribute<Point3<f32>>,
    proj: ShaderUniform<Matrix4<f32>>,
    view: ShaderUniform<Matrix4<f32>>,
    colored_points: GPUVec<Point3<f32>>,
    id_map: HashMap<String, Range<usize>>,
    point_size: f32,
}

impl PointCloudRenderer {
    pub(crate) fn new(point_size: f32) -> PointCloudRenderer {
        let mut shader = Effect::new_from_str(&vertex_shader_src(point_size), FRAGMENT_SHADER_SRC);

        shader.use_program();

        PointCloudRenderer {
            colored_points: GPUVec::new(Vec::new(), BufferType::Array, AllocationType::StreamDraw),
            id_map: HashMap::new(),
            pos: shader.get_attrib::<Point3<f32>>("position").unwrap(),
            color: shader.get_attrib::<Point3<f32>>("color").unwrap(),
            proj: shader.get_uniform::<Matrix4<f32>>("proj").unwrap(),
            view: shader.get_uniform::<Matrix4<f32>>("view").unwrap(),
            shader,
            point_size,
        }
    }

    pub(crate) fn insert(&mut self, id: Option<String>, points: &[[f32; 3]], colors: &[[f32; 3]]) {
        assert_eq!(points.len(), colors.len());
        if let Some(colored_points) = self.colored_points.data_mut() {
            if let Some(id) = id {
                if let Some(range) = self.id_map.remove(&id) {
                    let len = range.end - range.start;
                    colored_points.drain(range.clone());
                    for r in self.id_map.values_mut() {
                        if r.start > range.start {
                            r.start -= len;
                            r.end -= len;
                        }
                    }
                }

                let start = colored_points.len();
                colored_points.reserve(points.len() * 2);
                for (point, color) in points.iter().zip(colors) {
                    colored_points.push((*point).into());
                    colored_points.push((*color).into());
                }
                self.id_map.insert(id, start..colored_points.len());
            } else {
                colored_points.reserve(points.len() * 2);
                for (point, color) in points.iter().zip(colors) {
                    colored_points.push((*point).into());
                    colored_points.push((*color).into());
                }
            }
        }
    }
}

impl Renderer for PointCloudRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        if self.colored_points.len() == 0 {
            return;
        }

        self.shader.use_program();
        self.pos.enable();
        self.color.enable();

        camera.upload(pass, &mut self.proj, &mut self.view);

        self.color.bind_sub_buffer(&mut self.colored_points, 1, 1);
        self.pos.bind_sub_buffer(&mut self.colored_points, 1, 0);

        let ctxt = Context::get();
        ctxt.point_size(self.point_size);
        ctxt.draw_arrays(Context::POINTS, 0, (self.colored_points.len() / 2) as i32);

        self.pos.disable();
        self.color.disable();
    }
}

fn vertex_shader_src(point_size: f32) -> String {
    format!(
        "#version 100
attribute vec3 position;
attribute vec3 color;
varying   vec3 Color;
uniform   mat4 proj;
uniform   mat4 view;
void main() {{
    gl_Position = proj * view * vec4(position, 1.0);
    gl_PointSize = {point_size:.1};
    Color = color;
}}",
    )
}

const FRAGMENT_SHADER_SRC: &str = "#version 100
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
varying vec3 Color;
void main() {
    gl_FragColor = vec4(Color, 1.0);
}";
