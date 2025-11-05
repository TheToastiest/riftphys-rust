//! Minimal 3D renderer for riftphys (wgpu + winit 0.29).
//! - free-fly camera (360° yaw, ±89° pitch)
//! - instanced debug primitives: box, sphere (UV), capsule (cyl + hemispheres)
//! - simple directional light
//! - grid lines
use std::sync::Arc;
use winit::window::Window;

use std::f32::consts::PI;
use glam::{Mat4, Vec3, Quat};
use wgpu::util::DeviceExt;
    // use std::num::NonZeroU32;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pos: [f32; 3],
    nrm: [f32; 3],
}
impl Vertex {
    const ATTRS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0=>Float32x3, 1=>Float32x3];
    fn layout<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct InstanceRaw {
    // model matrix as 4xVec4 + color
    m0: [f32;4], m1: [f32;4], m2: [f32;4], m3: [f32;4],
    color: [f32;3], _pad: f32,
}
impl InstanceRaw {
    const ATTRS: [wgpu::VertexAttribute; 5] = [
        wgpu::VertexAttribute { shader_location: 2, offset: 0,  format: wgpu::VertexFormat::Float32x4 },
        wgpu::VertexAttribute { shader_location: 3, offset: 16, format: wgpu::VertexFormat::Float32x4 },
        wgpu::VertexAttribute { shader_location: 4, offset: 32, format: wgpu::VertexFormat::Float32x4 },
        wgpu::VertexAttribute { shader_location: 5, offset: 48, format: wgpu::VertexFormat::Float32x4 },
        wgpu::VertexAttribute { shader_location: 6, offset: 64, format: wgpu::VertexFormat::Float32x3 },
    ];
    fn layout<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: 16*4 + 16, // 4 vec4 (64) + color padded (16) = 80 bytes
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRS,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct CameraUBO {
    view_proj: [[f32;4];4],
    eye_ws:    [f32;4],
    light_dir: [f32;4],
}

pub struct Camera {
    pub eye: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub fov_y_radians: f32,
    pub aspect: f32,
    pub znear: f32,
    pub zfar: f32,
}
impl Camera {
    pub fn new() -> Self {
        Self {
            eye: Vec3::new(-6.0, 4.0, 6.0),
            yaw: 45_f32.to_radians(),
            pitch: (-15_f32).to_radians(),
            fov_y_radians: 60_f32.to_radians(),
            aspect: 16.0/9.0,
            znear: 0.05,
            zfar: 1_000.0,
        }
    }
    pub fn view(&self) -> Mat4 {
        let dir = Vec3::new(self.yaw.cos()*self.pitch.cos(), self.pitch.sin(), self.yaw.sin()*self.pitch.cos());
        Mat4::look_at_rh(self.eye, self.eye + dir, Vec3::Y)
    }
    pub fn proj(&self) -> Mat4 {
        Mat4::perspective_rh(self.fov_y_radians, self.aspect.max(0.01), self.znear, self.zfar)
    }
    pub fn view_proj(&self) -> Mat4 { self.proj() * self.view() }
    pub fn forward(&self) -> Vec3 {
        Vec3::new(self.yaw.cos()*self.pitch.cos(), self.pitch.sin(), self.yaw.sin()*self.pitch.cos()).normalize()
    }
    pub fn right(&self) -> Vec3 { self.forward().cross(Vec3::Y).normalize() }
    pub fn up(&self) -> Vec3 { self.right().cross(self.forward()).normalize() }
    pub fn add_yaw_pitch(&mut self, dyaw: f32, dpitch: f32) {
        self.yaw = (self.yaw + dyaw + 2.0*PI) % (2.0*PI);
        self.pitch = (self.pitch + dpitch).clamp((-89_f32).to_radians(), (89_f32).to_radians());
    }
}

pub enum ShapeKind {
    Box { hx: f32, hy: f32, hz: f32 },
    Sphere { r: f32 },
    CapsuleY { r: f32, hh: f32 }, // axis aligned along +Y
}
pub struct Instance {
    pub model: Mat4,
    pub color: Vec3,
    pub kind: ShapeKind,
}
impl Instance {
    fn raw(&self) -> InstanceRaw {
        let (m0, m1, m2, m3) = (self.model.x_axis, self.model.y_axis, self.model.z_axis, self.model.w_axis);
        InstanceRaw {
            m0: m0.to_array(), m1: m1.to_array(), m2: m2.to_array(), m3: m3.to_array(),
            color: self.color.to_array(), _pad: 0.0
        }
    }
}

struct Mesh {
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    icount: u32,
}

pub struct Renderer {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue:  wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    depth:  (wgpu::Texture, wgpu::TextureView),
    // pipelines
    mesh_pipe: wgpu::RenderPipeline,
    line_pipe: wgpu::RenderPipeline,
    // common buffers
    cam_buf: wgpu::Buffer,
    cam_bind: wgpu::BindGroup,
    cam_bgl: wgpu::BindGroupLayout,
    // unit meshes
    unit_box: Mesh,
    unit_sphere: Mesh,
    unit_capsule_y: Mesh,
    grid_lines: Mesh,
    pub camera: Camera,
}

impl Renderer {
    pub async fn new(window: Arc<Window>) -> Self {
        let size = window.inner_size();
        let instance = wgpu::Instance::default();

        // SAFE + yields Surface<'static> when given Arc<Window>
        let surface = instance.create_surface(window.clone())
            .expect("create wgpu surface");

        let adapter = instance.request_adapter(&wgpu::RequestAdapterOptions{
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: Some(&surface),
            force_fallback_adapter: false,
        }).await.expect("adapter");

        let (device, queue) = adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::downlevel_defaults().using_resolution(adapter.limits()),
            },
            None
        ).await.expect("device");
        let format = surface.get_capabilities(&adapter).formats
            .into_iter().find(|f| f.is_srgb()).unwrap_or(wgpu::TextureFormat::Bgra8UnormSrgb);

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format,
            width:  size.width.max(1),
            height: size.height.max(1),
            present_mode: wgpu::PresentMode::Fifo,
            alpha_mode:   wgpu::CompositeAlphaMode::Auto,
            view_formats: vec![format],
            desired_maximum_frame_latency: 2,

        };
        surface.configure(&device, &config);

        // depth
        let (depth_tex, depth_view) = make_depth(&device, config.width, config.height);

        // camera resources
        let cam_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("cam_bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let cam_buf = device.create_buffer(&wgpu::BufferDescriptor{
            label: Some("cam_ubo"),
            size: std::mem::size_of::<CameraUBO>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let cam_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("cam_bind"),
            layout: &cam_bgl,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: cam_buf.as_entire_binding() }],
        });

        // shaders
        let mesh_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor{
            label: Some("mesh.wgsl"),
            source: wgpu::ShaderSource::Wgsl(MESH_WGSL.into())
        });
        let line_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor{
            label: Some("line.wgsl"),
            source: wgpu::ShaderSource::Wgsl(LINE_WGSL.into())
        });

        // pipelines
        let mesh_pipe = {
            let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor{
                label: Some("mesh_pl"),
                bind_group_layouts: &[&cam_bgl],
                push_constant_ranges: &[],
            });
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor{
                label: Some("mesh_pipe"),
                layout: Some(&pl),
                // MESH PIPELINE
                vertex: wgpu::VertexState {
                    module: &mesh_shader,
                    entry_point: "vs_main",
                    buffers: &[Vertex::layout(), InstanceRaw::layout()],
                    compilation_options: wgpu::PipelineCompilationOptions::default(), // <-- add this
                },
                fragment: Some(wgpu::FragmentState {
                    module: &mesh_shader,
                    entry_point: "fs_main",
                    targets: &[Some(wgpu::ColorTargetState {
                        format,
                        blend: Some(wgpu::BlendState::REPLACE),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(), // <-- and this
                }),

                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FMT, depth_write_enabled: true, depth_compare: wgpu::CompareFunction::Less,
                    stencil: Default::default(), bias: Default::default()
                }),
                multisample: Default::default(),
                multiview: None
            })
        };
        let line_pipe = {
            let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor{
                label: Some("line_pl"),
                bind_group_layouts: &[&cam_bgl],
                push_constant_ranges: &[],
            });
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor{
                label: Some("line_pipe"),
                layout: Some(&pl),
                // LINE PIPELINE
                vertex: wgpu::VertexState {
                    module: &line_shader,
                    entry_point: "vs_main",
                    buffers: &[Vertex::layout()],
                    compilation_options: wgpu::PipelineCompilationOptions::default(), // <-- add
                },
                fragment: Some(wgpu::FragmentState {
                    module: &line_shader,
                    entry_point: "fs_main",
                    targets: &[Some(wgpu::ColorTargetState {
                        format,
                        blend: Some(wgpu::BlendState::REPLACE),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(), // <-- add
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::LineList,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FMT, depth_write_enabled: true, depth_compare: wgpu::CompareFunction::Less,
                    stencil: Default::default(), bias: Default::default()
                }),
                multisample: Default::default(),
                multiview: None
            })
        };

        // meshes
        let unit_box = upload_mesh(&device, &cube_unit());
        let unit_sphere = upload_mesh(&device, &uv_sphere(16, 16));
        let unit_capsule_y = upload_mesh(&device, &capsule_y(12, 6));

        // grid
        let grid_lines = upload_lines(&device, &grid_lines_xz(200, 1.0));

        Self  {
            surface, device, queue, config, depth: (depth_tex, depth_view),
            mesh_pipe, line_pipe,
            cam_buf, cam_bind, cam_bgl,
            unit_box, unit_sphere, unit_capsule_y, grid_lines,
            camera: Camera::new(),
        }
    }

    pub fn resize(&mut self, w: u32, h: u32) {
        if w == 0 || h == 0 { return; }
        self.config.width = w; self.config.height = h;
        self.surface.configure(&self.device, &self.config);
        self.depth = make_depth(&self.device, w, h);
        self.camera.aspect = w as f32 / h as f32;
    }

    pub fn render(&mut self, instances: &[Instance]) -> Result<(), wgpu::SurfaceError> {
        // update camera UBO
        let u = CameraUBO {
            view_proj: (self.camera.view_proj()).to_cols_array_2d(),
            eye_ws:    [self.camera.eye.x, self.camera.eye.y, self.camera.eye.z, 1.0],
            light_dir: [0.577, 0.577, 0.577, 0.0], // normalized (1,1,1)
        };
        self.queue.write_buffer(&self.cam_buf, 0, bytemuck::bytes_of(&u));

        let frame = self.surface.get_current_texture()?;
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor{ label: Some("encoder") });

        // background pass
        {
            // 1) Bucket instances by shape
            let mut boxes    = Vec::<InstanceRaw>::new();
            let mut spheres  = Vec::<InstanceRaw>::new();
            let mut capsules = Vec::<InstanceRaw>::new();

            for inst in instances {
                match inst.kind {
                    ShapeKind::Box { .. }      => boxes.push(inst.raw()),
                    ShapeKind::Sphere { .. }   => spheres.push(inst.raw()),
                    ShapeKind::CapsuleY { .. } => capsules.push(inst.raw()),
                }
            }

            // 2) Create instance buffers OUTSIDE the pass so they outlive rpass
            let boxes_buf = (!boxes.is_empty()).then(|| {
                self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("inst_buf.boxes"),
                    contents: bytemuck::cast_slice(&boxes),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            });
            let spheres_buf = (!spheres.is_empty()).then(|| {
                self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("inst_buf.spheres"),
                    contents: bytemuck::cast_slice(&spheres),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            });
            let capsules_buf = (!capsules.is_empty()).then(|| {
                self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("inst_buf.capsules"),
                    contents: bytemuck::cast_slice(&capsules),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            });

            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("main_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color{ r: 0.05, g: 0.06, b: 0.08, a: 1.0 }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth.1,
                    depth_ops: Some(wgpu::Operations {
                        load:  wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            // grid
            rpass.set_pipeline(&self.line_pipe);
            rpass.set_bind_group(0, &self.cam_bind, &[]);
            rpass.set_vertex_buffer(0, self.grid_lines.vbuf.slice(..));
            rpass.set_index_buffer(self.grid_lines.ibuf.slice(..), wgpu::IndexFormat::Uint32);
            rpass.draw_indexed(0..self.grid_lines.icount, 0, 0..1);

            // meshes
            rpass.set_pipeline(&self.mesh_pipe);
            rpass.set_bind_group(0, &self.cam_bind, &[]);

            // boxes
            if let Some(buf) = &boxes_buf {
                rpass.set_vertex_buffer(0, self.unit_box.vbuf.slice(..));
                rpass.set_vertex_buffer(1, buf.slice(..));
                rpass.set_index_buffer(self.unit_box.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                rpass.draw_indexed(0..self.unit_box.icount, 0, 0..(boxes.len() as u32));
            }

            // spheres
            if let Some(buf) = &spheres_buf {
                rpass.set_vertex_buffer(0, self.unit_sphere.vbuf.slice(..));
                rpass.set_vertex_buffer(1, buf.slice(..));
                rpass.set_index_buffer(self.unit_sphere.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                rpass.draw_indexed(0..self.unit_sphere.icount, 0, 0..(spheres.len() as u32));
            }

            // capsules
            if let Some(buf) = &capsules_buf {
                rpass.set_vertex_buffer(0, self.unit_capsule_y.vbuf.slice(..));
                rpass.set_vertex_buffer(1, buf.slice(..));
                rpass.set_index_buffer(self.unit_capsule_y.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                rpass.draw_indexed(0..self.unit_capsule_y.icount, 0, 0..(capsules.len() as u32));
            }
            //
            // draw_batch(&self.unit_box, &mut boxes);
            // draw_batch(&self.unit_sphere, &mut spheres);
            // draw_batch(&self.unit_capsule_y, &mut capsules);
        }

        self.queue.submit(Some(encoder.finish()));
        frame.present();
        Ok(())
    }
}

/* ---------- public helpers ---------- */

pub fn trs(position: Vec3, rotation: Quat, scale: Vec3) -> Mat4 {
    Mat4::from_translation(position) * Mat4::from_quat(rotation) * Mat4::from_scale(scale)
}

/* ---------- internal helpers / mesh gen ---------- */

const DEPTH_FMT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

fn make_depth(device: &wgpu::Device, w: u32, h: u32) -> (wgpu::Texture, wgpu::TextureView) {
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("depth"),
        size: wgpu::Extent3d{ width: w.max(1), height: h.max(1), depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FMT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
    (tex, view)
}

fn upload_mesh(device: &wgpu::Device, (vtx, idx): &(Vec<Vertex>, Vec<u32>)) -> Mesh {
    let vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("mesh_vb"),
        contents: bytemuck::cast_slice(vtx.as_slice()),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let ibuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("mesh_ib"),
        contents: bytemuck::cast_slice(idx.as_slice()),
        usage: wgpu::BufferUsages::INDEX,
    });
    Mesh { vbuf, ibuf, icount: idx.len() as u32 }
}
fn upload_lines(device: &wgpu::Device, (vtx, idx): &(Vec<Vertex>, Vec<u32>)) -> Mesh {
    upload_mesh(device, &(vtx.clone(), idx.clone()))
}

fn cube_unit() -> (Vec<Vertex>, Vec<u32>) {
    // unit cube centered at origin, size 2 (scale later)
    let p = [
        // +X
        (Vec3::new(1.,-1.,-1.), Vec3::X), (Vec3::new(1.,-1., 1.), Vec3::X),
        (Vec3::new(1., 1., 1.), Vec3::X), (Vec3::new(1., 1.,-1.), Vec3::X),
        // -X
        (Vec3::new(-1.,-1., 1.), -Vec3::X), (Vec3::new(-1.,-1.,-1.), -Vec3::X),
        (Vec3::new(-1., 1.,-1.), -Vec3::X), (Vec3::new(-1., 1., 1.), -Vec3::X),
        // +Y
        (Vec3::new(-1., 1.,-1.), Vec3::Y), (Vec3::new(1., 1.,-1.), Vec3::Y),
        (Vec3::new(1., 1., 1.), Vec3::Y), (Vec3::new(-1., 1., 1.), Vec3::Y),
        // -Y
        (Vec3::new(-1.,-1., 1.), -Vec3::Y), (Vec3::new(1.,-1., 1.), -Vec3::Y),
        (Vec3::new(1.,-1.,-1.), -Vec3::Y), (Vec3::new(-1.,-1.,-1.), -Vec3::Y),
        // +Z
        (Vec3::new(-1.,-1., 1.), Vec3::Z), (Vec3::new(-1., 1., 1.), Vec3::Z),
        (Vec3::new(1., 1., 1.), Vec3::Z), (Vec3::new(1.,-1., 1.), Vec3::Z),
        // -Z
        (Vec3::new(1.,-1.,-1.), -Vec3::Z), (Vec3::new(1., 1.,-1.), -Vec3::Z),
        (Vec3::new(-1., 1.,-1.), -Vec3::Z), (Vec3::new(-1.,-1.,-1.), -Vec3::Z),
    ];
    let mut v = Vec::with_capacity(24);
    for (pos, nrm) in p { v.push(Vertex{ pos: pos.to_array(), nrm: nrm.to_array() }); }
    let mut i = Vec::new();
    for f in 0..6 {
        let b = (f*4) as u32;
        i.extend_from_slice(&[b, b+1, b+2,  b, b+2, b+3]);
    }
    (v, i)
}

fn uv_sphere(segments: u32, rings: u32) -> (Vec<Vertex>, Vec<u32>) {
    let seg = segments.max(3);
    let rng = rings.max(2);
    let mut v = Vec::new();
    for y in 0..=rng {
        let vty = y as f32 / rng as f32;
        let phi = vty * PI;
        for x in 0..=seg {
            let vtx = x as f32 / seg as f32;
            let theta = vtx * 2.0*PI;
            let n = Vec3::new(theta.cos()*phi.sin(), phi.cos(), theta.sin()*phi.sin());
            v.push(Vertex{ pos: n.to_array(), nrm: n.to_array() });
        }
    }
    let mut i = Vec::new();
    let stride = seg + 1;
    for y in 0..rng {
        for x in 0..seg {
            let a = y*stride + x;
            let b = a + 1;
            let c = a + stride;
            let d = c + 1;
            i.extend_from_slice(&[a, c, b,  b, c, d].map(|k| k as u32));
        }
    }
    (v, i)
}

fn capsule_y(segments: u32, rings_half: u32) -> (Vec<Vertex>, Vec<u32>) {
    // Cylinder of height 2*hh + hemispheres (unit radius, unit hh = 1); scale later
    let seg = segments.max(6);
    let rng = rings_half.max(3);
    let mut v = Vec::new();
    let mut i = Vec::new();

    // cylinder
    let y0 = -1.0_f32;
    let y1 =  1.0_f32;
    for y in [y0, y1] {
        for x in 0..=seg {
            let t = (x as f32 / seg as f32) * 2.0*PI;
            let n = Vec3::new(t.cos(), 0.0, t.sin());
            v.push(Vertex{ pos: [n.x, y, n.z], nrm: [n.x, 0.0, n.z] });
        }
    }
    let stride = seg + 1;
    for x in 0..seg {
        let a = x;
        let b = x + 1;
        let c = stride + x;
        let d = stride + x + 1;
        i.extend_from_slice(&[a, c, b,  b, c, d].map(|k| k as u32));
    }

    // hemispheres
    let base_top = v.len() as u32;
    for hemi in 0..2 {
        let sign = if hemi==0 { -1.0 } else { 1.0 };
        let y_center = if hemi==0 { y0 } else { y1 };
        for ry in 1..=rng {
            let vty = ry as f32 / (rng as f32 + 1.0);
            let phi = vty * 0.5*PI;
            let cy = y_center + sign * phi.sin();
            let r = phi.cos();
            for x in 0..=seg {
                let t = (x as f32 / seg as f32) * 2.0*PI;
                let n = Vec3::new(t.cos()*r, sign*phi.sin(), t.sin()*r).normalize();
                let pos = Vec3::new(t.cos()*r, cy, t.sin()*r);
                v.push(Vertex{ pos: pos.to_array(), nrm: n.to_array() });
            }
        }
        // indices for hemi
        let rows = rng;
        for r in 0..rows {
            for x in 0..seg {
                let a = base_top + (r*stride + x) as u32;
                let b = a + 1;
                let c = a + stride as u32;
                let d = c + 1;
                i.extend_from_slice(&[a, c, b,  b, c, d]);
            }
        }
        // next hemi base
        // (no cap triangles at the poles; cyl covers connection)
    }

    (v, i)
}

fn grid_lines_xz(half: i32, step: f32) -> (Vec<Vertex>, Vec<u32>) {
    let mut v = Vec::new();
    let mut idx = Vec::new();
    let mut push_line = |a: Vec3, b: Vec3| {
        let base = v.len() as u32;
        v.push(Vertex{ pos: a.to_array(), nrm: [0.0, 1.0, 0.0] });
        v.push(Vertex{ pos: b.to_array(), nrm: [0.0, 1.0, 0.0] });
        idx.extend_from_slice(&[base, base+1]);
    };
    for i in -half..=half {
        let x = i as f32 * step;
        push_line(Vec3::new(x, 0.0, -half as f32 * step), Vec3::new(x, 0.0, half as f32 * step));
    }
    for k in -half..=half {
        let z = k as f32 * step;
        push_line(Vec3::new(-half as f32 * step, 0.0, z), Vec3::new(half as f32 * step, 0.0, z));
    }
    (v, idx)
}

/* ---------- WGSL ---------- */

const MESH_WGSL: &str = r#"
struct Camera {
  view_proj : mat4x4<f32>,
  eye_ws    : vec4<f32>,
  light_dir : vec4<f32>,
};
@group(0) @binding(0) var<uniform> uCam : Camera;

struct VsIn {
  @location(0) pos : vec3<f32>,
  @location(1) nrm : vec3<f32>,
  // instanced:
  @location(2) i_m0 : vec4<f32>,
  @location(3) i_m1 : vec4<f32>,
  @location(4) i_m2 : vec4<f32>,
  @location(5) i_m3 : vec4<f32>,
  @location(6) i_color : vec3<f32>,
};
struct VsOut {
  @builtin(position) pos_cs : vec4<f32>,
  @location(0) n_ws : vec3<f32>,
  @location(1) p_ws : vec3<f32>,
  @location(2) color : vec3<f32>,
};
@vertex
fn vs_main(v: VsIn) -> VsOut {
  let M = mat4x4<f32>(v.i_m0, v.i_m1, v.i_m2, v.i_m3);
  let p_ws = (M * vec4<f32>(v.pos, 1.0)).xyz;
  // upper-left 3x3 for normal (assumes uniform scale)
  let m0 = M[0].xyz;
let m1 = M[1].xyz;
let m2 = M[2].xyz;

let sx = length(m0);
let sy = length(m1);
let sz = length(m2);

// rotation columns (orthonormal)
let r0 = m0 / sx;
let r1 = m1 / sy;
let r2 = m2 / sz;

// normal matrix = R * inv(S)
let n_ws = normalize(mat3x3<f32>(r0 * (1.0 / sx), r1 * (1.0 / sy), r2 * (1.0 / sz)) * v.nrm);
  var o: VsOut;
  o.pos_cs = uCam.view_proj * vec4<f32>(p_ws, 1.0);
  o.n_ws = n_ws;
  o.p_ws = p_ws;
  o.color = v.i_color;
  return o;
}

@fragment
fn fs_main(i: VsOut) -> @location(0) vec4<f32> {
  let L = normalize(uCam.light_dir.xyz);
  let N = normalize(i.n_ws);
  let V = normalize(uCam.eye_ws.xyz - i.p_ws);
  let H = normalize(L + V);
  let ndotl = max(dot(N,L), 0.0);
  let diff = ndotl;
  let spec = pow(max(dot(N,H), 0.0), 32.0);
  let c = i.color * (0.08 + 0.92 * diff) + vec3<f32>(spec);
  return vec4<f32>(c, 1.0);
}
"#;

const LINE_WGSL: &str = r#"
struct Camera {
  view_proj : mat4x4<f32>,
  eye_ws    : vec4<f32>,
  light_dir : vec4<f32>,
};
@group(0) @binding(0) var<uniform> uCam : Camera;

struct VsIn {
  @location(0) pos : vec3<f32>,
  @location(1) nrm : vec3<f32>,
};
struct VsOut {
  @builtin(position) pos_cs : vec4<f32>,
  @location(0) color : vec3<f32>,
};
@vertex
fn vs_main(v: VsIn) -> VsOut {
  var o: VsOut;
  o.pos_cs = uCam.view_proj * vec4<f32>(v.pos, 1.0);
  // grid tint by normal (y=1) -> bluish
  o.color = vec3<f32>(0.2, 0.3, 0.45);
  return o;
}
@fragment
fn fs_main(i: VsOut) -> @location(0) vec4<f32> {
  return vec4<f32>(i.color, 1.0);
}
"#;
