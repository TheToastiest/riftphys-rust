use wgpu::util::DeviceExt;
use bytemuck::{Pod, Zeroable};
use std::sync::mpsc;

use crate::{Ray, VoxelChunk};

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct RayGpu {
    o: [f32; 4],        // origin.xyz, padding
    d_tmax: [f32; 4],   // dir.xyz,  tmax in w
}
impl From<Ray> for RayGpu {
    fn from(r: Ray) -> Self {
        RayGpu { o: [r.o.x, r.o.y, r.o.z, 0.0], d_tmax: [r.d.x, r.d.y, r.d.z, r.tmax] }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Default)]
pub struct HitGpu {
    pub t: f32,
    pub nx: f32,
    pub ny: f32,
    pub nz: f32,
    pub cx: u32,
    pub cy: u32,
    pub cz: u32,
    pub hit: u32, // 0/1
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Default)]
struct Config {
    dims:   [u32; 4],  // x,y,z, pad
    params: [f32; 4],  // x=voxel_size, y/z/w pad
}
pub struct GpuRaycaster {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    bind_layout: wgpu::BindGroupLayout,
    ts_mode: TsMode,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum TsMode {
    None,
    Encoder, // needs TIMESTAMP_QUERY + TIMESTAMP_QUERY_INSIDE_ENCODERS
    Pass,    // needs TIMESTAMP_QUERY only
}

impl GpuRaycaster {
    pub fn new() -> Self {
        let instance = wgpu::Instance::default();
        let adapter = pollster::block_on(instance.request_adapter(
            &wgpu::RequestAdapterOptions::default(),
        ))
            .expect("wgpu adapter");

        // Feature detection
        let feats = adapter.features();
        let has_ts = feats.contains(wgpu::Features::TIMESTAMP_QUERY);
        let has_ts_enc = feats.contains(wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS);
        let ts_mode = if has_ts && has_ts_enc {
            TsMode::Encoder
        } else if has_ts {
            TsMode::Pass
        } else {
            TsMode::None
        };

        // Request only what we actually can use
        let mut req = wgpu::Features::empty();
        if has_ts { req |= wgpu::Features::TIMESTAMP_QUERY; }
        if has_ts_enc { req |= wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS; }

        let (device, queue) = pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("vox-device"),
                required_features: req,
                required_limits: wgpu::Limits::default(),
            },
            None,
        ))
            .expect("wgpu device");

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("vox-raycast"),
            source: wgpu::ShaderSource::Wgsl(include_str!("wgsl/raycast.wgsl").into()),
        });

        let bind_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("vox-layout"),
            entries: &[
                // 0: voxel bitset (u32 words)
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // 1: config
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // 2: rays
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // 3: hits
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("vox-pl"),
            bind_group_layouts: &[&bind_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("vox-pipe"),
            layout: Some(&pipeline_layout),
            module: &shader,
            entry_point: "main",
            compilation_options: wgpu::PipelineCompilationOptions::default(),
        });

        Self { device, queue, pipeline, bind_layout, ts_mode }
    }

    /// Dispatch a batch of rays. Returns (kernel_ms, hits_out).
    pub fn raycast_batch(&self, chunk: &VoxelChunk, rays: &[Ray]) -> (f64, Vec<HitGpu>) {
        // BitVec -> Vec<u32> (LSB-first per word)
        let nbits = chunk.solid.len();
        let mut words = vec![0u32; (nbits + 31) / 32];
        for i in 0..nbits {
            if chunk.solid[i] {
                let w = i / 32;
                let b = (i % 32) as u32;
                words[w] |= 1u32 << b;
            }
        }

        let cfg = Config {
            dims:   [chunk.dims.x, chunk.dims.y, chunk.dims.z, 0],
            params: [chunk.voxel_size, 0.0, 0.0, 0.0],
        };


        let rays_gpu: Vec<RayGpu> = rays.iter().copied().map(Into::into).collect();

        let vox_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("voxels"),
            contents: bytemuck::cast_slice(&words),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let cfg_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cfg"),
            contents: bytemuck::bytes_of(&cfg),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let ray_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("rays"),
            contents: bytemuck::cast_slice(&rays_gpu),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let hit_sz = (std::mem::size_of::<HitGpu>() * rays.len()) as u64;
        let hit_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("hits"),
            size: hit_sz,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let readback = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("hits-readback"),
            size: hit_sz,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("vox-bind"),
            layout: &self.bind_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: vox_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: cfg_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: ray_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: hit_buf.as_entire_binding() },
            ],
        });

        // Optional timestamp setup (encoder or pass)
        let (qs_opt, ts_buf_opt, ts_rb_opt) = match self.ts_mode {
            TsMode::None => (None, None, None),
            _ => {
                let qs = self.device.create_query_set(&wgpu::QuerySetDescriptor {
                    label: Some("ts"),
                    ty: wgpu::QueryType::Timestamp,
                    count: 2,
                });
                // GPU-only resolve target (not mappable)
                let ts_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("ts-buf"),
                    size: 16, // 2 * u64
                    usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
                    mapped_at_creation: false,
                });
                // CPU readback staging
                let ts_readback = self.device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("ts-readback"),
                    size: 16,
                    usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                (Some(qs), Some(ts_buf), Some(ts_readback))
            }
        };


        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("vox-enc") });

        // Either encoder-level stamps, or pass-level stamps, or none.
        if self.ts_mode == TsMode::Encoder {
            let qs = qs_opt.as_ref().unwrap();
            encoder.write_timestamp(qs, 0);
            {
                let mut cpass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("vox-cp"),
                    timestamp_writes: None,
                });
                cpass.set_pipeline(&self.pipeline);
                cpass.set_bind_group(0, &bind, &[]);
                let wg = ((rays.len() as u32) + 63) / 64;
                cpass.dispatch_workgroups(wg, 1, 1);
            }
            let qs = qs_opt.as_ref().unwrap();
            let ts_buf = ts_buf_opt.as_ref().unwrap();
            encoder.write_timestamp(qs, 1);
            encoder.resolve_query_set(qs, 0..2, ts_buf, 0);
            let ts_readback = ts_rb_opt.as_ref().unwrap();
            encoder.copy_buffer_to_buffer(ts_buf, 0, ts_readback, 0, 16);
        } else {
            // TsMode::Pass or TsMode::None
            let mut cpass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("vox-cp"),
                timestamp_writes: if self.ts_mode == TsMode::Pass {
                    let qs = qs_opt.as_ref().unwrap();
                    Some(wgpu::ComputePassTimestampWrites {
                        query_set: qs,
                        beginning_of_pass_write_index: Some(0),
                        end_of_pass_write_index: Some(1),
                    })
                } else {
                    None
                },
            });
            cpass.set_pipeline(&self.pipeline);
            cpass.set_bind_group(0, &bind, &[]);
            let wg = ((rays.len() as u32) + 63) / 64;
            cpass.dispatch_workgroups(wg, 1, 1);
            drop(cpass);
            if self.ts_mode == TsMode::Pass {
                let qs = qs_opt.as_ref().unwrap();
                let ts_buf = ts_buf_opt.as_ref().unwrap();
                encoder.resolve_query_set(qs, 0..2, ts_buf, 0);
                let ts_readback = ts_rb_opt.as_ref().unwrap();
                encoder.copy_buffer_to_buffer(ts_buf, 0, ts_readback, 0, 16);
            }
        }

        // Copy results
        encoder.copy_buffer_to_buffer(&hit_buf, 0, &readback, 0, hit_sz);

        // Submit + wall-clock fallback timer
        let wall_start = std::time::Instant::now();
        self.queue.submit(Some(encoder.finish()));
        self.device.poll(wgpu::Maintain::Wait);

        // Read timestamps (if any)
        let mut kernel_ms = 0.0;
        if self.ts_mode != TsMode::None {
            let ts_readback = ts_rb_opt.as_ref().unwrap();
            let ts_slice = ts_readback.slice(..);
            let (tx, rx) = mpsc::channel();
            ts_slice.map_async(wgpu::MapMode::Read, move |res| { let _ = tx.send(res); });
            self.device.poll(wgpu::Maintain::Wait);
            if let Ok(Ok(())) = rx.recv() {
                let data = ts_slice.get_mapped_range();
                let ticks: &[u64] = bytemuck::cast_slice(&data);
                if ticks.len() >= 2 {
                    let period_ns = self.queue.get_timestamp_period() as f64;
                    kernel_ms = (ticks[1] - ticks[0]) as f64 * period_ns / 1.0e6;
                }
                drop(data);
                ts_readback.unmap();
            }

        }
        // Wall-clock fallback
        if kernel_ms == 0.0 {
            kernel_ms = wall_start.elapsed().as_secs_f64() * 1e3;
        }

        // Read hits
        let rb_slice = readback.slice(..);
        let (tx, rx) = mpsc::channel();
        rb_slice.map_async(wgpu::MapMode::Read, move |res| { let _ = tx.send(res); });
        self.device.poll(wgpu::Maintain::Wait);
        let hits = match rx.recv() {
            Ok(Ok(())) => {
                let data = rb_slice.get_mapped_range();
                let v: Vec<HitGpu> = bytemuck::cast_slice(&data).to_vec();
                drop(data);
                readback.unmap();
                v
            }
            _ => Vec::new(),
        };

        (kernel_ms, hits)
    }
}
