use wgpu::util::DeviceExt;
use riftphys_materials::{self as mats};

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpuMatVis { pub base: [f32;3], pub rough: f32, pub metal: f32, pub _pad: [f32;2] }
unsafe impl bytemuck::Zeroable for GpuMatVis {}
unsafe impl bytemuck::Pod for GpuMatVis {}

pub struct MatPalette {
    pub buf: wgpu::Buffer,
    pub layout: wgpu::BindGroupLayout,
    pub bind: wgpu::BindGroup,
}

pub fn build_palette(device:&wgpu::Device) -> MatPalette {
    // Fill table
    let mut table = [GpuMatVis { base:[0.5,0.5,0.5], rough:0.9, metal:0.0, _pad:[0.0;2] }; 256];
    for i in 0..256 {
        // safe because MaterialId is #[repr(u8)]
        let id = unsafe { std::mem::transmute::<u8, mats::MaterialId>(i as u8) };
        let v = mats::vis::mat_vis(id);
        table[i] = GpuMatVis { base: v.base_rgb, rough: v.rough, metal: v.metal, _pad:[0.0;2] };
    }

    let buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("mat-palette"),
        contents: bytemuck::cast_slice(&table),
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
    });

    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("mat-palette-bgl"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: true },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    });

    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor{
        label: Some("mat-palette-bg"),
        layout: &layout,
        entries: &[wgpu::BindGroupEntry { binding: 0, resource: buf.as_entire_binding() }],
    });

    MatPalette { buf, layout, bind }
}
