pub mod cpu;
#[cfg(feature = "gpu-wgpu")]
pub mod gpu_wgpu;

use bitvec::prelude::*;
use glam::{UVec3, Vec3};
use riftphys_materials::MaterialId;
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Ray {
    pub o: Vec3,
    pub d: Vec3,
    pub tmax: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RayHit {
    pub hit: bool,
    pub t: f32,
    pub normal: Vec3,
    pub cell: UVec3,
}


pub struct VoxelChunk {
    pub dims: UVec3,
    pub solid: BitVec,      // occupancy
    pub mats:  Vec<u8>,     // MaterialId as u8 (same length as bits)
    pub voxel_size: f32,
}

impl VoxelChunk {
    pub fn new(dims: UVec3) -> Self {
        let n = (dims.x * dims.y * dims.z) as usize;
        Self {
            dims,
            solid: bitvec![0; n],
            mats: vec![MaterialId::Default as u8; n],
            voxel_size: 1.0,
        }
    }
    #[inline]
    fn idx(&self, x: u32, y: u32, z: u32) -> usize {
        (x + self.dims.x * (y + self.dims.y * z)) as usize
    }
    #[inline]
    pub fn set(&mut self, x: u32, y: u32, z: u32, solid: bool) {
        let i = self.idx(x, y, z);
        self.solid.set(i, solid);
    }
    #[inline]
    pub fn get(&self, x: u32, y: u32, z: u32) -> bool {
        self.solid[self.idx(x, y, z)]
    }
    #[inline]
    pub fn set_mat(&mut self, x: u32, y: u32, z: u32, mat: MaterialId) {
        let i = self.idx(x, y, z);
        self.mats[i] = mat as u8;
    }
    #[inline]
    pub fn mat(&self, x: u32, y: u32, z: u32) -> MaterialId {
        // repr(u8) guarantees this cast is sound
        unsafe { std::mem::transmute::<u8, MaterialId>(self.mats[self.idx(x, y, z)]) }
    }
    #[inline]
    pub fn aabb_min(&self) -> Vec3 { Vec3::ZERO }
    #[inline]
    pub fn aabb_max(&self) -> Vec3 {
        Vec3::new(self.dims.x as f32, self.dims.y as f32, self.dims.z as f32) * self.voxel_size
    }
}