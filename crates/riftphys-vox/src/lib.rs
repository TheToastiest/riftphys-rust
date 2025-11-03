use bitvec::prelude::*;
use glam::UVec3;

/// Very small voxel chunk: solid (1) / empty (0).
pub struct VoxelChunk {
    pub dims: UVec3,          // e.g., 32x32x32
    pub solid: BitVec,        // dims.x * dims.y * dims.z bits
}

impl VoxelChunk {
    pub fn new(dims: UVec3) -> Self {
        let count = (dims.x * dims.y * dims.z) as usize;
        Self { dims, solid: bitvec![0; count] }
    }
    #[inline] fn idx(&self, x: u32, y: u32, z: u32) -> usize {
        (x + self.dims.x * (y + self.dims.y * z)) as usize
    }
    pub fn set(&mut self, x: u32, y: u32, z: u32, value: bool) {
        let i = self.idx(x,y,z);
        self.solid.set(i, value);
    }
    pub fn get(&self, x: u32, y: u32, z: u32) -> bool {
        self.solid[self.idx(x,y,z)]
    }
}
