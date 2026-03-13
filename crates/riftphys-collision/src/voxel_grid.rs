// File: crates/riftphys-collision/src/voxel_grid.rs

use crate::environment::{EnvCollider, SpatialEnvironment};
use riftphys_core::types::{Isometry, Quat, Vec3};
use riftphys_geom::{Aabb, Shape};
use riftphys_materials::materials::MaterialId;
use std::collections::BTreeMap;

pub struct VoxelGrid {
    pub cell_size: f32,
    // BTreeMap guarantees deterministic order across platforms.
    // Key: [X, Y, Z]. Value: MaterialId.
    pub cells: BTreeMap<[i32; 3], MaterialId>,
}

impl VoxelGrid {
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            cells: BTreeMap::new(),
        }
    }

    pub fn set_voxel(&mut self, x: i32, y: i32, z: i32, mat: MaterialId) {
        self.cells.insert([x, y, z], mat);
    }

    pub fn remove_voxel(&mut self, x: i32, y: i32, z: i32) {
        self.cells.remove(&[x, y, z]);
    }

    #[inline(always)]
    fn pack_coord(x: i32, y: i32, z: i32) -> u64 {
        // Bitwise packing: 21 bits per axis ensures valid coordinates up to +/- 1 million.
        let ux = (x as u32 & 0x1FFFFF) as u64;
        let uy = (y as u32 & 0x1FFFFF) as u64;
        let uz = (z as u32 & 0x1FFFFF) as u64;
        (ux << 42) | (uy << 21) | uz
    }
}

impl SpatialEnvironment for VoxelGrid {
    fn query_aabb(&self, aabb: &Aabb, out_buffer: &mut Vec<EnvCollider>) {
        // Floor division safely handles negative coordinate space.
        let min_x = (aabb.min.x / self.cell_size).floor() as i32;
        let min_y = (aabb.min.y / self.cell_size).floor() as i32;
        let min_z = (aabb.min.z / self.cell_size).floor() as i32;

        let max_x = (aabb.max.x / self.cell_size).floor() as i32;
        let max_y = (aabb.max.y / self.cell_size).floor() as i32;
        let max_z = (aabb.max.z / self.cell_size).floor() as i32;

        let half = self.cell_size * 0.5;
        let shape = Shape::Box { hx: half, hy: half, hz: half };

        // Strictly nested loops guarantee deterministic yield order.
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                for z in min_z..=max_z {
                    if let Some(&material) = self.cells.get(&[x, y, z]) {
                        // The position is exactly the center of the voxel.
                        let pos = Vec3::new(
                            x as f32 * self.cell_size + half,
                            y as f32 * self.cell_size + half,
                            z as f32 * self.cell_size + half,
                        );

                        out_buffer.push(EnvCollider {
                            shape: shape.clone(),
                            transform: Isometry { pos, rot: Quat::IDENTITY },
                            material,
                            user_data: Self::pack_coord(x, y, z),
                        });
                    }
                }
            }
        }
    }
}