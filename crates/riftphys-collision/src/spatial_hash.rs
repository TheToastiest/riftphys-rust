// crates/riftphys-collision/src/spatial_hash.rs
use glam::IVec3;
use std::collections::HashMap;
use riftphys_core::Vec3;
use riftphys_geom::Aabb; // Corrected import path

pub struct SpatialHash {
    cell_size: f32,
    grid: HashMap<IVec3, Vec<u32>>,
}

impl SpatialHash {
    pub fn new(cell_size: f32) -> Self {
        Self { cell_size, grid: HashMap::new() }
    }

    pub fn insert(&mut self, id: u32, aabb: &Aabb) {
        // We floor the coordinates to get the integer cell index
        let min = (aabb.min / self.cell_size).floor();
        let max = (aabb.max / self.cell_size).floor();

        for x in (min.x as i32)..=(max.x as i32) {
            for y in (min.y as i32)..=(max.y as i32) {
                for z in (min.z as i32)..=(max.z as i32) {
                    self.grid.entry(IVec3::new(x, y, z)).or_default().push(id);
                }
            }
        }
    }

    pub fn query_pairs(&self) -> Vec<(u32, u32)> {
        let mut pairs = Vec::new();
        for cell in self.grid.values() {
            if cell.len() < 2 { continue; }
            for i in 0..cell.len() {
                for j in (i + 1)..cell.len() {
                    let a = cell[i];
                    let b = cell[j];
                    // Ensure deterministic pair ordering (smaller ID first)
                    if a < b { pairs.push((a, b)); } else { pairs.push((b, a)); }
                }
            }
        }
        // Deduplicate pairs that might appear in multiple grid cells
        pairs.sort_unstable();
        pairs.dedup();
        pairs
    }
}