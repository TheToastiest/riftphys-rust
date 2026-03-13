// File: crates/riftphys-collision/src/broadphase.rs
use riftphys_geom::Aabb;
use rustc_hash::FxHashMap;

/// A persistent, deterministic 3D Spatial Hash Broadphase.
/// Reuses allocations frame-over-frame to eliminate heap thrashing.
pub struct GridBroadphase {
    pub cell_size: f32,
    grid: FxHashMap<[i32; 3], Vec<usize>>,
    oversized: Vec<usize>,
    pairs: Vec<(usize, usize)>,
}

impl GridBroadphase {
    pub fn get_pairs(&self) -> usize{
        self.pairs.len()
    }
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            // Pre-allocate to prevent early resizing spikes
            grid: FxHashMap::default(),
            oversized: Vec::with_capacity(16),
            pairs: Vec::with_capacity(8192),
        }
    }

    /// Runs the broadphase and returns a deterministic, sorted slice of colliding pairs.
    pub fn run(&mut self, aabbs: &[Aabb]) -> &[(usize, usize)] {
        // Clear previous frame data (keeps capacity, avoids re-allocation)
        self.grid.clear();
        self.oversized.clear();
        self.pairs.clear();

        // 1. Binning Phase & Macro-Bypass
        for (i, a) in aabbs.iter().enumerate() {
            if !a.min.x.is_finite() { continue; }

            let dx = a.max.x - a.min.x;
            let dy = a.max.y - a.min.y;
            let dz = a.max.z - a.min.z;

            // Macro-bypass: massive objects skip the grid to prevent map explosion
            if dx > self.cell_size * 5.0 || dy > self.cell_size * 5.0 || dz > self.cell_size * 5.0 {
                self.oversized.push(i);
                continue;
            }

            let min_x = (a.min.x / self.cell_size).floor() as i32;
            let max_x = (a.max.x / self.cell_size).floor() as i32;
            let min_y = (a.min.y / self.cell_size).floor() as i32;
            let max_y = (a.max.y / self.cell_size).floor() as i32;
            let min_z = (a.min.z / self.cell_size).floor() as i32;
            let max_z = (a.max.z / self.cell_size).floor() as i32;

            for x in min_x..=max_x {
                for y in min_y..=max_y {
                    for z in min_z..=max_z {
                        self.grid.entry([x, y, z]).or_default().push(i);
                    }
                }
            }
        }

        // 2. Micro-Pair Extraction (Grid Cells)
        for cell in self.grid.values() {
            let len = cell.len();
            if len < 2 { continue; }

            for i in 0..len {
                for j in (i + 1)..len {
                    let a = cell[i];
                    let b = cell[j];
                    let (idx_a, idx_b) = if a < b { (a, b) } else { (b, a) };

                    if aabbs[idx_a].overlaps(&aabbs[idx_b]) {
                        self.pairs.push((idx_a, idx_b));
                    }
                }
            }
        }

        // 3. Macro-Pair Extraction (Oversized vs Everything)
        for &big_idx in &self.oversized {
            let big_aabb = &aabbs[big_idx];
            for (i, a) in aabbs.iter().enumerate() {
                if i != big_idx && big_aabb.overlaps(a) {
                    let (idx_a, idx_b) = if i < big_idx { (i, big_idx) } else { (big_idx, i) };
                    self.pairs.push((idx_a, idx_b));
                }
            }
        }

        // 4. The Determinism Guarantee
        self.pairs.sort_unstable();
        self.pairs.dedup();

        &self.pairs
    }
}