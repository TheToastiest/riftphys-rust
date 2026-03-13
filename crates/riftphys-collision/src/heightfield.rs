// File: crates/riftphys-collision/src/heightfield.rs

use crate::environment::{EnvCollider, SpatialEnvironment};
use riftphys_core::types::{Isometry, Quat, Vec3};
use riftphys_geom::{Aabb, Shape, Triangle};
use riftphys_materials::materials::MaterialId;

/// A grid-based heightfield for static terrain.
/// The grid lies on the XZ plane. Y is up.
pub struct Heightfield {
    pub heights: Vec<f32>,
    pub cols: usize,  // Number of vertices along the X axis
    pub rows: usize,  // Number of vertices along the Z axis
    pub scale_x: f32, // Spacing between columns in world units
    pub scale_z: f32, // Spacing between rows in world units
    pub material: MaterialId,
}

impl Heightfield {
    /// Safely constructs a new Heightfield, ensuring the data buffer matches the grid bounds.
    pub fn new(cols: usize, rows: usize, scale_x: f32, scale_z: f32, material: MaterialId, heights: Vec<f32>) -> Self {
        assert_eq!(heights.len(), cols * rows, "Heightfield data length must exactly match cols * rows");
        Self { heights, cols, rows, scale_x, scale_z, material }
    }

    #[inline(always)]
    fn get_height(&self, c: usize, r: usize) -> f32 {
        self.heights[r * self.cols + c]
    }

    #[inline(always)]
    fn get_vertex(&self, c: usize, r: usize) -> Vec3 {
        Vec3::new(
            c as f32 * self.scale_x,
            self.get_height(c, r),
            r as f32 * self.scale_z,
        )
    }

    #[inline(always)]
    fn pack_coord(c: usize, r: usize) -> u64 {
        // Pack column and row into a single u64 identifier for gameplay Raycasts/Contacts
        ((c as u64) << 32) | (r as u64)
    }
}

impl SpatialEnvironment for Heightfield {
    fn query_aabb(&self, aabb: &Aabb, out_buffer: &mut Vec<EnvCollider>) {
        // A heightfield needs at least a 2x2 grid of vertices to form 1 cell (2 triangles)
        if self.cols < 2 || self.rows < 2 { return; }

        // Max valid index for the top-left vertex of a cell
        let max_c_idx = (self.cols - 2) as isize;
        let max_r_idx = (self.rows - 2) as isize;

        // Map AABB min/max to grid indices.
        // We expand the search by 1 cell (-1 / +1) to ensure edges overlapping the AABB boundary are caught.
        let min_c = ((aabb.min.x / self.scale_x).floor() as isize - 1).clamp(0, max_c_idx) as usize;
        let max_c = ((aabb.max.x / self.scale_x).ceil() as isize + 1).clamp(0, max_c_idx) as usize;

        let min_r = ((aabb.min.z / self.scale_z).floor() as isize - 1).clamp(0, max_r_idx) as usize;
        let max_r = ((aabb.max.z / self.scale_z).ceil() as isize + 1).clamp(0, max_r_idx) as usize;

        // Triangles are generated natively in world space, so the transform is Identity.
        let ident = Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY };

        // Yield two triangles per grid cell deterministically (Row major, then Col)
        for r in min_r..=max_r {
            for c in min_c..=max_c {
                let v00 = self.get_vertex(c, r);
                let v10 = self.get_vertex(c + 1, r);
                let v01 = self.get_vertex(c, r + 1);
                let v11 = self.get_vertex(c + 1, r + 1);

                let user_data = Self::pack_coord(c, r);

                // Triangle 1: Top-Left
                out_buffer.push(EnvCollider {
                    shape: Shape::Triangle(Triangle { a: v00, b: v01, c: v10 }),
                    transform: ident,
                    material: self.material,
                    user_data,
                });

                // Triangle 2: Bottom-Right
                out_buffer.push(EnvCollider {
                    shape: Shape::Triangle(Triangle { a: v10, b: v01, c: v11 }),
                    transform: ident,
                    material: self.material,
                    user_data,
                });
            }
        }
    }
}