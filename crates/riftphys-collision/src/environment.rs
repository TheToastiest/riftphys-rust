// File: crates/riftphys-collision/src/environment.rs

use riftphys_core::types::Isometry;
use riftphys_geom::{Aabb, Shape};
use riftphys_materials::materials::MaterialId;

/// A geometric primitive yielded by the environment for narrowphase testing.
#[derive(Clone, Debug)]
pub struct EnvCollider {
    pub shape: Shape,
    pub transform: Isometry,
    pub material: MaterialId,
    /// A generic payload used by game logic to identify what was hit.
    /// For a voxel grid, this might be a packed [i32; 3] coordinate.
    /// For a heightfield, it might be a packed (x, z) cell index.
    pub user_data: u64,
}

/// The interface for any static or destructible environment.
pub trait SpatialEnvironment: Send + Sync {
    /// Queries the environment for all colliders overlapping the given AABB.
    /// Results MUST be pushed into `out_buffer` in a strictly deterministic order.
    fn query_aabb(&self, aabb: &Aabb, out_buffer: &mut Vec<EnvCollider>);
}