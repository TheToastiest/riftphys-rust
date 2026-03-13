// File: crates/riftphys-world/src/pipeline.rs or inside world.rs
use riftphys_core::BodyId;
use riftphys_collision::narrowphase::ContactManifold;
use riftphys_collision::environment::EnvCollider;

pub enum ContactEvent {
    /// A collision between two dynamic bodies
    Dynamic(BodyId, BodyId, ContactManifold),
    /// A collision between a dynamic body and the static environment
    Environment(BodyId, EnvCollider, ContactManifold),
}