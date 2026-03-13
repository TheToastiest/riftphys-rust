use glam::{Mat3A, Quat as GQuat, Vec3A, Vec3 as GVec3};
use crate::Scalar;

// Core math aliases (glam-backed).
pub type Vec3a = Vec3A;
pub type Vec3 = GVec3;
pub type Mat3 = Mat3A;

// Canonical quaternion type for the whole codebase.
// Keep QuatP as an old alias so older crates don’t break.
pub type Quat = GQuat;
pub type QuatP = Quat;

#[inline] pub fn vec3(x: Scalar, y: Scalar, z: Scalar) -> Vec3 { Vec3::new(x, y, z) }
#[inline] pub fn quat_identity() -> Quat { Quat::IDENTITY }

#[inline] pub fn iso(pos: Vec3, rot: Quat) -> Isometry { Isometry { pos, rot } }

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct Isometry {
    pub pos: Vec3,
    pub rot: Quat,
}

impl Isometry {
    pub const IDENTITY: Self = Self { pos: Vec3::ZERO, rot: Quat::IDENTITY };
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct Velocity {
    pub lin: Vec3,
    pub ang: Vec3,
}
