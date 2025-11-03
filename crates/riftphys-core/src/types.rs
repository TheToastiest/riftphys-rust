use glam::{Vec3A, Mat3A, Quat};
use crate::Scalar;

pub type Vec3 = Vec3A;
pub type Mat3 = Mat3A;

#[inline] pub fn vec3(x: Scalar, y: Scalar, z: Scalar) -> Vec3 { Vec3::new(x, y, z) }
#[inline] pub fn iso(pos: Vec3, rot: Quat) -> Isometry { Isometry { pos, rot } }
#[inline] pub fn quat_identity() -> Quat { Quat::IDENTITY }

#[derive(Copy, Clone, Debug)]
pub struct Isometry { pub pos: Vec3, pub rot: Quat }

#[derive(Copy, Clone, Debug, Default)]
pub struct Velocity { pub lin: Vec3, pub ang: Vec3 }

impl Default for Isometry {
    fn default() -> Self { Self { pos: Vec3::ZERO, rot: Quat::IDENTITY } }
}
