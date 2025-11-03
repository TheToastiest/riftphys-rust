use riftphys_core::types::{Isometry, Vec3, Mat3};
use glam::Mat3A;
use crate::aabb::Aabb;

#[derive(Copy, Clone, Debug)]
pub enum Shape {
    Sphere { r: f32 },
    Box { hx: f32, hy: f32, hz: f32 },
    Capsule { r: f32, hh: f32 }, // half-height along local Y
}

#[inline]
pub fn aabb_of(shape: &Shape, xf: &Isometry) -> Aabb {
    match *shape {
        Shape::Sphere { r } => Aabb::from_center_half_extents(xf.pos, Vec3::splat(r)),
        Shape::Box { hx, hy, hz } => {
            let he = Vec3::new(hx, hy, hz);
            let rot = Mat3A::from_quat(xf.rot);
            let m = Mat3::from_cols(rot.x_axis.abs(), rot.y_axis.abs(), rot.z_axis.abs());
            let world_he = m * he;
            Aabb::from_center_half_extents(xf.pos, world_he)
        }
        Shape::Capsule { r, hh } => {
            let axis_world = xf.rot * Vec3::Y * hh.abs();
            let he = axis_world.abs() + Vec3::splat(r);
            Aabb::from_center_half_extents(xf.pos, he)
        }
    }
}
