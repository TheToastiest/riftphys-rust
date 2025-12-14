use riftphys_core::types::{Isometry, Mat3, Vec3};
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
        Shape::Sphere { r } => {
            let rr = r.abs();
            Aabb::from_center_half_extents(xf.pos, Vec3::splat(rr))
        }

        Shape::Box { hx, hy, hz } => {
            let he = Vec3::new(hx.abs(), hy.abs(), hz.abs());

            // For OBB->AABB: world_half_extents = abs(R) * local_half_extents
            // Mat3 here is your core alias (currently glam::Mat3A), so keep it consistent.
            let rot = Mat3::from_quat(xf.rot);
            let abs_r = Mat3::from_cols(rot.x_axis.abs(), rot.y_axis.abs(), rot.z_axis.abs());
            let world_he = abs_r * he;

            Aabb::from_center_half_extents(xf.pos, world_he)
        }

        Shape::Capsule { r, hh } => {
            let rr = r.abs();
            let h = hh.abs();

            // Capsule axis is local +Y.
            let axis_world = xf.rot * Vec3::Y * h;
            let he = axis_world.abs() + Vec3::splat(rr);

            Aabb::from_center_half_extents(xf.pos, he)
        }
    }
}
