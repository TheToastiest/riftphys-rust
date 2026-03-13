// riftphys-geom/src/shape.rs
use riftphys_core::types::{Isometry, Mat3, Vec3};
use crate::aabb::Aabb;
use crate::plane::Plane;
use crate::triangle::Triangle;

#[derive(Copy, Clone, Debug)]
pub enum Shape {
    Sphere { r: f32 },
    Box { hx: f32, hy: f32, hz: f32 },
    Capsule { r: f32, hh: f32 },  // half-height along local Y
    Cylinder { r: f32, hh: f32 }, // half-height along local Y
    Triangle(Triangle),
    Plane(Plane),
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

            let rot = Mat3::from_quat(xf.rot);
            let abs_r = Mat3::from_cols(rot.x_axis.abs(), rot.y_axis.abs(), rot.z_axis.abs());
            let world_he = abs_r * he;

            Aabb::from_center_half_extents(xf.pos, world_he)
        }

        Shape::Capsule { r, hh } => {
            let rr = r.abs();
            let h = hh.abs();

            let axis_world = xf.rot * Vec3::Y * h;
            let he = axis_world.abs() + Vec3::splat(rr);

            Aabb::from_center_half_extents(xf.pos, he)
        }

        Shape::Cylinder { r, hh } => {
            let rr = r.abs();
            let h = hh.abs();

            let rot = Mat3::from_quat(xf.rot);
            let y_axis = rot.y_axis;

            // Exact OBB extent projection for a cylinder
            let ex = h * y_axis.x.abs() + rr * (1.0 - y_axis.x * y_axis.x).max(0.0).sqrt();
            let ey = h * y_axis.y.abs() + rr * (1.0 - y_axis.y * y_axis.y).max(0.0).sqrt();
            let ez = h * y_axis.z.abs() + rr * (1.0 - y_axis.z * y_axis.z).max(0.0).sqrt();

            Aabb::from_center_half_extents(xf.pos, Vec3::new(ex, ey, ez))
        }

        Shape::Triangle(tri) => {
            let wa = xf.pos + xf.rot * tri.a;
            let wb = xf.pos + xf.rot * tri.b;
            let wc = xf.pos + xf.rot * tri.c;

            let min = wa.min(wb).min(wc);
            let max = wa.max(wb).max(wc);

            // Using your specific Aabb::new constructor
            Aabb::new(min, max)
        }

        Shape::Plane(_) => {
            // Infinite extents ensure it passes the broadphase
            Aabb::from_center_half_extents(Vec3::ZERO, Vec3::splat(f32::INFINITY))
        }
    }
}