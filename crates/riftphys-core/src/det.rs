// riftphys-core/src/det.rs
use crate::types::{Quat, Vec3};
use crate::Scalar;

pub const Q6: Scalar = 1.0e-6;

#[inline]
pub fn q6(x: Scalar) -> Scalar {
    (x * 1.0e6_f32).round() * 1.0e-6_f32
}

#[inline]
pub fn q6_vec3(v: Vec3) -> Vec3 {
    Vec3::new(q6(v.x), q6(v.y), q6(v.z))
}

#[inline]
pub fn q6_unit_vec3(v: Vec3) -> Vec3 {
    // quantize -> normalize -> quantize, so the unit vector is stable across platforms
    let n = q6_vec3(v).normalize_or_zero();
    q6_vec3(n)
}

#[inline]
pub fn safe_contact_dist(cd: Scalar) -> Scalar {
    if cd.is_finite() { cd.max(0.0) } else { 0.0 }
}

#[inline]
pub fn quat_canonical(mut q: Quat) -> Quat {
    // q and -q represent the same rotation; canonicalize sign so hashing/compare is stable
    q = q.normalize();
    if q.w < 0.0 {
        q = Quat::from_xyzw(-q.x, -q.y, -q.z, -q.w);
    }
    q
}

#[inline]
pub fn q6_quat(q: Quat) -> Quat {
    let q = quat_canonical(q);
    let q = Quat::from_xyzw(q6(q.x), q6(q.y), q6(q.z), q6(q.w));
    let q = quat_canonical(q);
    Quat::from_xyzw(q6(q.x), q6(q.y), q6(q.z), q6(q.w))
}

#[inline]
pub fn vec3_is_finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

#[inline]
pub fn quat_is_finite(q: Quat) -> bool {
    q.x.is_finite() && q.y.is_finite() && q.z.is_finite() && q.w.is_finite()
}
