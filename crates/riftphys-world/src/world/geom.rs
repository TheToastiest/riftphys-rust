use riftphys_core::Vec3;
use riftphys_geom::Aabb;

#[inline]
pub(super) fn clampf(x: f32, lo: f32, hi: f32) -> f32 { x.max(lo).min(hi) }

#[inline]
pub(super) fn clamp_vec3(p: Vec3, mn: Vec3, mx: Vec3) -> Vec3 {
    Vec3::new(clampf(p.x, mn.x, mx.x), clampf(p.y, mn.y, mx.y), clampf(p.z, mn.z, mx.z))
}

#[inline]
pub(super) fn closest_point_on_segment(a: Vec3, b: Vec3, p: Vec3) -> (Vec3, f32) {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    (a + ab * t, t)
}

pub(super) fn closest_points_segment_aabb(a: Vec3, b: Vec3, mn: Vec3, mx: Vec3) -> (Vec3, Vec3) {
    let mut ps = (a + b) * 0.5;
    let mut qs = clamp_vec3(ps, mn, mx);
    for _ in 0..3 {
        let (p2, _t) = closest_point_on_segment(a, b, qs);
        ps = p2;
        qs = clamp_vec3(ps, mn, mx);
    }
    (ps, qs)
}

pub(super) fn orthonormal_basis(n: Vec3) -> (Vec3, Vec3) {
    let ax = n.x.abs(); let ay = n.y.abs(); let az = n.z.abs();
    let base = if ax <= ay && ax <= az { Vec3::new(1.0, 0.0, 0.0) }
    else if ay <= az        { Vec3::new(0.0, 1.0, 0.0) }
    else                    { Vec3::new(0.0, 0.0, 1.0) };
    let t1 = (base.cross(n)).normalize_or_zero();
    let t2 = n.cross(t1);
    (t1, t2)
}

pub(super) fn normal_from_aabb_point(aabb: &Aabb, p: Vec3) -> Vec3 {
    let eps = 1.0e-4;

    if (p.x - aabb.min.x).abs() < eps { return Vec3::new(-1.0, 0.0, 0.0); }
    if (p.x - aabb.max.x).abs() < eps { return Vec3::new( 1.0, 0.0, 0.0); }

    if (p.y - aabb.min.y).abs() < eps { return Vec3::new(0.0, -1.0, 0.0); }
    if (p.y - aabb.max.y).abs() < eps { return Vec3::new(0.0,  1.0, 0.0); }

    if (p.z - aabb.min.z).abs() < eps { return Vec3::new(0.0, 0.0, -1.0); }
    if (p.z - aabb.max.z).abs() < eps { return Vec3::new(0.0, 0.0,  1.0); }

    let center = (aabb.min + aabb.max) * 0.5;
    let mut n = p - center;
    let len2 = n.length_squared();
    if len2 > 1.0e-20 { n /= len2.sqrt(); } else { n = Vec3::new(0.0, 1.0, 0.0); }
    n
}

pub(super) fn ray_aabb(origin: Vec3, dir: Vec3, aabb: &Aabb, max_dist: f32) -> Option<(f32, Vec3, Vec3)> {
    let eps = 1.0e-8;
    let mut tmin = 0.0_f32;
    let mut tmax = max_dist;

    let o = [origin.x, origin.y, origin.z];
    let d = [dir.x,    dir.y,    dir.z];
    let mn = [aabb.min.x, aabb.min.y, aabb.min.z];
    let mx = [aabb.max.x, aabb.max.y, aabb.max.z];

    for axis in 0..3 {
        let da = d[axis];
        let oa = o[axis];
        let amin = mn[axis];
        let amax = mx[axis];

        if da.abs() < eps {
            if oa < amin || oa > amax { return None; }
        } else {
            let inv = 1.0 / da;
            let mut t1 = (amin - oa) * inv;
            let mut t2 = (amax - oa) * inv;
            if t1 > t2 { std::mem::swap(&mut t1, &mut t2); }
            if t1 > tmin { tmin = t1; }
            if t2 < tmax { tmax = t2; }
            if tmin > tmax { return None; }
        }
    }

    if tmin < 0.0 || tmin > max_dist { return None; }

    let point = origin + dir * tmin;
    let normal = normal_from_aabb_point(aabb, point);
    Some((tmin, point, normal))
}
