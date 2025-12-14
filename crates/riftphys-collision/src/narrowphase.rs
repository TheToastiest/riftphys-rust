// riftphys-collision/src/narrowphase.rs
use riftphys_core::types::{Isometry, Vec3};
use riftphys_geom::{Shape, Triangle};

#[derive(Copy, Clone, Debug)]
pub struct ContactPoint {
    pub position:    Vec3,
    pub normal:      Vec3, // points from A → B in collide_shapes; from tri → sphere in sphere_triangle
    pub penetration: f32,  // includes contact slop (contact_dist)
}

#[derive(Clone, Debug, Default)]
pub struct ContactManifold {
    pub normal: Vec3,           // shared normal for manifold
    pub points: Vec<ContactPoint>,
}

#[inline]
fn q6(x: f32) -> f32 {
    if x.is_finite() {
        (x * 1.0e6_f32).round() * 1.0e-6_f32
    } else {
        0.0
    }
}

#[inline]
fn q6_vec3(v: Vec3) -> Vec3 {
    Vec3::new(q6(v.x), q6(v.y), q6(v.z))
}

#[inline]
fn q6_norm_vec3(v: Vec3, fallback: Vec3) -> Vec3 {
    let qv = q6_vec3(v);
    let len2 = qv.length_squared();
    if len2 > 1.0e-20 {
        // renormalize after quantization (critical for stable impulses)
        let inv = 1.0 / len2.sqrt();
        q6_vec3(qv * inv)
    } else {
        fallback
    }
}

#[inline]
fn finite_vec3(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// Canonical public entry for dynamic collider pairs.
/// - `normal` points from A → B.
/// - `contact_dist` is a slop / speculative distance (meters).
pub fn collide_shapes(
    shape_a: &Shape,
    xf_a: &Isometry,
    shape_b: &Shape,
    xf_b: &Isometry,
    contact_dist: f32,
) -> Option<ContactManifold> {
    use Shape::*;

    if !finite_vec3(xf_a.pos) || !finite_vec3(xf_b.pos) {
        return None;
    }

    let contact_dist = q6(contact_dist.max(0.0));

    match (shape_a, shape_b) {
        (Sphere { r: ra }, Sphere { r: rb }) => {
            collide_sphere_sphere(xf_a.pos, *ra, xf_b.pos, *rb, contact_dist)
        }

        (Capsule { r: ra, hh: ha }, Sphere { r: rb }) => {
            collide_capsule_sphere(xf_a, *ra, *ha, xf_b.pos, *rb, contact_dist)
        }

        (Sphere { r: ra }, Capsule { r: rb, hh: hb }) => {
            collide_capsule_sphere(xf_b, *rb, *hb, xf_a.pos, *ra, contact_dist).map(|mut m| {
                m.normal = -m.normal;
                for p in &mut m.points { p.normal = m.normal; }
                m
            })
        }

        (Capsule { r: ra, hh: ha }, Capsule { r: rb, hh: hb }) => {
            collide_capsule_capsule(xf_a, *ra, *ha, xf_b, *rb, *hb, contact_dist)
        }

        _ => None,
    }
}

fn collide_sphere_sphere(
    center_a: Vec3,
    ra: f32,
    center_b: Vec3,
    rb: f32,
    contact_dist: f32,
) -> Option<ContactManifold> {
    if !finite_vec3(center_a) || !finite_vec3(center_b) { return None; }

    let ra = q6(ra.max(0.0));
    let rb = q6(rb.max(0.0));
    let contact_dist = q6(contact_dist.max(0.0));

    let delta = center_b - center_a;
    let dist2 = delta.length_squared();
    if !dist2.is_finite() { return None; }

    let r_sum = ra + rb;
    let allowed = q6(r_sum + contact_dist);

    if dist2 > allowed * allowed {
        return None;
    }

    let dist = dist2.sqrt();

    // raw normal (deterministic fallback if coincident)
    let raw_n = if dist > 1.0e-6 { delta / dist } else { Vec3::new(1.0, 0.0, 0.0) };
    let normal = q6_norm_vec3(raw_n, Vec3::new(1.0, 0.0, 0.0));

    // Penetration includes slop: = allowed - separation.
    let penetration = q6((allowed - dist).max(0.0));

    // Midpoint of the two surface points along the normal
    let pa = center_a + normal * ra;
    let pb = center_b - normal * rb;
    let position = q6_vec3((pa + pb) * 0.5);

    let cp = ContactPoint { position, normal, penetration };

    Some(ContactManifold { normal, points: vec![cp] })
}

#[inline]
fn capsule_segment_world(xf: &Isometry, half: f32) -> (Vec3, Vec3) {
    let axis = xf.rot * Vec3::Y;
    let h = half.abs();
    let p0 = xf.pos - axis * h;
    let p1 = xf.pos + axis * h;
    (p0, p1)
}

#[inline]
fn closest_point_segment(p: Vec3, a: Vec3, b: Vec3) -> (Vec3, f32) {
    let ab = b - a;
    let ab_len2 = ab.length_squared();
    if ab_len2 <= 1.0e-12 {
        return (a, 0.0);
    }
    let t = ((p - a).dot(ab) / ab_len2).clamp(0.0, 1.0);
    (a + ab * t, t)
}

fn closest_points_segment_segment(p0: Vec3, p1: Vec3, q0: Vec3, q1: Vec3) -> (Vec3, Vec3) {
    // Ericson "ClosestPtSegmentSegment" with clamping
    let u = p1 - p0;
    let v = q1 - q0;
    let w = p0 - q0;
    let a = u.dot(u);
    let b = u.dot(v);
    let c = v.dot(v);
    let d = u.dot(w);
    let e = v.dot(w);
    let d0 = a * c - b * b;

    let mut s_n;
    let mut t_n;
    let mut s_d = d0;
    let mut t_d = d0;

    const EPS: f32 = 1.0e-6;

    if d0 < EPS {
        s_n = 0.0; s_d = 1.0;
        t_n = e;   t_d = c;
    } else {
        s_n = b * e - c * d;
        t_n = a * e - b * d;
        if s_n < 0.0 {
            s_n = 0.0;
            t_n = e;
            t_d = c;
        } else if s_n > s_d {
            s_n = s_d;
            t_n = e + b;
            t_d = c;
        }
    }

    if t_n < 0.0 {
        t_n = 0.0;
        if -d < 0.0 { s_n = 0.0; }
        else if -d > a { s_n = s_d; }
        else { s_n = -d; s_d = a; }
    } else if t_n > t_d {
        t_n = t_d;
        if (-d + b) < 0.0 { s_n = 0.0; }
        else if (-d + b) > a { s_n = s_d; }
        else { s_n = -d + b; s_d = a; }
    }

    let sc = if s_n.abs() < EPS { 0.0 } else { s_n / s_d };
    let tc = if t_n.abs() < EPS { 0.0 } else { t_n / t_d };

    let p_closest = p0 + u * sc;
    let q_closest = q0 + v * tc;
    (p_closest, q_closest)
}

fn collide_capsule_sphere(
    cap_xf: &Isometry,
    cap_r: f32,
    cap_half: f32,
    sphere_center: Vec3,
    sphere_r: f32,
    contact_dist: f32,
) -> Option<ContactManifold> {
    let (a, b) = capsule_segment_world(cap_xf, cap_half);
    let (q, _t) = closest_point_segment(sphere_center, a, b);
    collide_sphere_sphere(q, cap_r, sphere_center, sphere_r, contact_dist)
}

fn collide_capsule_capsule(
    xf_a: &Isometry,
    ra: f32,
    ha: f32,
    xf_b: &Isometry,
    rb: f32,
    hb: f32,
    contact_dist: f32,
) -> Option<ContactManifold> {
    let (a0, a1) = capsule_segment_world(xf_a, ha);
    let (b0, b1) = capsule_segment_world(xf_b, hb);

    let (pa, pb) = closest_points_segment_segment(a0, a1, b0, b1);
    collide_sphere_sphere(pa, ra, pb, rb, contact_dist)
}

/// Sphere vs Triangle contact.
/// - `center` and `radius` are sphere in world space.
/// - `tri` is world-space triangle.
/// - `normal` points from triangle towards the sphere (tri plane normal, oriented to face the sphere).
pub fn collide_sphere_triangle(
    center: Vec3,
    radius: f32,
    tri: &Triangle,
    contact_dist: f32,
) -> Option<ContactPoint> {
    if !finite_vec3(center) { return None; }
    if tri.is_degenerate() { return None; }

    let radius = q6(radius.max(0.0));
    let contact_dist = q6(contact_dist.max(0.0));

    let closest = tri.closest_point(center);
    if !finite_vec3(closest) { return None; }

    let delta = center - closest;
    let dist2 = delta.length_squared();
    if !dist2.is_finite() { return None; }

    let allowed = q6(radius + contact_dist);
    if dist2 > allowed * allowed { return None; }

    let dist = dist2.sqrt();

    // Start with triangle normal; if degenerate, fallback.
    let mut n = tri.normal();
    if n.length_squared() < 1.0e-8 {
        n = delta.normalize_or_zero();
        if n.length_squared() < 1.0e-8 {
            n = Vec3::new(0.0, 1.0, 0.0);
        }
    }

    // Ensure normal points from triangle towards sphere.
    if delta.dot(n) < 0.0 { n = -n; }

    let normal = q6_norm_vec3(n, Vec3::new(0.0, 1.0, 0.0));
    let penetration = q6((allowed - dist).max(0.0));
    let position = q6_vec3(closest);

    Some(ContactPoint { position, normal, penetration })
}
