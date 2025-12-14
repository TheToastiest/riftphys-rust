// riftphys-geom/src/triangle.rs
use riftphys_core::types::Vec3;
use crate::aabb::Aabb;
use crate::plane::Plane;
use crate::ray::Ray;

const EPS_AREA2: f32 = 1.0e-16;   // (1e-8)^2 for area-style degeneracy
const EPS_DENOM: f32 = 1.0e-12;   // denom guard (barycentric / inside-face)
const EPS_BARY:  f32 = -1.0e-5;   // inside tolerance; negative means allow slight outside

/// Triangle primitive in 3D space (for heightfields, mesh collisions, etc.).
#[derive(Copy, Clone, Debug)]
pub struct Triangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
}

impl Triangle {
    #[inline]
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Self { Self { a, b, c } }

    /// Unnormalized geometric normal (area-weighted).
    #[inline]
    pub fn normal_unnormalized(&self) -> Vec3 {
        (self.b - self.a).cross(self.c - self.a)
    }

    /// Unit normal. Returns ZERO if degenerate or non-finite.
    #[inline]
    pub fn normal(&self) -> Vec3 {
        let n = self.normal_unnormalized();
        if !n.is_finite() { return Vec3::ZERO; }
        n.normalize_or_zero()
    }

    /// Plane as a struct; None if the triangle is degenerate or non-finite.
    #[inline]
    pub fn plane_struct(&self) -> Option<Plane> {
        Plane::from_points(self.a, self.b, self.c)
    }

    /// Plane equation n·x + d = 0, with n normalized if possible.
    /// Falls back to (0,0,0),0 if degenerate.
    #[inline]
    pub fn plane(&self) -> (Vec3, f32) {
        self.plane_struct().map(|pl| (pl.n, pl.d)).unwrap_or((Vec3::ZERO, 0.0))
    }

    /// Axis-aligned bounding box of the triangle (Aabb::new handles quantize + ordering).
    #[inline]
    pub fn aabb(&self) -> Aabb {
        let min = self.a.min(self.b).min(self.c);
        let max = self.a.max(self.b).max(self.c);
        Aabb::new(min, max)
    }

    /// Double area squared = |n_unnormalized|^2 (no sqrt).
    #[inline]
    pub fn double_area2(&self) -> f32 {
        let n = self.normal_unnormalized();
        n.dot(n)
    }

    /// Double area (|n_unnormalized|).
    #[inline]
    pub fn double_area(&self) -> f32 {
        self.double_area2().sqrt()
    }

    /// Area of the triangle.
    #[inline]
    pub fn area(&self) -> f32 {
        0.5 * self.double_area()
    }

    /// Whether the triangle is effectively degenerate.
    #[inline]
    pub fn is_degenerate(&self) -> bool {
        let a2 = self.double_area2();
        !a2.is_finite() || a2 <= EPS_AREA2
    }

    /// Barycentric coordinates (u,v,w) of `p` relative to this triangle.
    ///
    /// p = u*a + v*b + w*c, u+v+w = 1.
    /// If degenerate or non-finite, returns (-1,-1,-1).
    #[inline]
    pub fn barycentric(&self, p: Vec3) -> Vec3 {
        if !(self.a.is_finite() && self.b.is_finite() && self.c.is_finite() && p.is_finite()) {
            return Vec3::new(-1.0, -1.0, -1.0);
        }

        let v0 = self.b - self.a;
        let v1 = self.c - self.a;
        let v2 = p - self.a;

        let d00 = v0.dot(v0);
        let d01 = v0.dot(v1);
        let d11 = v1.dot(v1);
        let d20 = v2.dot(v0);
        let d21 = v2.dot(v1);

        // denom = |v0|^2|v1|^2 - (v0·v1)^2
        let denom = d00 * d11 - d01 * d01;

        // Relative-ish guard: if triangle is very small, denom will be tiny.
        // We keep it deterministic by a fixed epsilon, but scaled a bit by magnitude.
        let scale = (d00 * d11).abs().max(1.0);
        if !denom.is_finite() || denom.abs() <= EPS_DENOM * scale {
            return Vec3::new(-1.0, -1.0, -1.0);
        }

        let inv_denom = 1.0 / denom;
        let v = (d11 * d20 - d01 * d21) * inv_denom;
        let w = (d00 * d21 - d01 * d20) * inv_denom;
        let u = 1.0 - v - w;

        if !(u.is_finite() && v.is_finite() && w.is_finite()) {
            return Vec3::new(-1.0, -1.0, -1.0);
        }

        Vec3::new(u, v, w)
    }

    #[inline]
    pub fn contains_barycentric(&self, bary: Vec3) -> bool {
        let u = bary.x;
        let v = bary.y;
        let w = bary.z;
        u.is_finite() && v.is_finite() && w.is_finite() &&
            u >= EPS_BARY && v >= EPS_BARY && w >= EPS_BARY
    }

    /// Closest point on the triangle to `p` (Ericson-style region tests).
    /// Pure geometry; quantize at the call site if you want it hashed.
    pub fn closest_point(&self, p: Vec3) -> Vec3 {
        if !(self.a.is_finite() && self.b.is_finite() && self.c.is_finite() && p.is_finite()) {
            return self.a;
        }

        let a = self.a;
        let b = self.b;
        let c = self.c;

        let ab = b - a;
        let ac = c - a;
        let ap = p - a;

        // Region around vertex A
        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);
        if d1 <= 0.0 && d2 <= 0.0 { return a; }

        // Region around vertex B
        let bp = p - b;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0.0 && d4 <= d3 { return b; }

        // Region on edge AB
        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let denom = d1 - d3;
            if denom.abs() <= EPS_DENOM { return a; }
            let v = d1 / denom;
            return a + ab * v;
        }

        // Region around vertex C
        let cp = p - c;
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);
        if d6 >= 0.0 && d5 <= d6 { return c; }

        // Region on edge AC
        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let denom = d2 - d6;
            if denom.abs() <= EPS_DENOM { return a; }
            let w = d2 / denom;
            return a + ac * w;
        }

        // Region on edge BC
        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let denom = (d4 - d3) + (d5 - d6);
            if denom.abs() <= EPS_DENOM { return b; }
            let w = (d4 - d3) / denom;
            return b + (c - b) * w;
        }

        // Inside face region
        let denom = va + vb + vc;
        if !denom.is_finite() || denom.abs() <= EPS_DENOM {
            return a;
        }

        let inv_d = 1.0 / denom;
        let v = vb * inv_d;
        let w = vc * inv_d;
        let u = 1.0 - v - w;

        a * u + b * v + c * w
    }

    /// Ray–triangle intersection using Plane + barycentrics.
    ///
    /// Returns (t, bary) where:
    ///   - t is the ray parameter
    ///   - bary = (u,v,w) are barycentric coords at the hit
    ///
    /// No quantization here; caller can quantize t for hashing if needed.
    pub fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<(f32, Vec3)> {
        if t_min > t_max { return None; }
        if self.is_degenerate() { return None; }

        let pl = self.plane_struct()?;

        let t = pl.intersect_ray(ray, t_min, t_max)?;
        if !t.is_finite() { return None; }

        let hit = ray.point_at(t);
        let bary = self.barycentric(hit);

        if self.contains_barycentric(bary) {
            Some((t, bary))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use riftphys_core::types::Vec3;

    #[test]
    fn barycentric_center() {
        let tri = Triangle::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        );
        let p = Vec3::new(1.0 / 3.0, 0.0, 1.0 / 3.0);
        let bc = tri.barycentric(p);
        let sum = bc.x + bc.y + bc.z;
        assert!((sum - 1.0).abs() < 1.0e-5);
        assert!(bc.x > 0.0 && bc.y > 0.0 && bc.z > 0.0);
    }

    #[test]
    fn ray_triangle_hit() {
        let tri = Triangle::new(
            Vec3::new(-1.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, 1.0),
        );
        let ray = Ray::new(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0));

        let res = tri.intersect_ray(&ray, 0.0, 10.0);
        assert!(res.is_some());
        let (t, bary) = res.unwrap();
        let hit = ray.point_at(t);
        assert!(hit.y.abs() < 1.0e-5);
        assert!(tri.contains_barycentric(bary));
    }

    #[test]
    fn degenerate_triangle_rejected() {
        let tri = Triangle::new(Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        assert!(tri.is_degenerate());
        let ray = Ray::new(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0));
        assert!(tri.intersect_ray(&ray, 0.0, 10.0).is_none());
    }
}
