// riftphys-geom/src/plane.rs
use riftphys_core::types::Vec3;
use crate::ray::Ray;

const EPS_NORM: f32 = 1.0e-6;
const EPS_DENOM: f32 = 1.0e-6;

#[derive(Copy, Clone, Debug)]
pub struct Plane {
    pub n: Vec3, // unit normal
    pub d: f32,  // n·x + d = 0
}

impl Plane {
    /// From point + normal; returns None if the normal is degenerate or inputs are non-finite.
    pub fn from_point_normal(p: Vec3, n: Vec3) -> Option<Self> {
        // Reject non-finite inputs early (keeps the rest of the engine from turning into NaN soup).
        if !(p.is_finite() && n.is_finite()) {
            return None;
        }

        let len2 = n.length_squared();
        if !len2.is_finite() || len2 <= (EPS_NORM * EPS_NORM) {
            return None;
        }

        let inv_len = 1.0 / len2.sqrt();
        let nu = n * inv_len; // unit normal
        let d = -nu.dot(p);

        if !(nu.is_finite() && d.is_finite()) {
            return None;
        }

        Some(Self { n: nu, d })
    }

    /// From three non-collinear points; returns None if collinear/degenerate or non-finite.
    pub fn from_points(a: Vec3, b: Vec3, c: Vec3) -> Option<Self> {
        if !(a.is_finite() && b.is_finite() && c.is_finite()) {
            return None;
        }
        let ab = b - a;
        let ac = c - a;
        let n = ab.cross(ac);
        Self::from_point_normal(a, n)
    }

    #[inline]
    pub fn signed_distance(&self, p: Vec3) -> f32 {
        self.n.dot(p) + self.d
    }

    #[inline]
    pub fn project_point(&self, p: Vec3) -> Vec3 {
        let dist = self.signed_distance(p);
        // If dist is non-finite, don't inject NaNs into downstream math.
        if !dist.is_finite() { return p; }
        p - self.n * dist
    }

    #[inline]
    pub fn flip(&self) -> Self {
        Self { n: -self.n, d: -self.d }
    }

    /// Ray–plane intersection. Returns t in [t_min, t_max] if hit.
    pub fn intersect_ray(&self, ray: &Ray, t_min: f32, t_max: f32) -> Option<f32> {
        if t_min > t_max { return None; } // refuse inverted ranges deterministically
        if !(self.n.is_finite() && self.d.is_finite() && ray.origin.is_finite() && ray.dir.is_finite()) {
            return None;
        }

        let denom = self.n.dot(ray.dir);
        if !denom.is_finite() || denom.abs() < EPS_DENOM {
            return None; // parallel / nearly so / broken input
        }

        let numer = -(self.n.dot(ray.origin) + self.d);
        if !numer.is_finite() { return None; }

        let t = numer / denom;
        if !t.is_finite() { return None; }

        if t >= t_min && t <= t_max { Some(t) } else { None }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use riftphys_core::types::Vec3;

    #[test]
    fn plane_from_point_normal() {
        let p = Vec3::new(0.0, 1.0, 0.0);
        let n = Vec3::new(0.0, 1.0, 0.0);
        let pl = Plane::from_point_normal(p, n).unwrap();
        // y = 1 plane: n·x + d = 0 -> x.y + d = 0 -> 1 + d = 0 -> d = -1
        assert!((pl.d + 1.0).abs() < 1.0e-6);
        assert!((pl.n.y - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn ray_hits_plane() {
        // Plane y = 0: n = +Y, passing through origin => d = 0
        let pl = Plane { n: Vec3::new(0.0, 1.0, 0.0), d: 0.0 };
        let ray = Ray::new(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0));

        let t = pl.intersect_ray(&ray, 0.0, 10.0).unwrap();
        let hit = ray.point_at(t);
        assert!((hit.y - 0.0).abs() < 1.0e-5);
    }
}
