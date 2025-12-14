// riftphys-geom/src/ray.rs
use riftphys_core::types::Vec3;

const EPS_DIR2: f32 = 1.0e-24; // (1e-12)^2

#[derive(Copy, Clone, Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub dir:    Vec3, // not forced-normalized; caller decides
}

impl Ray {
    #[inline]
    pub fn new(origin: Vec3, dir: Vec3) -> Self {
        Self { origin, dir }
    }

    #[inline]
    pub fn from_points(origin: Vec3, target: Vec3) -> Self {
        Self { origin, dir: target - origin }
    }

    #[inline]
    pub fn point_at(&self, t: f32) -> Vec3 {
        // If t is NaN/Inf, don't spray NaNs into the world; return origin deterministically.
        if !t.is_finite() { return self.origin; }
        self.origin + self.dir * t
    }

    #[inline]
    pub fn dir_len2(&self) -> f32 {
        self.dir.length_squared()
    }

    #[inline]
    pub fn is_degenerate(&self) -> bool {
        let l2 = self.dir_len2();
        !l2.is_finite() || l2 <= EPS_DIR2
    }

    #[inline]
    pub fn normalized(self) -> Self {
        // Reject non-finite direction; deterministic fallback.
        if !self.dir.is_finite() || !self.origin.is_finite() {
            return Self { origin: self.origin, dir: Vec3::ZERO };
        }

        let len2 = self.dir.length_squared();
        if !len2.is_finite() || len2 <= EPS_DIR2 {
            Self { origin: self.origin, dir: Vec3::ZERO }
        } else {
            let inv_len = 1.0 / len2.sqrt();
            Self { origin: self.origin, dir: self.dir * inv_len }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_at_works() {
        let r = Ray::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        let p = r.point_at(2.5);
        assert!((p.y - 2.5).abs() < 1.0e-6);
    }

    #[test]
    fn normalized_degenerate() {
        let r = Ray::new(Vec3::ZERO, Vec3::ZERO).normalized();
        assert_eq!(r.dir, Vec3::ZERO);
    }
}
