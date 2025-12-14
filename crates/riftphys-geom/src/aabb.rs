use riftphys_core::determinism::{qf32, DEFAULT_QF32};
use riftphys_core::types::Vec3;

#[inline(always)]
fn qv3(v: Vec3) -> Vec3 {
    Vec3::new(
        qf32(v.x, DEFAULT_QF32),
        qf32(v.y, DEFAULT_QF32),
        qf32(v.z, DEFAULT_QF32),
    )
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    #[inline]
    pub fn new(min: Vec3, max: Vec3) -> Self {
        // Quantize first, then enforce ordering.
        let a = qv3(min);
        let b = qv3(max);
        Self { min: a.min(b), max: a.max(b) }
    }

    #[inline]
    pub fn from_center_half_extents(c: Vec3, he: Vec3) -> Self {
        // Sorting happens in new().
        Self::new(c - he, c + he)
    }

    #[inline]
    pub fn overlaps(&self, other: &Aabb) -> bool {
        !(self.max.x < other.min.x || self.min.x > other.max.x ||
            self.max.y < other.min.y || self.min.y > other.max.y ||
            self.max.z < other.min.z || self.min.z > other.max.z)
    }

    #[inline]
    pub fn expand_by(&mut self, r: f32) {
        // Expand must preserve quantization contract for stable hashing/bucketing.
        let rr = qf32(r.abs(), DEFAULT_QF32);
        let e = Vec3::splat(rr);

        self.min = qv3(self.min - e);
        self.max = qv3(self.max + e);

        // Preserve invariant min <= max.
        let mn = self.min.min(self.max);
        let mx = self.min.max(self.max);
        self.min = mn;
        self.max = mx;
    }
    #[inline]
    pub fn debug_assert_valid(&self) {
        debug_assert!(self.min.x <= self.max.x);
        debug_assert!(self.min.y <= self.max.y);
        debug_assert!(self.min.z <= self.max.z);
    }
    #[inline]
    pub fn union(self, other: Aabb) -> Aabb {
        Aabb {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    #[inline]
    pub fn expand(self, e: f32) -> Aabb {
        let v = Vec3::splat(e);
        Aabb { min: self.min - v, max: self.max + v }
    }
}
