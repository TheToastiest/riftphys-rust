use riftphys_core::types::Vec3;

#[derive(Copy, Clone, Debug, Default)]
pub struct Aabb { pub min: Vec3, pub max: Vec3 }

impl Aabb {
    #[inline] pub fn new(min: Vec3, max: Vec3) -> Self { Self { min, max } }
    #[inline] pub fn from_center_half_extents(c: Vec3, he: Vec3) -> Self {
        Self { min: c - he, max: c + he }
    }
    #[inline] pub fn overlaps(&self, other: &Aabb) -> bool {
        !(self.max.x < other.min.x || self.min.x > other.max.x ||
            self.max.y < other.min.y || self.min.y > other.max.y ||
            self.max.z < other.min.z || self.min.z > other.max.z)
    }
    #[inline] pub fn expand_by(&mut self, r: f32) {
        let e = Vec3::splat(r);
        self.min -= e; self.max += e;
    }
}
