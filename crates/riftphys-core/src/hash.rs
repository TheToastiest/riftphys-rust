use blake3::Hasher;
use crate::types::Vec3;
use glam::Quat;

pub struct StepHasher(Hasher);

impl StepHasher {
    pub fn new() -> Self { StepHasher(Hasher::new()) }
    pub fn update_bytes(&mut self, bytes: &[u8]) { self.0.update(bytes); }
    pub fn finalize(self) -> [u8; 32] { *self.0.finalize().as_bytes() }
}

#[inline]
pub fn hash_vec3(h: &mut StepHasher, v: &Vec3) {
    for c in [v.x, v.y, v.z] { h.update_bytes(&c.to_le_bytes()); }
}

#[inline]
pub fn hash_quat(h: &mut StepHasher, q: &Quat) {
    for c in [q.x, q.y, q.z, q.w] { h.update_bytes(&c.to_le_bytes()); }
}
