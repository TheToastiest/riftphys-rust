use blake3::Hasher;
use crate::types::{Quat, Vec3};

pub struct StepHasher(Hasher);

impl StepHasher {
    #[inline] pub fn new() -> Self { StepHasher(Hasher::new()) }

    #[inline] pub fn update_bytes(&mut self, bytes: &[u8]) { self.0.update(bytes); }

    #[inline] pub fn update_u8(&mut self, v: u8) { self.0.update(&[v]); }
    #[inline] pub fn update_u32(&mut self, v: u32) { self.0.update(&v.to_le_bytes()); }
    #[inline] pub fn update_u64(&mut self, v: u64) { self.0.update(&v.to_le_bytes()); }

    #[inline]
    pub fn update_f32_bits(&mut self, bits: u32) {
        self.0.update(&bits.to_le_bytes());
    }

    #[inline]
    pub fn finalize(self) -> [u8; 32] { *self.0.finalize().as_bytes() }
}

// Canonicalize float bits so hashing is stable even if NaNs/Infs appear.
// -0.0 collapses to +0.0.
// Non-finite collapses to a fixed quiet-NaN sentinel.
#[inline]
pub fn canon_f32_bits(x: f32) -> u32 {
    if !x.is_finite() {
        return 0x7fc0_0000; // canonical quiet NaN
    }
    let b = x.to_bits();
    if b == 0x8000_0000 { 0 } else { b } // -0.0 -> +0.0
}

#[inline]
pub fn hash_vec3(h: &mut StepHasher, v: &Vec3) {
    h.update_f32_bits(canon_f32_bits(v.x));
    h.update_f32_bits(canon_f32_bits(v.y));
    h.update_f32_bits(canon_f32_bits(v.z));
}

#[inline]
pub fn hash_quat(h: &mut StepHasher, q: &Quat) {
    h.update_f32_bits(canon_f32_bits(q.x));
    h.update_f32_bits(canon_f32_bits(q.y));
    h.update_f32_bits(canon_f32_bits(q.z));
    h.update_f32_bits(canon_f32_bits(q.w));
}
