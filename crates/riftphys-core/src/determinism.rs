use blake3::Hasher;

use crate::types::{Isometry, Quat, Vec3, Velocity};

/// High-level assumptions your determinism contract is built on.
/// This is documentation + runtime config, not magic.
#[derive(Copy, Clone, Debug)]
pub struct DeterminismContract {
    pub fixed_dt: f32,
    pub float: &'static str,
    pub fma: bool,
    pub iterations: u32,
    pub stable_sorts: bool,
}

/// Canonical units (documentation helper; you can stash this into telemetry).
#[derive(Copy, Clone, Debug)]
pub struct Units {
    pub length: &'static str,
    pub mass:   &'static str,
    pub time:   &'static str,
}

impl DeterminismContract {
    /// Old helper kept for backwards compatibility.
    #[inline]
    pub fn default_contract() -> Self { Self::default() }
}

impl Default for DeterminismContract {
    fn default() -> Self {
        Self {
            fixed_dt: 1.0 / 60.0,
            float: "f32",
            fma: false,
            iterations: 8,
            stable_sorts: true,
        }
    }
}

/// Default quantization step used for world-facing float hashing.
pub const DEFAULT_QF32: f32 = 1.0e-6;

// ----- float canonicalization + quantization primitives -----

#[inline(always)]
pub fn canon_f32_bits(x: f32) -> u32 {
    if !x.is_finite() {
        return 0x7fc0_0000; // canonical quiet NaN
    }
    let b = x.to_bits();
    if b == 0x8000_0000 { 0 } else { b } // -0.0 -> +0.0
}

#[inline(always)]
pub fn qf32(x: f32, q: f32) -> f32 {
    if !x.is_finite() {
        return f32::NAN; // will hash to sentinel
    }
    if q <= 0.0 || !q.is_finite() {
        return x; // refuse to do something nonsensical
    }
    (x / q).round() * q
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Qf32(pub f32);

impl Qf32 {
    #[inline(always)]
    pub fn quantized(self, q: f32) -> f32 { qf32(self.0, q) }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Qv3(pub [f32; 3]);

impl Qv3 {
    #[inline(always)]
    pub fn quantized(self, q: f32) -> [f32; 3] {
        [qf32(self.0[0], q), qf32(self.0[1], q), qf32(self.0[2], q)]
    }
}

// ----- deterministic hashing -----

/// Anything that wants to appear in the world hash implements this.
/// `q` is the quantization step (usually 1e-6).
pub trait DeterministicHash {
    fn hash_to(&self, h: &mut Hasher, q: f32);
}

#[inline(always)]
fn hash_u32(h: &mut Hasher, v: u32) { h.update(&v.to_le_bytes()); }
#[inline(always)]
fn hash_u64(h: &mut Hasher, v: u64) { h.update(&v.to_le_bytes()); }

#[inline(always)]
fn hash_f32_q(h: &mut Hasher, x: f32, q: f32) {
    let xq = qf32(x, q);
    let bits = canon_f32_bits(xq);
    h.update(&bits.to_le_bytes());
}

#[inline(always)]
fn hash_vec3_q(h: &mut Hasher, v: &Vec3, q: f32) {
    hash_f32_q(h, v.x, q);
    hash_f32_q(h, v.y, q);
    hash_f32_q(h, v.z, q);
}

// Canonical quaternion sign so q and -q hash identically.
// We quantize first, then apply sign canonicalization.
#[inline(always)]
fn canon_quat_q(qin: Quat, qstep: f32) -> Quat {
    let mut q = qin;

    // Quantize components to stabilize the sign-choice boundary.
    q.x = qf32(q.x, qstep);
    q.y = qf32(q.y, qstep);
    q.z = qf32(q.z, qstep);
    q.w = qf32(q.w, qstep);

    // Choose a unique representative using lexicographic priority (w, z, y, x).
    // If the first non-zero component is negative, flip.
    let flip =
        (q.w < 0.0) ||
            (q.w == 0.0 && q.z < 0.0) ||
            (q.w == 0.0 && q.z == 0.0 && q.y < 0.0) ||
            (q.w == 0.0 && q.z == 0.0 && q.y == 0.0 && q.x < 0.0);

    if flip { -q } else { q }
}

#[inline(always)]
fn hash_quat_q(h: &mut Hasher, q: Quat, qstep: f32) {
    let qc = canon_quat_q(q, qstep);
    hash_f32_q(h, qc.x, qstep);
    hash_f32_q(h, qc.y, qstep);
    hash_f32_q(h, qc.z, qstep);
    hash_f32_q(h, qc.w, qstep);
}

// ---- impls ----

impl DeterministicHash for u32 {
    #[inline] fn hash_to(&self, h: &mut Hasher, _q: f32) { hash_u32(h, *self); }
}
impl DeterministicHash for u64 {
    #[inline] fn hash_to(&self, h: &mut Hasher, _q: f32) { hash_u64(h, *self); }
}
impl DeterministicHash for bool {
    #[inline] fn hash_to(&self, h: &mut Hasher, _q: f32) { h.update(&[*self as u8]); }
}
impl DeterministicHash for f32 {
    #[inline] fn hash_to(&self, h: &mut Hasher, q: f32) { hash_f32_q(h, *self, q); }
}
impl DeterministicHash for Vec3 {
    #[inline] fn hash_to(&self, h: &mut Hasher, q: f32) { hash_vec3_q(h, self, q); }
}
impl DeterministicHash for Quat {
    #[inline] fn hash_to(&self, h: &mut Hasher, q: f32) { hash_quat_q(h, *self, q); }
}
impl DeterministicHash for Isometry {
    #[inline]
    fn hash_to(&self, h: &mut Hasher, q: f32) {
        self.pos.hash_to(h, q);
        self.rot.hash_to(h, q);
    }
}
impl DeterministicHash for Velocity {
    #[inline]
    fn hash_to(&self, h: &mut Hasher, q: f32) {
        self.lin.hash_to(h, q);
        self.ang.hash_to(h, q);
    }
}

impl<T: DeterministicHash, const N: usize> DeterministicHash for [T; N] {
    #[inline]
    fn hash_to(&self, h: &mut Hasher, q: f32) {
        for v in self { v.hash_to(h, q); }
    }
}
