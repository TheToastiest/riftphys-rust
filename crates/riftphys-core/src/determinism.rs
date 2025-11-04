use blake3::Hasher;

#[derive(Copy, Clone, Debug)]
pub struct DeterminismContract {
    pub fixed_dt: f32,
    pub float: &'static str,
    pub fma: bool,
    pub iterations: u32,
    pub stable_sorts: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct Units {
    pub length: &'static str,
    pub mass:   &'static str,
    pub time:   &'static str,
}

impl DeterminismContract {
    pub fn default_contract() -> Self {
        Self {
            fixed_dt: 1.0/60.0,
            float: "f32",
            fma: false,
            iterations: 8,
            stable_sorts: true,
        }
    }
}
#[inline(always)]
pub fn qf32(x: f32, q: f32) -> f32 {
    // quantize to multiples of q (e.g., 1e-6)
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
    pub fn quantized(self, q: f32) -> [f32; 3] {
        [qf32(self.0[0], q), qf32(self.0[1], q), qf32(self.0[2], q)]
    }
}

pub trait DeterministicHash {
    fn hash_to(&self, h: &mut Hasher, q: f32);
}