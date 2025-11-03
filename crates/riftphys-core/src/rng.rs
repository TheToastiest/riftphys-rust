#[derive(Copy, Clone, Debug)]
pub struct XorShift64 { state: u64 }

impl XorShift64 {
    pub fn new(seed: u64) -> Self { Self { state: seed | 1 } }
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.state;
        x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
        self.state = x;
        ((x.wrapping_mul(2685821657736338717)) >> 32) as u32
    }
    pub fn state(&self) -> u64 { self.state }
}
