#[derive(Copy, Clone, Debug)]
pub struct XorShift64 { state: u64 }

impl XorShift64 {
    #[inline]
    pub fn new(seed: u64) -> Self {
        // Force non-zero, deterministic.
        Self { state: (seed | 1) }
    }

    #[inline]
    pub fn state(&self) -> u64 { self.state }

    #[inline]
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        ((x.wrapping_mul(2685821657736338717)) >> 32) as u32
    }

    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        let hi = self.next_u32() as u64;
        let lo = self.next_u32() as u64;
        (hi << 32) | lo
    }

    /// Deterministic float in [0, 1).
    #[inline]
    pub fn next_f32_01(&mut self) -> f32 {
        // Construct f32 from bits: sign 0, exponent 127, 23 bits of mantissa
        // This gives a value in [1.0, 2.0), then subtract 1.0.
        let u = self.next_u32();
        let bits = (u >> 9) | 0x3f80_0000;
        f32::from_bits(bits) - 1.0
    }
    /// Deterministic integer in [0, bound). bound must be > 0.
    #[inline]
    pub fn next_u32_bounded(&mut self, bound: u32) -> u32 {
        debug_assert!(bound > 0);
        // Rejection sampling to avoid modulo bias.
        let threshold = u32::MAX - (u32::MAX % bound);
        loop {
            let v = self.next_u32();
            if v < threshold {
                return v % bound;
            }
        }
    }
}
