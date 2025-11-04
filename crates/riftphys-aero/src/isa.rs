// riftphys-aero/src/isa.rs

pub struct ISA {
    pub rho0: f32, // 1.225 kg/m^3
    pub T0: f32,   // 288.15 K
    pub L: f32,    // 0.0065 K/m
    pub R: f32,    // 287.05 J/(kg*K)
    pub g: f32,    // 9.80665 m/s^2
}

impl Default for ISA {
    fn default() -> Self {
        Self { rho0: 1.225, T0: 288.15, L: 0.0065, R: 287.05, g: 9.80665 }
    }
}

impl ISA {
    // valid up to ~11 km
    pub fn density(&self, altitude_m: f32) -> f32 {
        let a = 1.0 - self.L * altitude_m / self.T0;
        // ρ = ρ0 * (T/T0)^{(g/(R L) - 1)}
        let exp = self.g / (self.R * self.L) - 1.0; // ≈ 4.25588
        self.rho0 * a.max(0.0).powf(exp)
    }
}
