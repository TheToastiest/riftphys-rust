use riftphys_core::{Vec3, StepHasher};

#[derive(Copy, Clone, Debug)]
pub enum GravitySpec {
    /// Constant acceleration (e.g., Earth: [0,-9.81,0])
    Uniform { g: [f32; 3] },

    /// Point-mass field: a = -mu * (r̂) / r^2  (mu = G*M). Center in world coords.
    InverseSquare { mu: f32, center: [f32; 3], min_r: f32 },

    /// Simple planet: inverse-square from a center & radius, clamped inside.
    LayeredPlanet { surface_g: f32, radius: f32, center: [f32; 3], min_r: f32 },
}

#[inline]
pub fn eval(spec: &GravitySpec, p: Vec3) -> Vec3 {
    match *spec {
        GravitySpec::Uniform { g } => Vec3::new(g[0], g[1], g[2]),

        GravitySpec::InverseSquare { mu, center, min_r } => {
            let c = Vec3::new(center[0], center[1], center[2]);
            let r = p - c;
            let r2 = r.length_squared().max(min_r * min_r);
            if r2 == 0.0 { return Vec3::ZERO; }
            let inv_r3 = 1.0 / (r2 * r2.sqrt());
            -r * (mu * inv_r3)
        }

        GravitySpec::LayeredPlanet { surface_g, radius, center, min_r } => {
            let c = Vec3::new(center[0], center[1], center[2]);
            let rv = p - c;
            let dist = rv.length().max(min_r);
            if dist == 0.0 { return Vec3::ZERO; }
            let dir = rv / dist;
            // g = g_surface * (R^2 / r^2) — very simple profile
            let g_mag = surface_g * (radius * radius) / (dist * dist);
            -dir * g_mag
        }
    }
}

/// Deterministic 64-bit ID for a gravity spec (used as EpochID when swapping).
#[inline]
pub fn spec_id(spec: &GravitySpec) -> u64 {
    let mut h = StepHasher::new();
    match *spec {
        GravitySpec::Uniform { g } => {
            h.update_bytes(&[0u8]); for f in g { h.update_bytes(&f.to_le_bytes()); }
        }
        GravitySpec::InverseSquare { mu, center, min_r } => {
            h.update_bytes(&[1u8]);
            h.update_bytes(&mu.to_le_bytes());
            for f in center { h.update_bytes(&f.to_le_bytes()); }
            h.update_bytes(&min_r.to_le_bytes());
        }
        GravitySpec::LayeredPlanet { surface_g, radius, center, min_r } => {
            h.update_bytes(&[2u8]);
            h.update_bytes(&surface_g.to_le_bytes());
            h.update_bytes(&radius.to_le_bytes());
            for f in center { h.update_bytes(&f.to_le_bytes()); }
            h.update_bytes(&min_r.to_le_bytes());
        }
    }
    let b = h.finalize();
    u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
}
