use riftphys_core::{Vec3, StepHasher};

/// A single gravitating body with standard gravitational parameter μ = G*M
#[derive(Copy, Clone, Debug)]
pub struct PointMass {
    pub mu: f32,             // in your world units^3 / s^2
    pub center: [f32; 3],    // world-space position of the body
}

#[inline]
pub const fn point_mass(mu: f32, center: [f32; 3]) -> PointMass {
    PointMass { mu, center }
}

#[derive(Copy, Clone, Debug)]
pub enum GravitySpec {
    /// Constant acceleration (e.g., Earth: [0,-9.81,0])
    Uniform { g: [f32; 3] },

    /// Point-mass field: a = -mu * (r̂) / r^2  (mu = G*M). Center in world coords.
    InverseSquare { mu: f32, center: [f32; 3], min_r: f32 },

    /// Simple planet: inverse-square from a center & radius, clamped inside.
    LayeredPlanet { surface_g: f32, radius: f32, center: [f32; 3], min_r: f32 },

    /// Sum of inverse-square fields from multiple bodies (e.g., Sun/Earth/Moon).
    /// `bodies` should point to a static slice so the contents are stable.
    MultiBody { bodies: &'static [PointMass], min_r: f32 },
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
        GravitySpec::MultiBody { bodies, min_r } => {
            let mut a = Vec3::ZERO;
            let min_r2 = min_r * min_r;
            for b in bodies {
                let c = Vec3::new(b.center[0], b.center[1], b.center[2]);
                let r = p - c;                     // vector from body → point
                let r2 = r.length_squared().max(min_r2);
                if r2 == 0.0 { continue; }
                let inv_r3 = 1.0 / (r2 * r2.sqrt());
                a += -r * (b.mu * inv_r3);        // sum contributions
            }
            a
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
        GravitySpec::MultiBody { bodies, min_r } => {
            h.update_bytes(&[3u8]);
            h.update_bytes(&min_r.to_le_bytes());
            // Hash the full list deterministically: length + per-body fields.
            let len = bodies.len() as u32;
            h.update_bytes(&len.to_le_bytes());
            for b in bodies {
                h.update_bytes(&b.mu.to_le_bytes());
                for f in b.center { h.update_bytes(&f.to_le_bytes()); }
            }
        }

    }
    let b = h.finalize();
    u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
}
