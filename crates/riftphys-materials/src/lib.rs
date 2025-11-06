use std::cmp::Ordering;

/// Material catalog is fixed and deterministic.
/// Add more IDs here; order matters only for display (pair mixing is symmetric).
#[derive(Copy,Clone,Debug,Eq,PartialEq,Hash)]
pub enum MaterialId {
    Default, Ice, Rubber, Steel, Grit,
    Wood, Concrete, Asphalt, Leather, Skin, Carpet, Glass,
}

/// Canonical single-material properties.
/// Units:
/// - friction coefficients are dimensionless
/// - restitution in [0,1]
/// - stribeck_v (m/s) sets transition speed from static→kinetic
#[derive(Copy,Clone,Debug)]
pub struct MatProps {
    pub mu_s: f32,         // static
    pub mu_k: f32,         // kinetic (high-speed plateau)
    pub restitution: f32,  // bounciness
    pub stribeck_v: f32,   // characteristic slip speed (m/s)
}

/// Pair properties after mixing + overrides; what the solver should use.
#[derive(Copy,Clone,Debug)]
pub struct MatPairProps {
    pub mu_s: f32,
    pub mu_k: f32,        // kinetic plateau (used when v is high)
    pub restitution: f32,
    pub stribeck_v: f32,  // blended; used to compute mu_k(v)
}

/// Base single-material table.
pub fn props(id: MaterialId) -> MatProps {
    use MaterialId::*;
    match id {
        Default  => MatProps { mu_s: 0.60, mu_k: 0.50, restitution: 0.0,  stribeck_v: 0.10 },
        Ice      => MatProps { mu_s: 0.03, mu_k: 0.02, restitution: 0.02, stribeck_v: 0.02 },
        Rubber   => MatProps { mu_s: 1.20, mu_k: 1.00, restitution: 0.80, stribeck_v: 0.15 },
        Steel    => MatProps { mu_s: 0.60, mu_k: 0.50, restitution: 0.10, stribeck_v: 0.05 },
        Grit     => MatProps { mu_s: 1.30, mu_k: 1.05, restitution: 0.00, stribeck_v: 0.12 },
        Wood     => MatProps { mu_s: 0.55, mu_k: 0.45, restitution: 0.05, stribeck_v: 0.08 },
        Concrete => MatProps { mu_s: 0.80, mu_k: 0.70, restitution: 0.00, stribeck_v: 0.10 },
        Asphalt  => MatProps { mu_s: 0.90, mu_k: 0.80, restitution: 0.00, stribeck_v: 0.12 },
        Leather  => MatProps { mu_s: 0.80, mu_k: 0.60, restitution: 0.20, stribeck_v: 0.10 },
        Skin     => MatProps { mu_s: 0.55, mu_k: 0.45, restitution: 0.15, stribeck_v: 0.06 },
        Carpet   => MatProps { mu_s: 1.10, mu_k: 0.95, restitution: 0.00, stribeck_v: 0.15 },
        Glass    => MatProps { mu_s: 0.40, mu_k: 0.35, restitution: 0.30, stribeck_v: 0.04 },
    }
}

/// Optional explicit pair overrides (deterministic and symmetric).
/// If not present, we use the default mixing law.
#[derive(Copy,Clone,Debug)]
pub struct PairOverride {
    pub mu_s: Option<f32>,
    pub mu_k: Option<f32>,
    pub restitution: Option<f32>,
    pub stribeck_v: Option<f32>,
}

/// A small, curated override table for well-known combinations.
fn override_for(a: MaterialId, b: MaterialId) -> Option<PairOverride> {
    use MaterialId::*;
    // Order-invariant: put (min,max)
    let (x,y) = ordered_pair(a,b);
    match (x,y) {
        (Ice, Rubber) => Some(PairOverride {
            // Rubber on ice often shows strong speed dependence and lower restitution.
            mu_s: Some(0.20), mu_k: Some(0.10), restitution: Some(0.05), stribeck_v: Some(0.08)
        }),
        (Rubber, Asphalt) => Some(PairOverride {
            mu_s: Some(1.10), mu_k: Some(0.95), restitution: Some(0.00), stribeck_v: Some(0.18)
        }),
        (Rubber, Concrete) => Some(PairOverride {
            mu_s: Some(1.00), mu_k: Some(0.85), restitution: Some(0.00), stribeck_v: Some(0.16)
        }),
        (Steel, Steel) => Some(PairOverride {
            mu_s: Some(0.58), mu_k: Some(0.50), restitution: Some(0.08), stribeck_v: Some(0.04)
        }),
        _ => None
    }
}

/// Symmetric (order-independent) pair mix with geometric-mean friction and max restitution.
fn mix_pair(a: MatProps, b: MatProps) -> MatPairProps {
    let mu_s = (a.mu_s * b.mu_s).abs().sqrt();
    let mu_k = (a.mu_k * b.mu_k).abs().sqrt();
    let restitution = a.restitution.max(b.restitution);
    let stribeck_v  = 0.5 * (a.stribeck_v + b.stribeck_v);
    MatPairProps { mu_s, mu_k, restitution, stribeck_v }
}

/// Public entry: compute pair properties with overrides applied.
pub fn pair_props(a: MaterialId, b: MaterialId) -> MatPairProps {
    let base = mix_pair(props(a), props(b));
    if let Some(ov) = override_for(a,b) {
        MatPairProps {
            mu_s: ov.mu_s.unwrap_or(base.mu_s),
            mu_k: ov.mu_k.unwrap_or(base.mu_k),
            restitution: ov.restitution.unwrap_or(base.restitution),
            stribeck_v: ov.stribeck_v.unwrap_or(base.stribeck_v),
        }
    } else {
        base
    }
}

/// Deterministic Stribeck curve for dynamic friction coefficient at slip speed `v`.
/// mu(v) = mu_k + (mu_s - mu_k) * exp(-(v / v_s)^2)
pub fn mu_dynamic(pair: &MatPairProps, v_slip: f32) -> f32 {
    let vs = pair.stribeck_v.max(1.0e-6);
    let t = -(v_slip / vs).powi(2);
    let mu = pair.mu_k + (pair.mu_s - pair.mu_k) * t.exp();
    quantize(mu)
}

/// Simple 1e-6 quantization for perfect determinism across platforms.
#[inline]
pub fn quantize(x: f32) -> f32 {
    (x * 1.0e6_f32).round() * 1.0e-6_f32
}

#[inline]
fn ordered_pair(a: MaterialId, b: MaterialId) -> (MaterialId, MaterialId) {
    match (a as u32).cmp(&(b as u32)) {
        Ordering::Less | Ordering::Equal => (a,b),
        Ordering::Greater                 => (b,a),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn symmetry() {
        let p1 = pair_props(MaterialId::Rubber, MaterialId::Ice);
        let p2 = pair_props(MaterialId::Ice, MaterialId::Rubber);
        assert!((p1.mu_s - p2.mu_s).abs() < 1e-12);
        assert!((p1.mu_k - p2.mu_k).abs() < 1e-12);
    }
    #[test] fn stribeck_limits() {
        let p = pair_props(MaterialId::Rubber, MaterialId::Concrete);
        let m0 = mu_dynamic(&p, 0.0);
        let m1 = mu_dynamic(&p, 10.0);
        assert!(m0 <= p.mu_s + 1e-6 && m0 >= p.mu_k - 1e-6);
        assert!((m1 - p.mu_k).abs() < 1e-3);
    }
}
