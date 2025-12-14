// riftphys-acceleration/src/propulsion.rs

use riftphys_core::StepCtx;
use riftphys_core::Vec3;
use riftphys_core::models::{PropQuery, PropulsionModel};

/// Quantize to 1e-6 for cross-platform stability.
#[inline]
fn q6(x: f32) -> f32 {
    if !x.is_finite() { return 0.0; }
    (x * 1.0e6_f32).round() * 1.0e-6_f32
}

#[inline]
fn q6v(v: Vec3) -> Vec3 {
    Vec3::new(q6(v.x), q6(v.y), q6(v.z))
}

/// Deterministic-ish normalize: quantize input, guard tiny/NaN, normalize, quantize output.
#[inline]
fn normalize_q6(v: Vec3) -> Vec3 {
    let v = q6v(v);
    let len2 = v.length_squared();
    if !len2.is_finite() || len2 <= 1.0e-20 {
        return Vec3::ZERO;
    }
    let inv_len = 1.0 / len2.sqrt();
    q6v(v * inv_len)
}

#[derive(Copy, Clone, Debug)]
pub enum ThrottleCurve {
    Linear,
    QuadEaseIn,
    /// Polynomial coefficients p0..p4 (up to 4th order), evaluated with Horner.
    Custom([f32; 5]),
}

#[inline]
fn eval_curve(c: &ThrottleCurve, t_in: f32) -> f32 {
    // Treat non-finite throttle as 0.
    let t = if t_in.is_finite() { t_in } else { 0.0 };
    let t = q6(t.clamp(0.0, 1.0));

    match c {
        ThrottleCurve::Linear => t,
        ThrottleCurve::QuadEaseIn => q6(t * t),
        ThrottleCurve::Custom(p) => {
            // Horner with quantization each step.
            let t = q6(t);
            let mut y = q6(p[4]);
            y = q6(y * t + q6(p[3]));
            y = q6(y * t + q6(p[2]));
            y = q6(y * t + q6(p[1]));
            y = q6(y * t + q6(p[0]));
            y
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SimplePropulsion {
    /// Baseline thrust added even at throttle=0.
    pub thrust_constant_n: f32,
    /// Thrust scale driven by the curve gain.
    pub thrust_max_n: f32,
    pub curve: ThrottleCurve,
}

impl PropulsionModel for SimplePropulsion {
    fn accel_contrib(&self, _ctx: &StepCtx, q: PropQuery) -> [f32; 3] {
        // Mass and throttle sanity.
        let mass = q6(q.mass);
        if !(mass > 0.0) {
            return [0.0; 3];
        }

        // Curve gain in [0..1], NaN-safe.
        let gain = eval_curve(&self.curve, q.throttle01).max(0.0);
        let gain = q6(gain);

        // Thrust and accel magnitude (quantized).
        let thrust_c = q6(self.thrust_constant_n);
        let thrust_m = q6(self.thrust_max_n);
        let thrust = q6(thrust_c + q6(thrust_m * gain));
        let a_mag = q6(thrust / mass);

        // Forward dir (world). Quantize + deterministic normalize.
        let dir_in = Vec3::from_array(q.forward_dir_world);
        let dir = normalize_q6(dir_in);

        // Final accel (quantized output).
        let a = q6v(dir * a_mag);
        [a.x, a.y, a.z]
    }
}
