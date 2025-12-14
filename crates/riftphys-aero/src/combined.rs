use std::sync::Arc;

use riftphys_core::StepCtx;
use riftphys_core::models::{AeroModel, AeroQuery};

#[inline]
fn q6(x: f32) -> f32 {
    if !x.is_finite() { return 0.0; }
    (x * 1.0e6_f32).round() * 1.0e-6_f32
}

/// Deterministic composition wrapper.
///
/// Determinism note:
/// - The *order* of `parts` is the order of accumulation. Keep it stable.
/// - We quantize each part contribution and the running sum to reduce
///   cross-platform float drift.
pub struct CombinedAero {
    parts: Vec<Arc<dyn AeroModel>>,
}

impl CombinedAero {
    /// Create a combined aero model from a stable-ordered list of parts.
    pub fn new(parts: Vec<Arc<dyn AeroModel>>) -> Self {
        Self { parts }
    }

    /// Append a part (keeps deterministic order as pushed).
    pub fn push(&mut self, part: Arc<dyn AeroModel>) {
        self.parts.push(part);
    }

    /// Immutable view of parts (debug/inspection).
    pub fn parts(&self) -> &[Arc<dyn AeroModel>] {
        &self.parts
    }
}

impl AeroModel for CombinedAero {
    fn accel_contrib(&self, ctx: &StepCtx, q: AeroQuery) -> [f32; 3] {
        let mut ax = 0.0f32;
        let mut ay = 0.0f32;
        let mut az = 0.0f32;

        for p in &self.parts {
            let a = p.accel_contrib(ctx, q);
            // quantize per-part contribution
            let px = q6(a[0]);
            let py = q6(a[1]);
            let pz = q6(a[2]);

            // quantize running sum each step
            ax = q6(ax + px);
            ay = q6(ay + py);
            az = q6(az + pz);
        }

        [q6(ax), q6(ay), q6(az)]
    }
}
