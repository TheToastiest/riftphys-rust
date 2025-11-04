use std::sync::Arc;
use riftphys_core::{StepCtx};
use riftphys_core::models::{AeroModel, AeroQuery};

pub struct CombinedAero {
    parts: Vec<Arc<dyn AeroModel>>,
}

impl CombinedAero {
    pub fn new(parts: Vec<Arc<dyn AeroModel>>) -> Self { Self { parts } }
}

impl AeroModel for CombinedAero {
    fn accel_contrib(&self, ctx: &StepCtx, q: AeroQuery) -> [f32; 3] {
        let mut ax = 0.0f32; let mut ay = 0.0f32; let mut az = 0.0f32;
        for p in &self.parts {
            let a = p.accel_contrib(ctx, q);
            ax += a[0]; ay += a[1]; az += a[2];
        }
        [ax, ay, az]
    }
}
