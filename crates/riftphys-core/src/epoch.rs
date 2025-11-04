use crate::StepHasher;
use crate::models::{ModelPackId, ModelRegistry};

/// Phase 3 minimal descriptor. We’ll evolve this to model/param handles later.
/// For now, gravity vector is the “model param” to drive Earth/Mars swaps.
#[derive(Clone, Debug)]
pub struct EpochDescriptor {
    pub gravity_g: [f32; 3], // world gravity in m/s^2, e.g., [0.0, -9.81, 0.0]
}

/// Deterministic 64-bit EpochID derived from the descriptor.
/// (BLAKE3 → first 8 bytes little-endian)
pub fn epoch_id(desc: &EpochDescriptor) -> u64 {
    let mut h = StepHasher::new();
    for f in desc.gravity_g {
        h.update_bytes(&f.to_le_bytes());
    }
    let bytes = h.finalize();
    u64::from_le_bytes([
        bytes[0], bytes[1], bytes[2], bytes[3],
        bytes[4], bytes[5], bytes[6], bytes[7],
    ])
}
#[derive(Copy, Clone, Debug, Default)]
pub struct EpochId(pub u64);

pub struct EpochManager {
    current: EpochId,
    // staged change becomes active at boundary
    staged: Option<PendingEpoch>,
}

pub struct PendingEpoch {
    pub next_models: ModelPackId,
    pub at_tick: u64, // boundary tick
}

impl EpochManager {
    pub fn current(&self) -> EpochId { self.current }
    pub fn stage(&mut self, next_models: ModelPackId, at_tick: u64) {
        self.staged = Some(PendingEpoch { next_models, at_tick });
    }
    pub fn maybe_promote(&mut self, tick: u64, models: &mut ModelRegistry) {
        if let Some(p) = &self.staged {
            if tick >= p.at_tick {
                models.activate(p.next_models);
                self.current.0 = self.current.0.wrapping_add(1);
                self.staged = None;
            }
        }
    }
}