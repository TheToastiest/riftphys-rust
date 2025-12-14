use crate::StepHasher;
use crate::determinism::{DEFAULT_QF32, qf32, canon_f32_bits};
use crate::models::{ModelPackId, ModelRegistry};

/// Minimal descriptor. Evolve this into model/param handles later.
/// For now gravity vector is the epoch-driving param.
#[derive(Clone, Debug)]
pub struct EpochDescriptor {
    pub gravity_g: [f32; 3], // m/s^2, e.g., [0.0, -9.81, 0.0]
}

/// Deterministic 64-bit EpochID derived from the descriptor.
/// (BLAKE3 → first 8 bytes little-endian)
pub fn epoch_id(desc: &EpochDescriptor) -> u64 {
    let mut h = StepHasher::new();
    for f in desc.gravity_g {
        // Quantize + canonicalize so epoch id doesn’t drift due to -0.0 or weird non-finites.
        let fq = qf32(f, DEFAULT_QF32);
        h.update_f32_bits(canon_f32_bits(fq));
    }
    let bytes = h.finalize();
    u64::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]])
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct EpochId(pub u64);

pub struct EpochManager {
    current: EpochId,
    staged: Option<PendingEpoch>,
}

pub struct PendingEpoch {
    pub next_models: ModelPackId,
    pub at_tick: u64, // boundary tick
}

impl Default for EpochManager {
    fn default() -> Self {
        Self { current: EpochId(0), staged: None }
    }
}

impl EpochManager {
    #[inline] pub fn current(&self) -> EpochId { self.current }

    #[inline]
    pub fn stage(&mut self, next_models: ModelPackId, at_tick: u64) {
        self.staged = Some(PendingEpoch { next_models, at_tick });
    }

    /// Promote staged epoch exactly at/after the boundary tick.
    /// Activation order is deterministic because this is a single-slot “pending” state.
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
