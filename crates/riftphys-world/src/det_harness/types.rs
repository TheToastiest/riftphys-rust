use riftphys_core::determinism::{Qf32, DeterministicHash};
use blake3::Hasher;
use riftphys_core::BodyId;
use riftphys_core::models::{AeroHandle, PropHandle};

/// What the harness expects from any sim world (your World implements this).
pub trait SimWorld {
    fn step_dt(&mut self, dt: f32) -> StepReport;
    fn epoch_id(&self) -> u64;
    fn step_hash(&self) -> [u8; 32];
    fn apply_inputs(&mut self, inputs: &Inputs);
}

/// Minimal per-step report used for provenance and checks.
#[derive(Clone, Copy, Default)]
pub struct StepReport {
    pub dt: f32,
    pub epoch: u64,
    pub hash: [u8; 32],
    pub pairs_tested: u32,
    pub contacts: u32,
    pub impulses_sum: f32,
    pub ccd_hits: u32,
    pub aero_sum: f32,
    pub prop_sum: f32,
}

/// On-disk inputs (tagged). Extend as needed; tags stable.
#[derive(Clone, Debug)]
pub enum InputEvent {
    SetThrottle { body: BodyId, throttle01: f32 },
    SetVelocity { body: BodyId, lin: [f32; 3], ang: [f32; 3] },
    SetBodyAccel {
        body: BodyId,
        aero: Option<AeroHandle>,
        prop: Option<PropHandle>,
        ref_area: f32,
        throttle01: f32,
    },
    GravityLayeredPlanet { surface_g: f32, radius: f32, center: [f32;3], min_r: f32 },
}
#[derive(Clone, Default)]
pub struct Inputs {
    pub tick_index: u32,
    pub events: Vec<InputEvent>,
}

/// Compact provenance written per frame.
#[derive(Clone, Copy, Default)]
pub struct ProvCompact {
    pub pairs: u32,
    pub contacts: u32,
    pub impulses_sum_q: f32,
    pub ccd_hits: u32,
    pub aero_sum_q: f32,
    pub prop_sum_q: f32,
}

impl DeterministicHash for ProvCompact {
    fn hash_to(&self, h: &mut Hasher, q: f32) {
        h.update(&self.pairs.to_le_bytes());
        h.update(&self.contacts.to_le_bytes());
        h.update(&Qf32(self.impulses_sum_q).quantized(q).to_le_bytes());
        h.update(&self.ccd_hits.to_le_bytes());
        h.update(&Qf32(self.aero_sum_q).quantized(q).to_le_bytes());
        h.update(&Qf32(self.prop_sum_q).quantized(q).to_le_bytes());
    }
}
