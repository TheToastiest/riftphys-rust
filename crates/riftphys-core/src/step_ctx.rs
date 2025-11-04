use crate::epoch::EpochId;

/// Per-tick context passed into model evaluations and the schedule.
#[derive(Copy, Clone, Debug)]
pub struct StepCtx {
    pub dt: f32,
    pub tick: u64,
    pub epoch: EpochId,
}
