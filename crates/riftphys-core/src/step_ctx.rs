use crate::epoch::EpochId;

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct StepCtx {
    pub dt: f32,
    pub tick: u64,
    pub epoch: EpochId,
}
