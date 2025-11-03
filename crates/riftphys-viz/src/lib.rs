use riftphys_core::{StepStage, schedule_digest};

#[derive(Default)]
pub struct ScheduleRecorder { stages: Vec<StepStage> }

impl ScheduleRecorder {
    pub fn new() -> Self { Self { stages: Vec::new() } }
    pub fn push(&mut self, s: StepStage) { self.stages.push(s); }
    pub fn clear(&mut self) { self.stages.clear(); }
    pub fn digest(&self) -> [u8; 32] { schedule_digest(&self.stages) }
}
