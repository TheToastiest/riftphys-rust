use crate::StepHasher;

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum StepStage {
    Integrate = 1,
    UpdateAabbsPre = 2,
    BroadphaseSap = 3,
    Narrowphase = 4,
    Solve = 5,
    UpdateAabbsPost = 6,
}

pub fn schedule_digest(stages: &[StepStage]) -> [u8; 32] {
    let mut h = StepHasher::new();
    for s in stages { h.update_bytes(&[*s as u8]); }
    h.finalize()
}
