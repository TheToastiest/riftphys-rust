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
    for s in stages {
        h.update_u8(*s as u8);
    }
    h.finalize()
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Slot {
    PreIntegrate       = 1,
    Gravity            = 2,
    Accel              = 3,
    ContactsBroadphase = 4,
    ContactsNarrowphase= 5,
    Solver             = 6,
    Integrate          = 7,
    PostIntegrate      = 8,
}

pub const FIXED_ORDER: &[Slot] = &[
    Slot::PreIntegrate,
    Slot::Gravity,
    Slot::Accel,
    Slot::ContactsBroadphase,
    Slot::ContactsNarrowphase,
    Slot::Solver,
    Slot::Integrate,
    Slot::PostIntegrate,
];
