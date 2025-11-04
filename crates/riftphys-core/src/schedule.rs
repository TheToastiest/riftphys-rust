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
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Slot {
    PreIntegrate,       // e.g., user inputs, controllers
    Gravity,            // uniform gravity accumulation
    Accel,              // <-- Phase 11: aero + propulsion (after gravity)
    ContactsBroadphase,
    ContactsNarrowphase,
    Solver,
    Integrate,          // integrate position/orientation
    PostIntegrate,      // housekeeping
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