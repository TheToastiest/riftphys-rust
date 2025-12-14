// crates/riftphys-world/src/lib.rs

//! riftphys-world: deterministic simulation world + queries + harness glue.

// Force Rust to use the folder module, even if `src/world.rs` still exists.
// This removes E0761 without requiring you to delete anything immediately.
#[path = "world/mod.rs"]
pub mod world;

// Minimal harness surface so `world/harness.rs` can implement
// `crate::det_harness::types::SimWorld` and your old imports keep working.
pub mod det_harness {
    pub mod types {
        use riftphys_core::BodyId;
        use riftphys_core::models::{AeroHandle, PropHandle};

        #[derive(Clone, Debug, Default)]
        pub struct Inputs {
            pub events: Vec<InputEvent>,
        }

        #[derive(Clone, Copy, Debug)]
        pub enum InputEvent {
            SetThrottle {
                body: BodyId,
                throttle01: f32,
            },
            SetVelocity {
                body: BodyId,
                lin: [f32; 3],
                ang: [f32; 3],
            },
            SetBodyAccel {
                body: BodyId,
                aero: Option<AeroHandle>,
                prop: Option<PropHandle>,
                ref_area: f32,
                throttle01: f32,
            },
            GravityLayeredPlanet {
                surface_g: f32,
                radius: f32,
                center: [f32; 3],
                min_r: f32,
            },
        }

        #[derive(Clone, Copy, Debug, Default)]
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

        pub trait SimWorld {
            fn step_dt(&mut self, dt: f32) -> StepReport;
            fn epoch_id(&self) -> u64;
            fn step_hash(&self) -> [u8; 32];
            fn apply_inputs(&mut self, inputs: &Inputs);
        }
    }

    // Re-export so old call sites keep working:
    // `use crate::det_harness::{Inputs, InputEvent, StepReport};`
    pub use types::{InputEvent, Inputs, SimWorld, StepReport};
}

// ---- ergonomic re-exports from world ----
pub use world::{Collider, RayHit, SweepHit, World, WorldBuilder};

pub mod prelude {
    pub use crate::{Collider, RayHit, SweepHit, World, WorldBuilder};
}
