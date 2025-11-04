#![deny(missing_docs)]
//! Minimal vehicle scaffold (ray wheels + simple tire forces).
//!
//! Usage:
//! - add a `VehicleInstance` via `World::add_vehicle(...)` (your world glue).
//! - call your `world.step_vehicles(dt)` before integration (see earlier patch).
//! - optional: emit `LedgerEvent::VehicleWheel` for telemetry.

pub mod vehicles;

pub use vehicles::{
    AxleInput,
    VehicleParams,
    VehicleInstance,
    WheelParams,
};
