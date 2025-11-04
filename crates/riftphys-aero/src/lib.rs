// riftphys-aero/src/lib.rs
mod isa;
mod models;
mod combined;

pub use isa::ISA;
pub use models::{FlatPlateDrag, SimpleWing};
pub use combined::CombinedAero;
