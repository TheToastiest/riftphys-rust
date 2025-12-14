// riftphys-core/src/lib.rs

// Optional, but strongly recommended for a “core” crate:
#![forbid(unsafe_code)]

#[cfg(feature = "glam-fast-math")]
compile_error!("glam fast-math enabled. This breaks determinism. Turn it off.");

#[cfg(feature = "glam-core-simd")]
compile_error!("glam core-simd enabled. Do not ship deterministic physics with this on.");


pub mod scalar;
pub mod ids;
pub mod types;
pub mod hash;
pub mod time;
pub mod determinism;
pub mod schedule;
pub mod rng;
pub mod epoch;
pub mod hash_world;
pub mod models;
pub mod step_ctx;
pub mod det;

// --- Core context ---
pub use step_ctx::StepCtx;
pub use epoch::{EpochDescriptor, EpochId, EpochManager, epoch_id};

// --- Models / aero / prop ---
pub use models::{
    AeroHandle,
    PropHandle,
    AccelPackHandle,
    ModelPackId,
    ModelPack,
    ModelRegistry,
    AeroModel,
    PropulsionModel,
    AeroQuery,
    PropQuery,
};

// --- Scalar + math / types ---
pub use scalar::Scalar;
pub use types::{
    Vec3,
    Mat3,
    Quat,      // canonical quat type for the entire workspace
    QuatP,     // legacy alias, keep until you delete it everywhere
    Isometry,
    Velocity,
    vec3,
    iso,
    quat_identity,
};

// --- IDs ---
pub use ids::{BodyId, ColliderId, JointId};

// --- Hashing / determinism ---
pub use hash::{StepHasher, hash_vec3, hash_quat};
pub use time::StepStats;

pub use determinism::{
    DeterminismContract,
    Units,
    DeterministicHash,
    qf32,
    Qf32,
    Qv3,
    DEFAULT_QF32,
};

// --- Schedule ---
pub use schedule::{
    StepStage,
    schedule_digest,
    Slot,
    FIXED_ORDER,
};

// --- RNG + world hashing ---
pub use rng::XorShift64;
pub use hash_world::hash_world;
