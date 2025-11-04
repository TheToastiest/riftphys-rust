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
// <-- add
pub mod step_ctx;

pub use step_ctx::StepCtx;
pub use epoch::{EpochId, EpochManager};
pub use models::{
    AeroHandle, PropHandle, AccelPackHandle, ModelPackId, ModelPack, ModelRegistry,
    AeroModel, PropulsionModel, AeroQuery, PropQuery,
};

pub use scalar::Scalar;
pub use ids::{BodyId, ColliderId, JointId};
pub use types::{Vec3, Mat3, Isometry, Velocity, vec3, iso, quat_identity};
pub use hash::{StepHasher, hash_vec3, hash_quat};
pub use time::StepStats;
pub use determinism::{DeterminismContract, Units};
pub use schedule::{StepStage, schedule_digest};
pub use rng::XorShift64;
pub use epoch::{EpochDescriptor, epoch_id};   // <-- add
pub use glam::Quat;
