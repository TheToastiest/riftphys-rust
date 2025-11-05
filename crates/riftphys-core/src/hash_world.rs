// core/src/hash_world.rs
use blake3::Hasher;
use crate::determinism::{DeterministicHash};

pub fn hash_world<W: DeterministicHash>(world: &W, epoch: u64, tick: u64, q: f32) -> blake3::Hash {
    let mut h = Hasher::new();
    h.update(&epoch.to_le_bytes());
    h.update(&tick.to_le_bytes());
    world.hash_to(&mut h, q);
    h.finalize()
}
