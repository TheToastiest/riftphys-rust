# RiftPhys — Deterministic Physics Core (Phase-20 Public Release)

**RiftPhys** is a data-oriented, deterministic physics stack powering the RiftForged project.  
This public release captures the Phase 0–20 milestone: determinism contracts, stepper, CCD, basic contact models, controllers (Guard/Balance), terrain sampling, aero/prop scaffolding, vehicles v1, epoch shadowing, rig I/O + hashing, and **Phase-20 contact polish (deterministic culling, quantization, warmstart cache)**.

> **Licensing:**  
> Public code in this branch is **dual-licensed** under **MIT OR Apache-2.0** (your choice).  
> Phases 21+ and the full game stack are **proprietary** and live in private branches/crates.

---

## Highlights (Phase-20)
- **Determinism Contract:** fixed `Δt`, `f32` (no fast-math), fixed iteration counts, stable sort keys.
- **Stepper & CCD:** semi-implicit Euler; sphere/capsule CCD; deterministic broadphase/narrowphase.
- **Contacts:** deterministic selection (≤4 per pair), normal/depth quantization, **warmstart cache**.
- **Epochs & Shadowing:** hot-swap gravity/model sets at tick boundaries with **epoch IDs**; epsilon compare.
- **Controllers:** Guard/Brace & Balance (deterministic support polygon).
- **Terrain:** height/normal sampling with deterministic tile cache.
- **Aero/Prop:** model registry; throttle & area refs (Phase-11 scaffolding).
- **Vehicles v1:** basic pre-step host hooks.
- **Rig I/O tools:** `rig_hash` (stable `.rig.json` + blake3), `rig_diff` (epsilon compare).
- **Benches:** perf (p50/p95/p99), soak tests, JSONL telemetry.

---

## Workspace & Crates
- `crates/riftphys-core` — math/types/IDs, hashing, step context.
- `crates/riftphys-world` — SoA bodies, collision, solver, controllers, epoch plumbing.
- `crates/riftphys-io` — `rig_hash`, `rig_diff`.
- `crates/riftphys-locomotion` — minimal gait primitives.
- `crates/riftphys-benchtests` — perf & determinism benches, epoch shadowing harness.
- (Other support crates as present: gravity, terrain, vehicles, viz, etc.)

---

## Build & Run
```bash
# build everything
cargo build --workspace

# run perf bench (adjust TICKS/DT as needed)
cargo run -p riftphys-benchtests --bin bench_perfs --release

# run the Phase 12–20 bench with epoch shadowing
cargo run -p riftphys-benchtests --bin riftphys-benchtests


Common env knobs

RPHYS_TICKS=600 — tick count
RPHYS_DT=0.0083333 — fixed Δt (120 Hz)
RPHYS_PRINT_EVERY=20 — debug cadence
RPHYS_PROMOTE_AFTER=180 — shadow promotion window (ticks)