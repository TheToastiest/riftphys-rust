#include "riftphys_c.h"

// 1. Initialize World
RPhysWorld* world = rphys_world_create(1024, 1024);
rphys_world_set_epoch(world, 1);
rphys_world_set_rng_seed(world, 0xDEADBEEF);

// 2. Add Terrain (Ensure memory outlives the physics step!)
// ... [Heightfield initialization] ...

// 3. Step Simulation
RPhysStepParams params{};
params.size_bytes = sizeof(RPhysStepParams);
params.dt = 1.0f / 60.0f;
params.substeps = 4;

rphys_world_step_ex(world, &params);

// 4. Verify Hash
RPhysHash32 hash;
rphys_world_get_step_hash(world, &hash);