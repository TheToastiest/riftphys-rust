#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RPhysWorld RPhysWorld;

typedef struct RPhysVec3 {
    float x, y, z;
} RPhysVec3;

typedef struct RPhysQuat {
    float x, y, z, w;
} RPhysQuat;

typedef struct RPhysIsometry {
    RPhysVec3 pos;
    RPhysQuat rot;
} RPhysIsometry;

typedef struct RPhysVelocity {
    RPhysVec3 lin;
    RPhysVec3 ang;
} RPhysVelocity;

typedef struct RPhysMaterial {
    float mu_static;
    float mu_dynamic;
    float restitution;
} RPhysMaterial;

typedef enum RPhysBodyType {
    RPHYS_BODY_STATIC  = 0,
    RPHYS_BODY_DYNAMIC = 1,
} RPhysBodyType;

typedef struct RPhysCapsuleBodyDesc {
    RPhysIsometry pose;
    RPhysVelocity vel;
    float radius;
    float half_height;
    float mass;
    RPhysBodyType body_type;
    RPhysMaterial material;
    uint32_t user_tag;
} RPhysCapsuleBodyDesc;

typedef struct RPhysBoxBodyDesc {
    RPhysIsometry pose;
    RPhysVelocity vel;
    float hx, hy, hz;
    float mass;
    RPhysBodyType body_type;
    RPhysMaterial material;
    uint32_t user_tag;
} RPhysBoxBodyDesc;

typedef struct RPhysSphereBodyDesc {
    RPhysIsometry pose;
    RPhysVelocity vel;
    float radius;
    float mass;
    RPhysBodyType body_type;
    RPhysMaterial material;
    uint32_t user_tag;
} RPhysSphereBodyDesc;

// lifecycle
RPhysWorld* rphys_world_create(uint32_t max_bodies, uint32_t max_colliders);
void        rphys_world_destroy(RPhysWorld* world);
void        rphys_world_set_epoch(RPhysWorld* world, uint64_t epoch);
void        rphys_world_set_rng_seed(RPhysWorld* world, uint64_t seed);

// bodies
uint32_t rphys_add_capsule_body(RPhysWorld* world, const RPhysCapsuleBodyDesc* desc);
uint32_t rphys_add_box_body    (RPhysWorld* world, const RPhysBoxBodyDesc* desc);
uint32_t rphys_add_sphere_body (RPhysWorld* world, const RPhysSphereBodyDesc* desc);
int      rphys_body_remove     (RPhysWorld* world, uint32_t body_id);

// state
int rphys_body_get_pose    (RPhysWorld* world, uint32_t body_id, RPhysIsometry* out_pose);
int rphys_body_set_pose    (RPhysWorld* world, uint32_t body_id, const RPhysIsometry* pose);
int rphys_body_get_velocity(RPhysWorld* world, uint32_t body_id, RPhysVelocity* out_vel);
int rphys_body_set_velocity(RPhysWorld* world, uint32_t body_id, const RPhysVelocity* vel);
int rphys_body_apply_impulse(RPhysWorld* world, uint32_t body_id, RPhysVec3 impulse);

// stepping
void rphys_world_step(RPhysWorld* world, float dt);

// raycast
typedef struct RPhysRayQuery {
    RPhysVec3 origin;
    RPhysVec3 dir;
    float max_distance;
    uint32_t ignore_body;
} RPhysRayQuery;

typedef struct RPhysRayHit {
    int hit;
    uint32_t body_id;
    float fraction;
    RPhysVec3 point;
    RPhysVec3 normal;
} RPhysRayHit;

void rphys_world_raycast(RPhysWorld* world,
                         const RPhysRayQuery* query,
                         RPhysRayHit* out_hit);

// capsule sweep
typedef struct RPhysCapsuleSweepQuery {
    RPhysVec3 from;
    RPhysVec3 to;
    float radius;
    float half_height;
    uint32_t ignore_body;
} RPhysCapsuleSweepQuery;

typedef struct RPhysCapsuleSweepHit {
    int hit;
    uint32_t body_id;
    float fraction;
    int started_overlapping;
    RPhysVec3 point;
    RPhysVec3 normal;
} RPhysCapsuleSweepHit;

void rphys_world_capsule_sweep(RPhysWorld* world,
                               const RPhysCapsuleSweepQuery* query,
                               RPhysCapsuleSweepHit* out_hit);

#ifdef __cplusplus
}
#endif
