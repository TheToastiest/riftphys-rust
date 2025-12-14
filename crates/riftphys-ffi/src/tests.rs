#![cfg(test)]

use super::*;

fn mk_world() -> *mut RPhysWorld {
    unsafe {
        let w = rphys_world_create(1024, 2048);
        assert!(!w.is_null());

        assert_eq!(rphys_world_set_epoch(w, 1), RPHYS_OK);
        assert_eq!(rphys_world_set_rng_seed(w, 0xBADC0FFEu64), RPHYS_OK);

        w
    }
}

#[test]
fn sphere_integrates_downward_velocity() {
    unsafe {
        let w = mk_world();

        let desc = RPhysSphereBodyDesc {
            pose: RPhysIsometry {
                pos: RPhysVec3 { x: 0.0, y: 10.0, z: 0.0 },
                rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            },
            vel: RPhysVelocity {
                lin: RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 }, // no gravity dependency
                ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            },
            radius: 0.5,
            mass: 1.0,
            body_type: RPhysBodyType::RPHYS_BODY_DYNAMIC,
            material: RPhysMaterial { mu_static: 0.5, mu_dynamic: 0.4, restitution: 0.1 },
            user_tag: 0,
        };

        let mut id: u32 = RPHYS_INVALID_BODY_ID;
        assert_eq!(rphys_add_sphere_body(w, &desc, &mut id), RPHYS_OK);
        assert_ne!(id, RPHYS_INVALID_BODY_ID);

        for _ in 0..60 {
            assert_eq!(rphys_world_step(w, 1.0 / 60.0), RPHYS_OK);
        }

        let mut pose_out = RPhysIsometry {
            pos: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        };

        assert_eq!(
            rphys_body_get_pose(w as *const RPhysWorld, id, &mut pose_out),
            RPHYS_OK
        );
        assert!(pose_out.pos.y < 10.0);

        rphys_world_destroy(w);
    }
}

#[test]
fn raycast_hits_ground() {
    unsafe {
        let w = mk_world();

        // Static ground box at y = -0.5 (top at y=0)
        let ground = RPhysBoxBodyDesc {
            pose: RPhysIsometry {
                pos: RPhysVec3 { x: 0.0, y: -0.5, z: 0.0 },
                rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            },
            vel: RPhysVelocity {
                lin: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
                ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            },
            hx: 50.0,
            hy: 0.5,
            hz: 50.0,
            mass: 0.0,
            body_type: RPhysBodyType::RPHYS_BODY_STATIC,
            material: RPhysMaterial { mu_static: 0.9, mu_dynamic: 0.7, restitution: 0.0 },
            user_tag: 0,
        };

        let mut ground_id: u32 = RPHYS_INVALID_BODY_ID;
        assert_eq!(rphys_add_box_body(w, &ground, &mut ground_id), RPHYS_OK);
        assert_ne!(ground_id, RPHYS_INVALID_BODY_ID);

        let q = RPhysRayQuery {
            origin: RPhysVec3 { x: 0.0, y: 5.0, z: 0.0 },
            dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
            max_distance: 20.0,
            ignore_body: 0,
        };

        let mut hit = RPhysRayHit {
            hit: RPHYS_FALSE,
            body_id: RPHYS_INVALID_BODY_ID,
            fraction: 0.0,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };

        assert_eq!(
            rphys_world_raycast(w as *const RPhysWorld, &q, &mut hit),
            RPHYS_OK
        );
        assert_eq!(hit.hit, RPHYS_TRUE);
        assert_eq!(hit.body_id, ground_id);

        rphys_world_destroy(w);
    }
}

#[test]
fn capsule_sweep_hits_wall() {
    unsafe {
        let w = mk_world();

        // Wall centered at x=3.0, thickness 0.25
        let wall = RPhysBoxBodyDesc {
            pose: RPhysIsometry {
                pos: RPhysVec3 { x: 3.0, y: 1.0, z: 0.0 },
                rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            },
            vel: RPhysVelocity {
                lin: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
                ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            },
            hx: 0.125,
            hy: 2.0,
            hz: 2.0,
            mass: 0.0,
            body_type: RPhysBodyType::RPHYS_BODY_STATIC,
            material: RPhysMaterial { mu_static: 0.9, mu_dynamic: 0.7, restitution: 0.0 },
            user_tag: 0,
        };

        let mut wall_id: u32 = RPHYS_INVALID_BODY_ID;
        assert_eq!(rphys_add_box_body(w, &wall, &mut wall_id), RPHYS_OK);
        assert_ne!(wall_id, RPHYS_INVALID_BODY_ID);

        let q = RPhysCapsuleSweepQuery {
            from: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
            to:   RPhysVec3 { x: 6.0, y: 1.0, z: 0.0 },
            radius: 0.5,
            half_height: 0.5,
            ignore_body: 0,
        };

        let mut hit = RPhysCapsuleSweepHit {
            hit: RPHYS_FALSE,
            body_id: RPHYS_INVALID_BODY_ID,
            fraction: 0.0,
            started_overlapping: RPHYS_FALSE,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };

        assert_eq!(
            rphys_world_capsule_sweep(w as *const RPhysWorld, &q, &mut hit),
            RPHYS_OK
        );
        assert_eq!(hit.hit, RPHYS_TRUE);
        assert_eq!(hit.body_id, wall_id);
        assert!((0.0..=1.0).contains(&hit.fraction));

        rphys_world_destroy(w);
    }
}

#[test]
fn determinism_hash_matches_between_two_worlds() {
    unsafe {
        let a = mk_world();
        let b = mk_world();

        let sphere = RPhysSphereBodyDesc {
            pose: RPhysIsometry {
                pos: RPhysVec3 { x: 2.0, y: 3.0, z: 4.0 },
                rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            },
            vel: RPhysVelocity {
                lin: RPhysVec3 { x: 1.0, y: -2.0, z: 0.5 },
                ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            },
            radius: 0.25,
            mass: 1.0,
            body_type: RPhysBodyType::RPHYS_BODY_DYNAMIC,
            material: RPhysMaterial { mu_static: 0.5, mu_dynamic: 0.4, restitution: 0.1 },
            user_tag: 0,
        };

        let mut ida: u32 = 0;
        let mut idb: u32 = 0;
        assert_eq!(rphys_add_sphere_body(a, &sphere, &mut ida), RPHYS_OK);
        assert_eq!(rphys_add_sphere_body(b, &sphere, &mut idb), RPHYS_OK);
        assert_ne!(ida, 0);
        assert_ne!(idb, 0);

        let mut ha = RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] };
        let mut hb = RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] };

        for _ in 0..120 {
            assert_eq!(rphys_world_step_and_hash(a, 1.0 / 120.0, &mut ha), RPHYS_OK);
            assert_eq!(rphys_world_step_and_hash(b, 1.0 / 120.0, &mut hb), RPHYS_OK);
            assert_eq!(ha.bytes, hb.bytes, "hash diverged between identical runs");
        }

        rphys_world_destroy(a);
        rphys_world_destroy(b);
    }
}
