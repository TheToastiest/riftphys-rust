#[cfg(test)]
    use super::*;

    #[test]
    fn sphere_drop_basic() {
        unsafe {
            let w = rphys_world_create(1024, 2048);

            let mut desc = RPhysSphereBodyDesc {
                pose: RPhysIsometry {
                    pos: RPhysVec3 { x: 0.0, y: 10.0, z: 0.0 },
                    rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                },
                vel: RPhysVelocity {
                    lin: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
                    ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
                },
                radius: 0.5,
                mass: 1.0,
                body_type: RPhysBodyType::RPHYS_BODY_DYNAMIC,
                material: RPhysMaterial {
                    mu_static: 0.5,
                    mu_dynamic: 0.4,
                    restitution: 0.1,
                },
                user_tag: 0,
            };

            let id = rphys_add_sphere_body(w, &desc);

            for _ in 0..60 {
                rphys_world_step(w, 1.0 / 60.0);
            }

            let mut pose_out = RPhysIsometry {
                pos: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
                rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            };
            assert_eq!(rphys_body_get_pose(w, id, &mut pose_out), 1);
            assert!(pose_out.pos.y < 10.0);

            rphys_world_destroy(w);
        }
    }

