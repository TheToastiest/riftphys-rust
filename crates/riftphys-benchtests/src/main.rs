        // Phase 12–15 bench with Balance controller + 120–200 Hz toggles and optional real-time pacer.
        // Uses EpochShadow to run active + shadow worlds in lockstep and promote on equivalence.
        use riftphys_riftnet::wire::{NetEnt, pack_snapshot};
        use riftphys_riftnet::sys;
        use std::ffi::CString;                                // for host string
        use std::ffi::c_void;                                 // for callback user ptr
        use std::sync::atomic::{AtomicBool, Ordering};
        use glam::{Quat};
        use riftphys_io::{ RigData, rig_physics::PhysicsRig };
        use riftphys_io::{rig_physics::humanoid_from_rig};
        use riftphys_io::{rig_physics::load_into_world};
        use std::fs;
        use anyhow::*;
        use std::result::Result::Ok;
        mod epoch_swap;
        mod bench_perf;
        mod epoch_shadow;
        // If the real EpochShadow is compiled (feature on), use it:
        #[cfg(feature = "shadow_epoch")]
        use epoch_shadow::{EpochShadow, Eps};

        // If feature is OFF, provide a local shim with the SAME surface.
        #[cfg(not(feature = "shadow_epoch"))]
        #[allow(dead_code)]
        pub struct Eps { pub pos: f32, pub vel: f32, pub quat_deg: f32, pub contacts: u32 }

        #[cfg(not(feature = "shadow_epoch"))]
        pub struct EpochShadow {
            pub active: riftphys_world::World,
            pub shadow: riftphys_world::World, // kept so existing code compiles unchanged
        }

        #[cfg(not(feature = "shadow_epoch"))]
        impl EpochShadow {
            pub fn new(active: riftphys_world::World,
                       shadow: riftphys_world::World,
                       _epoch_start: u32,
                       _promote_after: u32) -> Self {
                Self { active, shadow }
            }
            #[inline] pub fn step(&mut self, dt: f32) {
                let _ = self.active.step(dt);
                // shadow world intentionally not advanced in Earth-only mode
            }
            #[inline] pub fn step_eps(&mut self, dt: f32, _eps: Eps) -> bool {
                let _ = self.active.step(dt);
                false // “no promotion expected” semantics preserved
            }
        }

        use std::cell::RefCell;
        use std::sync::Arc;
        use std::time::{Duration, Instant};
        use glam::Vec3;
        mod bench_loco;
        use bench_loco::{loco_tick, loco_tick_with_heading};
        use riftphys_locomotion::{self as loco, TransitionPlan, TurnPlan, LocoState}; // add
        use riftphys_bridge_uriel as bridge;
        use riftphys_world::*;
        use riftphys_core::{vec3, iso, quat_identity, Velocity, BodyId};
        use riftphys_geom::{Shape, MassProps, Material};
        use riftphys_viz::DebugSettings;

        use riftphys_aero::{ISA, FlatPlateDrag, SimpleWing, CombinedAero};
        use riftphys_acceleration::{SimplePropulsion, ThrottleCurve};
        use riftphys_vehicles::{VehicleParams, WheelParams, AxleInput, VehicleInstance};
        use riftphys_terrain::HeightField;

        use riftphys_controllers::BalanceParams;
        use riftphys_gravity::{GravitySpec, PointMass, point_mass};

        // Rough μ (G*M) and centers in your existing “planet-below-origin” convention.
        // Feel free to tweak magnitudes if you want gentler forces.
        static SOLAR_BODIES: &[PointMass] = &[
            // Sun
            point_mass(1.327_124_4e20_f32, [0.0, -696_340_000.0, 0.0]),
            // Earth
            point_mass(3.986_004_4e14_f32, [0.0,   -6_371_000.0, 0.0]),
            // Moon (offset from Earth along +X)
            point_mass(4.904_869_5e12_f32, [384_400_000.0, -6_371_000.0, 0.0]),
            // Mars (optional; keep far below like the others)
            point_mass(4.282_837e13_f32,    [0.0,   -3_396_200.0, 0.0]),
        ];


        // ---------- tiny env helpers ----------
        fn env_u32(key: &str, default: u32) -> u32 {
            std::env::var(key).ok().and_then(|s| s.parse::<u32>().ok()).unwrap_or(default)
        }
        fn _env_f32(key: &str, default: f32) -> f32 {
            std::env::var(key).ok().and_then(|s| s.parse::<f32>().ok()).unwrap_or(default)
        }
        unsafe extern "C" fn on_server_event(_ev: *const riftphys_riftnet::sys::RiftEvent, _user: *mut c_void) {
            // you can log connect/disconnect here if you want
        }

        /* ====================== SCENE BUILDER ====================== */
        struct Scene {
            world: World,
            car: VehicleInstance,
            hf: HeightField,
            sphere_id: BodyId,
            capsule_id: BodyId,
            pelvis_id: BodyId,
            left_foot_id: BodyId,
            right_foot_id: BodyId,
        }
        fn load_or_build_rig(
            world: &mut World,
            physics_path: &str,
            rig_json_fallback: Option<&str>,
        ) -> anyhow::Result<(BodyId, BodyId, BodyId)> {
            // 1) Try prebuilt physics rig JSON
            let phys: PhysicsRig = if let Ok(txt) = fs::read_to_string(physics_path) {
                serde_json::from_str(&txt)?
            } else if let Some(rp) = rig_json_fallback {
                // 2) Fallback: build PhysicsRig from .rig.json on the fly
                let rt = fs::read_to_string(rp)?;
                let rig: RigData = serde_json::from_str(&rt)?;
                humanoid_from_rig(&rig)?
            } else {
                anyhow::bail!("No physics rig at {} and no .rig.json fallback provided", physics_path);
            };

            // 3) Spawn into the world
            let map = load_into_world(world, &phys)?;

            // 4) Resolve handles we’ll use for locomotion
            let pelvis     = map.body("pelvis");
            let left_foot  = map.body("l_foot");
            let right_foot = map.body("r_foot");

            Ok((pelvis, left_foot, right_foot))
        }

        fn build_scene(print_every: u32) -> Scene {
            use glam::{UVec2, Vec2};

            let mut w = WorldBuilder::new().with_capacity(256, 256).build();
            w.set_epoch(1);
            w.set_rng_seed(0xBADC0FFEE);
            w.set_debug(DebugSettings {
                print_every,
                show_bodies:   true,
                show_contacts: true,
                show_impulses: true,
                show_energy:   true,
                max_lines:     10,
                ..DebugSettings::default()
            });
            let mut mat_dyn = Material::default();
            mat_dyn.restitution = 0.0;  // no bounce
            mat_dyn.mu_s = 1.2;         // grippy
            mat_dyn.mu_k = 1.0;

            fn add_planet_ground(w: &mut World, center: [f32;3], radius: f32) {
                // Static body (inv_mass=0), no dynamics
                let body = w.add_body(
                    iso(vec3(center[0], center[1], center[2]), quat_identity()),
                    Velocity::default(),
                    MassProps::infinite(),
                    false
                );
                // Low restitution, grippy material so feet don't slide
                let mut m = Material::default();
                m.mu_s = 1.2; m.mu_k = 1.0; m.restitution = 0.0;
                w.add_collider(body, Shape::Sphere { r: radius }, m);
            }

            // ----- STATIC GEOMETRY -----
            // let ground = w.add_body(iso(vec3(0.0, 0.0, 0.0), quat_identity()), Velocity::default(), MassProps::infinite(), false);
            // w.add_collider(ground, Shape::Box { hx: 10.0, hy: 0.5, hz: 10.0 }, Material::default());
            //
            // let wall = w.add_body(iso(vec3(-3.5, 2.0, 0.0), quat_identity()), Velocity::default(), MassProps::infinite(), false);
            // w.add_collider(wall, Shape::Box { hx: 0.05, hy: 2.0, hz: 2.0 }, Material::default());

            // // ----- SIMPLE DYNAMICS -----
            // let b = w.add_body(iso(vec3(0.0, 2.0, 0.0), quat_identity()), Velocity::default(),
            //                    MassProps::from_box(vec3(0.25,0.25,0.25), 1000.0), true);
            // w.add_collider(b, Shape::Box { hx: 0.25, hy: 0.25, hz: 0.25 }, mat_dyn);

            // fast sphere (CCD)
            let s = w.add_body(
                iso(vec3(-5.0, 2.0, 0.0), quat_identity()),
                Velocity { lin: vec3(20.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
                MassProps::from_sphere(0.25, 1000.0), true
            );
            w.add_collider(s, Shape::Sphere { r: 0.25 }, mat_dyn);

            // capsule (aero/prop target)
            let pitch = 6.0_f32.to_radians();
            let c_rot  = Quat::from_rotation_z(pitch);            // tilt nose upward in +Y
            let c = w.add_body(
                iso(vec3(200.0, 60.0, -80.0), c_rot), // far from the walker & off to the side
                Velocity { lin: vec3(80.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
                MassProps::from_capsule(0.25, 0.5, 250.0),         // lighter for demo lift
                true
            );
            w.add_collider(c, Shape::Capsule { r: 0.25, hh: 0.5 }, mat_dyn);

            // ----- BALANCE DEMO (from rig) -----
            let (pelvis_id, left_foot_id, right_foot_id) =
                load_or_build_rig(
                    &mut w,
                    "assets/superhuman.physics.json",   // prebuilt if present
                    Some("assets/superhuman.rig.json"), // fallback to build on the fly
                ).expect("load/build physics rig");

            // Optional: ensure grippy soles if your rig loader gave defaults
            // (Only needed if you want explicit IDs; your current loader sets a grippy legacy material already)
            // use riftphys_materials::MaterialId;
            // TODO: If you add a "set_material" API on World/Collider, set feet to Rubber and ground to Grit here.

            w.add_balance_controller(
                pelvis_id, left_foot_id, right_foot_id,
                BalanceParams { k_accel: 25.0, max_accel: 40.0, com_height: 1.0, quantize: 1.0e-6 }
            );


            // ----- VEHICLE -----
            let car_body = w.add_body(iso(vec3(0.0, 1.0, -3.0), quat_identity()),
                                      Velocity::default(), MassProps::from_box(vec3(0.8, 0.2, 1.6), 1200.0), true);
            w.add_collider(car_body, Shape::Box { hx: 0.9, hy: 0.25, hz: 1.7 }, mat_dyn);
            let track = 0.9_f32; let wheel_r = 0.32_f32; let wheelbase_f = 1.3_f32; let wheelbase_r = -1.1_f32;
            let wheels = vec![
                WheelParams { local_pos: vec3(-track, -0.25, wheelbase_r), susp_dir: vec3(0.0, -1.0, 0.0), rest_len: 0.25, k_spring: 25_000.0, k_damp: 2_500.0, radius: wheel_r, mu_long: 1.2, mu_lat: 1.3 },
                WheelParams { local_pos: vec3( track, -0.25, wheelbase_r), susp_dir: vec3(0.0, -1.0, 0.0), rest_len: 0.25, k_spring: 25_000.0, k_damp: 2_500.0, radius: wheel_r, mu_long: 1.2, mu_lat: 1.3 },
                WheelParams { local_pos: vec3(-track, -0.25, wheelbase_f), susp_dir: vec3(0.0, -1.0, 0.0), rest_len: 0.25, k_spring: 25_000.0, k_damp: 2_500.0, radius: wheel_r, mu_long: 1.2, mu_lat: 1.3 },
                WheelParams { local_pos: vec3( track, -0.25, wheelbase_f), susp_dir: vec3(0.0, -1.0, 0.0), rest_len: 0.25, k_spring: 25_000.0, k_damp: 2_500.0, radius: wheel_r, mu_long: 1.2, mu_lat: 1.3 },
            ];
            let car_params = VehicleParams { body: car_body, wheels, drive_map: vec![0,1], steer_map: vec![2,3], mass_hint: 1200.0 };
            let car = VehicleInstance { p: car_params, input: AxleInput { throttle01: 0.4, steer_rad: 0.0 } };

            // ----- AERO/PROP -----
            let drag = Arc::new(FlatPlateDrag { cd: 1.0, area_m2: 0.20, isa: ISA::default() });
            let aero_capsule = Arc::new(CombinedAero::new(vec![
                Arc::new(FlatPlateDrag { cd: 1.0,  area_m2: 0.60,  isa: ISA::default() }),
                Arc::new(SimpleWing {
                    cl_per_rad: 6.0,              // ~0.105 per degree
                    stall_rad: 0.8,
                    area_m2: 1.0,                  // was 0.60; larger wing
                    lift_dir_body: Vec3::Y,        // *** up is +Y in your world ***
                    isa: ISA::default(),
                    cd0: 0.02,
                    k_induced: 0.05,
                }),
            ]));
            let prop_caps = Arc::new(SimplePropulsion {
                thrust_constant_n: 200.0,
                thrust_max_n:      1500.0,
                curve: ThrottleCurve::QuadEaseIn,
            });
            let drag_h = w.models_mut().register_aero(drag);
            let aero_capsule_h = w.models_mut().register_aero(aero_capsule);
            let prop_caps_h    = w.models_mut().register_prop(prop_caps);

            w.set_body_accel(s, Some(drag_h), None, 0.20, 0.0, None);
            // Use ~1.0 m² reference area to match the wing area; start at 0.5 throttle
            w.set_body_accel(
                c,
                Some(aero_capsule_h),
                Some(prop_caps_h),
                1.0,     // ref_area_m2
                0.5,     // throttle01
                None     // forward_dir_world (omit; we pitched the body via c_rot)
            );            let v0 = w.get_body_vel(c);
            w.set_body_vel(c, Velocity { lin: v0.lin + vec3(3.0, 0.0, 0.0), ang: v0.ang });

            // ----- GRAVITY -----
            // ----- GRAVITY (MultiBody: Sun + Earth + Moon [+ Mars]) -----
            w.queue_gravity_swap(GravitySpec::MultiBody {
                bodies: SOLAR_BODIES,
                min_r: 1_000.0, // clamp to avoid 1/r^2 spikes at the centers
            });
            // // Earth ground
            // add_planet_ground(&mut w, [0.0, -6_371_000.0, 0.0], 6_371_000.0);
            //
            // // (Optional) Mars ground too, if you want a second walkable body
            // add_planet_ground(&mut w, [0.0, -3_396_200.0, 0.0], 3_396_200.0)

            // make ground grippy & non-bouncy
            let mut mat_ground = Material::default();
            mat_ground.restitution = 0.0;
            mat_ground.mu_s = 1.2;
            mat_ground.mu_k = 1.0;

            let ground = w.add_body(
                iso(vec3(0.0, -0.25, 0.0), quat_identity()),   // hy=0.25 ⇒ top at y=0
                Velocity::default(), MassProps::infinite(), false);
            w.add_collider(ground, Shape::Box { hx:1000.0, hy:0.25, hz:1000.0 }, mat_ground);

            // ----- HEIGHTFIELD -----
            fn make_sine_hf(nx: u32, nz: u32, cell: f32, amp: f32, k: f32) -> HeightField {
                let dims = UVec2::new(nx, nz); let cellv = Vec2::new(cell, cell);
                let mut heights = Vec::with_capacity((nx * nz) as usize);
                for z in 0..nz { for x in 0..nx {
                    let wx = x as f32 * cell; let wz = z as f32 * cell;
                    heights.push(amp * (k*wx).sin() * (k*wz).cos());
                }}
                HeightField::from_heights(dims, cellv, heights)
            }
            let hf = make_sine_hf(256, 256, 0.5, 0.25, 0.15);
            w.set_heightfield(hf.clone(), 0.0);

            Scene {
                world: w, car, hf,
                sphere_id: s, capsule_id: c,
                pelvis_id, left_foot_id, right_foot_id,
            }}

        /* ====================== MAIN ====================== */
        fn main() {
            #[cfg(feature="shadow_epoch")]
            println!("shadow_epoch: ON (active+shadow)");
            #[cfg(not(feature="shadow_epoch"))]
            println!("shadow_epoch: OFF (Earth-only)");

            // --- determinism knobs / run config ---
            let hz = env_u32("RPHYS_HZ", 120).clamp(120, 200);
            let dt = std::env::var("RPHYS_DT").ok().and_then(|s| s.parse::<f32>().ok()).unwrap_or(1.0 / hz as f32);
            let ticks = env_u32("RPHYS_TICKS", 360);
            let print_every = env_u32("RPHYS_PRINT_EVERY", 20);
            let promote_after = env_u32("RPHYS_PROMOTE_AFTER", 180) as u32;

            // Optional wall-clock pacing (does not affect determinism)
            let realtime_hz: Option<f32> = std::env::var("RPHYS_REALTIME_HZ").ok().and_then(|s| s.parse::<f32>().ok());
            let mut pacer = realtime_hz.map(|hz| (Instant::now(), Duration::from_secs_f32(1.0 / hz)));
            // graceful exit on Ctrl-C
            let running = Arc::new(AtomicBool::new(true));
            {
                let r = running.clone();
                ctrlc::set_handler(move || { r.store(false, Ordering::SeqCst); })
                    .expect("ctrlc");
            }

            // ---------- RiftNet server init (broadcast snapshots to viewers) ----------
            let host_c = CString::new("0.0.0.0").unwrap(); // keep this alive while server exists!
            let server_cfg = riftphys_riftnet::sys::RiftServerConfig {
                host_address: host_c.as_ptr(),
                port: 49111,                                  // pick your port
                event_callback: Some(on_server_event),
                user_data: std::ptr::null_mut(),
            };
            let server = unsafe { riftphys_riftnet::sys::rift_server_create(&server_cfg) };
            assert!(!server.is_null(), "rift_server_create failed");
            let start_res = unsafe { riftphys_riftnet::sys::rift_server_start(server) };
            assert!(matches!(start_res, riftphys_riftnet::sys::RiftResult::RIFT_SUCCESS), "rift_server_start failed");

            // Build both scenes from identical initial conditions
            let Scene {
                world: active_w,
                car: mut car_a,
                hf: hf_a,
                sphere_id: s_a,
                capsule_id: c_a,
                pelvis_id, left_foot_id, right_foot_id,
            } = build_scene(print_every);
            let Scene { world: shadow_w, car: mut car_b, hf: hf_b, .. } = build_scene(print_every);

            // Create shadow orchestrator and make SHADOW different (e.g., Mars gravity)
            let mut sim = EpochShadow::new(active_w, shadow_w, /*epoch_start*/ 1, promote_after);
            // After: let mut sim = EpochShadow::new(active_w, shadow_w, 1, promote_after);
            sim.shadow.queue_gravity_swap(GravitySpec::MultiBody {
                bodies: SOLAR_BODIES,
                min_r: 1_000.0,
            });

            let sd_json = r#"
        { "dt": 0.0083333,
          "gravity": { "model":"LayeredPlanet", "params": {
              "surface_g": 3.721, "radius": 3396200.0, "center":[0,-3396200,0], "min_r": 1000.0 }},
          "accel": [ { "body_index": 4, "ref_area_m2": 0.6, "throttle01": 0.7, "aero": null, "prop": null } ]
        }"#;
            let sd: bridge::SimulationDescriptor = serde_json::from_str(sd_json).unwrap();
            bridge::validate_and_queue(&mut sim.shadow, &sd).unwrap();


            // ----- warmup (both)
            for _ in 0..ticks.min(120) {
                sim.step(1.0 / 60.0);
            }
            let left_clk0  = loco::FootClock::new(0.45, 0.45);
            let mut left_clk  = left_clk0;
            let mut right_clk = loco::FootClock::new(0.45, 0.45);
            // small phase offset so only one swings at a time
            right_clk.t = 0.45;

            let gait = loco::GaitSpec { stance_dur: 0.45, swing_dur: 0.45, step_len: 0.30, step_h: 0.10 };
            // --- Phase 21: heading + deterministic turn plan (doesn't change your current foot driver) ---
            let mut loco_state = LocoState::new(pelvis_id.0, left_foot_id.0, right_foot_id.0, gait);

            // keep LocoState's clocks aligned with your current clocks:
            loco_state.left_clk  = left_clk;
            loco_state.right_clk = right_clk;
            loco_state.enqueue(TransitionPlan::Turn(TurnPlan { yaw_total_rad: std::f32::consts::PI, steps: 4 }));

            // optional: demo a +90° turn over 6 steps, armed for the next double-stance
            if std::env::var("RPHYS_TURN_DEMO").ok().as_deref() == Some("1") {
                loco_state.enqueue(TransitionPlan::Turn(TurnPlan { yaw_total_rad: core::f32::consts::FRAC_PI_2, steps: 6 }));
            }
            // ----- main loop (lockstep)
            // ----- main loop (lockstep, continuous)
            let mut step: u64 = 0;
            loop {
                if !running.load(Ordering::SeqCst) { break; }

                // keep inputs identical
                car_b.input = car_a.input;

                // vehicle pre-step on ACTIVE
                {
                    let world_a = RefCell::new(&mut sim.active);
                    car_a.step_with_host(
                        |id| {
                            let p = world_a.borrow().get_body_pose(id);
                            (glam::Vec3::from(p.pos), p.rot)
                        },
                        |id| world_a.borrow().get_body_vel(id),
                        |id, v| world_a.borrow_mut().set_body_vel(id, v),
                        Some(&hf_a),
                        dt,
                    );
                }
                // vehicle pre-step on SHADOW
                {
                    let world_b = RefCell::new(&mut sim.shadow);
                    car_b.step_with_host(
                        |id| {
                            let p = world_b.borrow().get_body_pose(id);
                            (glam::Vec3::from(p.pos), p.rot)
                        },
                        |id| world_b.borrow().get_body_vel(id),
                        |id, v| world_b.borrow_mut().set_body_vel(id, v),
                        Some(&hf_b),
                        dt,
                    );
                }
                // Phase 21 planning: step heading & transitions deterministically.
                // This does NOT alter your current feet targets yet — it only advances the plan.
                let directive   = loco_state.step_and_plan(dt);
                let mut heading_dir = loco_state.heading_dir();
                if heading_dir.length_squared() < 1.0e-12 {
                    // default world-forward when planner hasn’t emitted a heading yet
                    heading_dir = glam::vec3(1.0, 0.0, 0.0);
                } else {
                    heading_dir = heading_dir.normalize();
                }


                // keep LocoState's clocks mirrored into the existing locals you already pass to loco_tick
                left_clk  = loco_state.left_clk;
                right_clk = loco_state.right_clk;
                // locomotion pre-step
                let mut l2 = left_clk;
                let mut r2 = right_clk;
                let dir = heading_dir;
                // Fallback if planner didn’t emit stride yet
                let mut left_len  = directive.left_step_len;
                let mut right_len = directive.right_step_len;
                if left_len == 0.0 && right_len == 0.0 {
                    left_len  = gait.step_len;
                    right_len = gait.step_len;
                }

                // --- use left_len/right_len below ---

                loco_tick_with_heading(
                    &mut sim.shadow,
                    pelvis_id.0, left_foot_id.0, right_foot_id.0,
                    &mut (l2, r2), &gait, heading_dir, left_len, right_len, dt
                );
                // --- DIAG A: print pelvis id + pose.y + PRE-step velocity ---
                {
                    let pid = pelvis_id;
                    let p   = sim.active.get_body_pose(pid).pos;
                    let v   = sim.active.get_body_vel(pid).lin;
                    if (sim.active.tick_index() % 40) == 0 {
                        println!("PRE  pelvis id={}  posY={:+.3}  v=({:+.3},{:+.3},{:+.3})",
                                 pid.0, p.y, v.x, v.y, v.z);
                    }
                }

                loco_tick_with_heading(
                    &mut sim.active,
                    pelvis_id.0, left_foot_id.0, right_foot_id.0,
                    &mut (left_clk, right_clk), &gait, heading_dir, left_len, right_len, dt
                );

                // loco_tick_with_heading(&mut sim.shadow, pelvis_id.0, left_foot_id.0, right_foot_id.0,
                //                        &mut (l2, r2), &gait, dir, directive.left_step_len, directive.right_step_len, dt);
                //
                // loco_tick_with_heading(&mut sim.active, pelvis_id.0, left_foot_id.0, right_foot_id.0,
                //                        &mut (left_clk, right_clk), &gait, dir, directive.left_step_len, directive.right_step_len, dt);

                // Keep clocks in sync both ways so hashes stay stable with or without Phase-21 enabled.
                loco_state.left_clk  = left_clk;
                loco_state.right_clk = right_clk;
                if (sim.active.tick_index() % 60) == 0 {
                    println!("stride L={:.3} R={:.3} (planner L={:.3} R={:.3})",
                             left_len, right_len, directive.left_step_len, directive.right_step_len);
                }

                // If you later extend bench_loco to consume heading/step lengths, you'll have them ready:
                let _unused_turn_rad  = directive.yaw_delta_rad;
                let _unused_dir_ws    = heading_dir;
                let _unused_left_len  = directive.left_step_len;
                let _unused_right_len = directive.right_step_len;
                let _unused_stop      = directive.stop_after;
                let _ = (directive.yaw_delta_rad,
                         directive.left_step_len,
                         directive.right_step_len,
                         directive.stop_after,
                         heading_dir);

                // sim step
                let eps = Eps { pos: 1.0e-3, vel: 1.0e-3, quat_deg: 0.05, contacts: 2 };
                // --- grounded hysteresis for the capsule's aero/prop ---
                struct GroundState { grounded: bool, on_ticks: u32, off_ticks: u32 }
                static mut CAPS_GROUND: Option<GroundState> = None;

                // thresholds (pick once, keep fixed for determinism)
                const NF_ON:  f32 = 20.0;   // become grounded if >= this for a few ticks
                const NF_OFF: f32 = 5.0;    // become airborne if <= this for a few ticks
                const ON_DEBOUNCE_T:  u32 = 3;   // require 3 consecutive ticks
                const OFF_DEBOUNCE_T: u32 = 6;   // require 6 consecutive ticks

                // read normal force
                let nf = sim.active.normal_force(c_a, dt);

                // update hysteresis
                unsafe {
                    let st = CAPS_GROUND.get_or_insert(GroundState { grounded: false, on_ticks: 0, off_ticks: 0 });
                    if st.grounded {
                        if nf <= NF_OFF { st.off_ticks += 1; } else { st.off_ticks = 0; }
                        if st.off_ticks >= OFF_DEBOUNCE_T { st.grounded = false; st.off_ticks = 0; }
                    } else {
                        if nf >= NF_ON { st.on_ticks += 1; } else { st.on_ticks = 0; }
                        if st.on_ticks >= ON_DEBOUNCE_T { st.grounded = true; st.on_ticks = 0; }
                    }

                    // apply throttle gating deterministically
                    if st.grounded {
                        sim.active.set_body_throttle(c_a, 0.0);
                    } else {
                        sim.active.set_body_throttle(c_a, 0.5);
                    }

                    // optional: print every 60 ticks to verify
                    if (sim.active.tick_index() % 60) == 0 {
                        println!("capsule nf={:.1}  grounded={}  thr={:.2}",
                                 nf, st.grounded, if st.grounded { 0.0 } else { 0.5 });
                    }
                }

                let _ = sim.step_eps(dt, eps); // continuous world; no promotion expected if epochs differ
                // --- DIAG B: see what survives the step ---
                {
                    let pid = pelvis_id;
                    let p   = sim.active.get_body_pose(pid).pos;
                    let v   = sim.active.get_body_vel(pid).lin;
                    if (sim.active.tick_index() % 40) == 0 {
                        println!("POST pelvis id={}  posY={:+.3}  v=({:+.3},{:+.3},{:+.3})",
                                 pid.0, p.y, v.x, v.y, v.z);
                    }
                }

                // ---------- pack & broadcast ACTIVE snapshot ----------
                let tick_u64 = step;
                let epoch_u64 = sim.active.epoch_id;
                fn encode_shape(world: &World, bid: BodyId) -> (f32,f32,f32,u32) {
                    use riftphys_geom::Shape;
                    match world.primary_shape(bid) {
                        Some(Shape::Sphere { r }) => (r, r, r, 1),
                        Some(Shape::Box { hx, hy, hz }) => (hx, hy, hz, 2),
                        Some(Shape::Capsule { r, hh }) => {
                            // viewer expects sy = (hh + r)/2 like your renderer
                            (r, (hh + r) / 2.0, r, 3)
                        }
                        _ => (0.25, 0.25, 0.25, 1), // fallback sphere
                    }
                }
                #[inline]
                fn apply_pelvis_yaw(w: &mut World, pelvis: BodyId, dy: f32) {
                    if dy == 0.0 { return; }
                    let mut pose = w.get_body_pose(pelvis);
                    let dq = glam::Quat::from_rotation_y(dy);
                    pose.rot = (dq * pose.rot).normalize();
                    w.set_body_pose(pelvis, pose);
                }

                // reuse buffers to avoid alloc churn
                let body_count = sim.active.num_bodies() as usize;
                let mut ents = Vec::with_capacity(body_count);
                for id in 0..(body_count as u32) {
                    let bid = riftphys_core::BodyId(id);
                    let pose = sim.active.get_body_pose(bid);
                    let (sx, sy, sz, kind) = encode_shape(&sim.active, bid);
                    ents.push(NetEnt {
                        id,
                        px: pose.pos.x, py: pose.pos.y, pz: pose.pos.z,
                        qx: pose.rot.x, qy: pose.rot.y, qz: pose.rot.z, qw: pose.rot.w,
                        sx, sy, sz, kind,
                    });
                }

                for payload in pack_snapshot(tick_u64, epoch_u64, &ents) {
                    unsafe {
                        let _ = sys::rift_server_broadcast(server, payload.as_ptr(), payload.len() as sys::size_t);
                    }
                }

                // light cadence log
                let wt = sim.active.tick_index();
                if print_every != 0 && (wt as u32) % print_every == 0 {
                    let pv = sim.active.get_body_vel(pelvis_id).lin;
                    println!("tick {}  pelvis |v|={:.3}  v=({:+.3},{:+.3},{:+.3})",
                             wt, pv.length(), pv.x, pv.y, pv.z);
                }
                // real-time pacer (doesn't affect determinism)
                if let Some((ref mut last, frame)) = pacer {
                    let now = Instant::now();
                    let next = *last + frame;
                    if now < next { std::thread::sleep(next - now); }
                    *last = Instant::now();
                }

                step = step.wrapping_add(1);
            }

            // quick blade sweep test (Phase 9) on ACTIVE
            let dt_small = dt;
            let tip_p0 = sim.active.get_body_pose(s_a).pos;
            let tip_v  = sim.active.get_body_vel(s_a).lin;
            let mid_p0 = tip_p0 + riftphys_core::types::vec3(0.0, 0.5, 0.0);
            let mid_v  = tip_v;
            if let Some((box_idx, toi, n)) =
                sim.active.sweep_blade_against_boxes(tip_p0, tip_v, 0.1, mid_p0, mid_v, 0.1, dt_small)
            {
                println!("blade HIT: box_index={} at t={:.3}  n=({:+.2},{:+.2},{:+.2})", box_idx, toi, n.x, n.y, n.z);
            }

            println!("final hash = {:02x?}", sim.active.step_hash());
        }
