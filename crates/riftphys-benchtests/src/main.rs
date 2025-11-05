// Phase 12–15 bench with Balance controller + 120–200 Hz toggles and optional real-time pacer.
// Uses EpochShadow to run active + shadow worlds in lockstep and promote on equivalence.

mod epoch_swap;
mod bench_perf;
mod epoch_shadow;
use epoch_shadow::{EpochShadow, Eps};
use std::cell::RefCell;
use std::sync::Arc;
use std::time::{Duration, Instant};
use glam::Vec3;
mod bench_loco;
use bench_loco::loco_tick;
use riftphys_locomotion as loco;
use riftphys_bridge_uriel as bridge;
use riftphys_world::*;
use riftphys_core::{vec3, iso, quat_identity, Velocity, BodyId};
use riftphys_geom::{Shape, MassProps, Material};
use riftphys_gravity::GravitySpec;
use riftphys_viz::DebugSettings;

use riftphys_aero::{ISA, FlatPlateDrag, SimpleWing, CombinedAero};
use riftphys_acceleration::{SimplePropulsion, ThrottleCurve};
use riftphys_vehicles::{VehicleParams, WheelParams, AxleInput, VehicleInstance};
use riftphys_terrain::HeightField;

use riftphys_controllers::BalanceParams;

// ---------- tiny env helpers ----------
fn env_u32(key: &str, default: u32) -> u32 {
    std::env::var(key).ok().and_then(|s| s.parse::<u32>().ok()).unwrap_or(default)
}
fn _env_f32(key: &str, default: f32) -> f32 {
    std::env::var(key).ok().and_then(|s| s.parse::<f32>().ok()).unwrap_or(default)
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

    // ----- STATIC GEOMETRY -----
    let ground = w.add_body(iso(vec3(0.0, 0.0, 0.0), quat_identity()), Velocity::default(), MassProps::infinite(), false);
    w.add_collider(ground, Shape::Box { hx: 10.0, hy: 0.5, hz: 10.0 }, Material::default());

    let wall = w.add_body(iso(vec3(-3.5, 2.0, 0.0), quat_identity()), Velocity::default(), MassProps::infinite(), false);
    w.add_collider(wall, Shape::Box { hx: 0.05, hy: 2.0, hz: 2.0 }, Material::default());

    // ----- SIMPLE DYNAMICS -----
    let b = w.add_body(iso(vec3(0.0, 2.0, 0.0), quat_identity()), Velocity::default(),
                       MassProps::from_box(vec3(0.25,0.25,0.25), 1000.0), true);
    w.add_collider(b, Shape::Box { hx: 0.25, hy: 0.25, hz: 0.25 }, Material::default());

    // fast sphere (CCD)
    let s = w.add_body(
        iso(vec3(-5.0, 2.0, 0.0), quat_identity()),
        Velocity { lin: vec3(20.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
        MassProps::from_sphere(0.25, 1000.0), true
    );
    w.add_collider(s, Shape::Sphere { r: 0.25 }, Material::default());

    // capsule (aero/prop target)
    let c = w.add_body(
        iso(vec3(2.0, 1.5, 0.0), quat_identity()),
        Velocity { lin: vec3(250.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
        MassProps::from_capsule(0.25, 0.5, 750.0), true
    );
    w.add_collider(c, Shape::Capsule { r: 0.25, hh: 0.5 }, Material::default());

    // ----- BALANCE DEMO -----
    let pelvis_id = w.add_body(
        iso(vec3(0.0, 1.20, 0.50), quat_identity()),
        Velocity::default(),
        MassProps::from_capsule(0.15, 0.35, 80.0),
        true,
    );

    w.add_collider(pelvis_id, Shape::Capsule { r: 0.15, hh: 0.35 }, Material::default());
    let left_foot_id  = w.add_body(iso(vec3(-0.37, 0.55, 0.50), quat_identity()),
                                   Velocity::default(), MassProps::from_box(vec3(0.12, 0.05, 0.25), 20.0), true);
    let right_foot_id = w.add_body(iso(vec3( 0.37, 0.55, 0.50), quat_identity()),
                                   Velocity::default(), MassProps::from_box(vec3(0.12, 0.05, 0.25), 20.0), true);
    w.add_collider(left_foot_id,  Shape::Box { hx: 0.12, hy: 0.051, hz: 0.25 }, Material::default());
    w.add_collider(right_foot_id, Shape::Box { hx: 0.12, hy: 0.051, hz: 0.25 }, Material::default());
    w.add_balance_controller(
        pelvis_id, left_foot_id, right_foot_id,
        BalanceParams { k_accel: 25.0, max_accel: 40.0, com_height: 1.0, quantize: 1.0e-6 }
    );

    // ----- VEHICLE -----
    let car_body = w.add_body(iso(vec3(0.0, 1.0, -3.0), quat_identity()),
                              Velocity::default(), MassProps::from_box(vec3(0.8, 0.2, 1.6), 1200.0), true);
    w.add_collider(car_body, Shape::Box { hx: 0.9, hy: 0.25, hz: 1.7 }, Material::default());
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
        Arc::new(FlatPlateDrag { cd: 1.0, area_m2: 0.60, isa: ISA::default() }),
        Arc::new(SimpleWing { cl_per_rad: 6.0, stall_rad: 0.8, area_m2: 0.60,
            lift_dir_body: Vec3::Z, isa: ISA::default(), cd0: 0.05, k_induced: 0.05 }),
    ]));
    let prop_caps = Arc::new(SimplePropulsion { thrust_constant_n: 0.0, thrust_max_n: 3000.0, curve: ThrottleCurve::QuadEaseIn });

    let drag_h = w.models_mut().register_aero(drag);
    let aero_capsule_h = w.models_mut().register_aero(aero_capsule);
    let prop_caps_h    = w.models_mut().register_prop(prop_caps);

    w.set_body_accel(s, Some(drag_h), None, 0.20, 0.0, None);
    w.set_body_accel(c, Some(aero_capsule_h), Some(prop_caps_h), 0.60, 0.55, None);
    let v0 = w.get_body_vel(c);
    w.set_body_vel(c, Velocity { lin: v0.lin + vec3(3.0, 0.0, 0.0), ang: v0.ang });

    // ----- GRAVITY -----
    w.queue_gravity_swap(GravitySpec::LayeredPlanet {
        surface_g: 9.81, radius: 6_371_000.0, center: [0.0, -6_371_000.0, 0.0], min_r: 1_000.0
    });

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
    // --- determinism knobs / run config ---
    let hz = env_u32("RPHYS_HZ", 120).clamp(120, 200);
    let dt = std::env::var("RPHYS_DT").ok().and_then(|s| s.parse::<f32>().ok()).unwrap_or(1.0 / hz as f32);
    let ticks = env_u32("RPHYS_TICKS", 360);
    let print_every = env_u32("RPHYS_PRINT_EVERY", 20);
    let promote_after = env_u32("RPHYS_PROMOTE_AFTER", 180) as u32;

    // Optional wall-clock pacing (does not affect determinism)
    let realtime_hz: Option<f32> = std::env::var("RPHYS_REALTIME_HZ").ok().and_then(|s| s.parse::<f32>().ok());
    let mut pacer = realtime_hz.map(|hz| (Instant::now(), Duration::from_secs_f32(1.0 / hz)));

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

    // ----- main loop (lockstep)
    for step in 0..ticks {
        // keep inputs identical
        car_b.input = car_a.input;

        // vehicle pre-step on ACTIVE (use RefCell to avoid E0500)
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

        // sim tick (both worlds)
        // ===== Locomotion pre-step (both worlds, identical phases)
        // Clone clocks BEFORE stepping so shadow and active use the same phase this tick.
        let mut l2 = left_clk;
        let mut r2 = right_clk;

        // SHADOW first (cloned clocks)
        loco_tick(&mut sim.shadow, pelvis_id.0, left_foot_id.0, right_foot_id.0, &mut (l2, r2), &gait, dt);

        // ACTIVE with the real clocks (advance them once per tick)
        loco_tick(&mut sim.active, pelvis_id.0, left_foot_id.0, right_foot_id.0, &mut (left_clk, right_clk), &gait, dt);

        let eps = Eps { pos: 1.0e-3, vel: 1.0e-3, quat_deg: 0.05, contacts: 2 };
        if let Some(new_epoch) = sim.step_eps(dt, eps) {
            println!("PROMOTED → epoch=0x{:016x} (after {} equal ticks)", new_epoch, promote_after);
        }
        // console cadence (just print speed; no StepStats needed)
        if print_every != 0 && (step % print_every == 0) {
            let v = sim.active.get_body_vel(c_a).lin;
            println!("tick {}  |v|={:.2}  vz={:.2}", step, v.length(), v.z);
        }

        // optional pacer to visualize without changing determinism
        if let Some((ref mut last, frame)) = pacer {
            let now = Instant::now();
            let next = *last + frame;
            if now < next { std::thread::sleep(next - now); }
            *last = Instant::now();
        }
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
