use std::time::{Duration, Instant};

use riftphys_world::*;
use riftphys_core::{vec3, iso, quat_identity, Velocity};
use riftphys_geom::{Shape, MassProps, Material};
use riftphys_gravity::GravitySpec;
use riftphys_viz::DebugSettings;

/// simple percentile
fn pct(mut xs: Vec<f32>, p: f32) -> f32 {
    if xs.is_empty() { return 0.0; }
    xs.sort_by(|a,b| a.partial_cmp(b).unwrap());
    let k = ((xs.len() as f32 - 1.0) * p).round() as usize;
    xs[k]
}

fn main() {
    // ---------- knobs ----------
    let n_spheres: usize = 250;      // scale this up/down
    let n_capsules: usize = 50;
    let n_walls: usize = 6;          // thin vertical slabs
    let ticks: usize = 400;          // total simulation ticks
    let dt: f32 = 1.0 / 120.0;       // bump to 240/360 to stress
    let p95_target_ms: f32 = 1.50;   // Phase-10 goal: p95 under this => ✅

    // ---------- world & debug ----------
    let mut w = WorldBuilder::new().with_capacity(n_spheres + n_capsules + n_walls + 16,
                                                  n_spheres + n_capsules + n_walls + 16).build();
    w.set_rng_seed(0xBEEF_CA_FE);
    w.set_epoch(42);
    w.set_debug(DebugSettings {
        print_every: 0, // set >0 if you want per-tick dumps too
        show_bodies: false, show_contacts: false, show_impulses: false, show_energy: false,
        max_lines: 32,
    });

    // gravity (uniform for stability)
    w.queue_gravity_swap(GravitySpec::Uniform { g: [0.0, -9.81, 0.0] });

    // ground
    let ground = w.add_body(
        iso(vec3(0.0, 0.0, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::infinite(),
        false,
    );
    let mut ground_mat = Material::default();
    ground_mat.mu_s = 0.8; ground_mat.mu_k = 0.7;
    w.add_collider(ground, Shape::Box { hx: 200.0, hy: 0.5, hz: 200.0 }, ground_mat);

    // thin vertical walls to force CCDs
    for wi in 0..n_walls {
        let x = -5.0 + wi as f32 * 2.0;
        let wall = w.add_body(iso(vec3(x, 2.0, 0.0), quat_identity()), Velocity::default(), MassProps::infinite(), false);
        let mut m = Material::default();
        m.mu_s = 0.6; m.mu_k = 0.5; // steel-ish
        w.add_collider(wall, Shape::Box { hx: 0.05, hy: 2.5, hz: 10.0 }, m);
    }

    // spheres with varying initial velocities (deterministic pattern)
    for i in 0..n_spheres {
        let row = i / 25;
        let col = i % 25;
        let x = -7.0 + (col as f32) * 0.6;
        let z = -7.0 + (row as f32) * 0.6;
        let y = 1.5 + ((i % 7) as f32) * 0.05;

        let b = w.add_body(
            iso(vec3(x, y, z), quat_identity()),
            Velocity { lin: vec3(6.0 + (i % 9) as f32 * 0.5, -0.5 * ((i % 5) as f32), 0.3 * ((i % 3) as f32 - 1.0)), ang: vec3(0.0, 0.0, 0.0) },
            MassProps::from_sphere(0.18, 1000.0),
            true,
        );
        let mut m = Material::default();
        // alternate materials to exercise pair-wise μ/restitution
        if i % 5 == 0 { m.mu_s = 0.2; m.mu_k = 0.1; m.restitution = 0.05; } // “icey”
        w.add_collider(b, Shape::Sphere { r: 0.18 }, m);
    }

    // capsules with gentle fall & lateral drift
    for j in 0..n_capsules {
        let x = 2.0 + (j % 10) as f32 * 0.5;
        let z = -3.0 + (j / 10) as f32 * 0.7;
        let c = w.add_body(
            iso(vec3(x, 2.0 + (j % 7) as f32 * 0.05, z), quat_identity()),
            Velocity { lin: vec3(-1.5 - (j % 7) as f32 * 0.2, -0.3, 0.2), ang: vec3(0.0, 0.0, 0.0) },
            MassProps::from_capsule(0.15, 0.35, 950.0),
            true,
        );
        let mut m = Material::default();
        m.mu_s = 1.0; m.mu_k = 0.8; m.restitution = 0.0; // “rubbery”
        w.add_collider(c, Shape::Capsule { r: 0.15, hh: 0.35 }, m);
    }

    // ---------- run & time ----------
    let mut step_ms: Vec<f32> = Vec::with_capacity(ticks);
    let mut total_contacts: u64 = 0;
    let mut errors: u64 = 0;

    for _ in 0..ticks {
        let t0 = Instant::now();
        let stats = w.step(dt);
        let dt_ms = t0.elapsed().as_secs_f64() as f32 * 1000.0;
        step_ms.push(dt_ms);
        total_contacts += stats.contacts as u64;

        // minimal sanity: sample a few IDs
        for id in [ground].iter() {
            let p = w.get_body_pose(*id).pos;
            if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) { errors += 1; }
        }
    }

    // ---------- summarize ----------
    let p50 = pct(step_ms.clone(), 0.50);
    let p95 = pct(step_ms.clone(), 0.95);
    let p99 = pct(step_ms.clone(), 0.99);
    let mean: f32 = step_ms.iter().sum::<f32>() / step_ms.len() as f32;

    println!("--- PERF SUMMARY ({} ticks) ---", ticks);
    println!("step latency (ms): mean={:.3}  p50={:.3}  p95={:.3}  p99={:.3}", mean, p50, p95, p99);
    println!("total contacts:    {}", total_contacts);
    println!("errors (finite):   {}", errors);
    if p95 <= p95_target_ms && errors == 0 {
        println!("Phase 10 ✅  (p95 ≤ {:.2} ms, no errors)", p95_target_ms);
    } else {
        println!("Phase 10 ⚠️  (p95 {:.3} ms, errors {})", p95, errors);
    }
}
