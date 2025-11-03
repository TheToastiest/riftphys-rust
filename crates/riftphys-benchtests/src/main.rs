// Remove this if you don't have src/epoch_swap.rs
mod epoch_swap;
mod bench_perf;

use riftphys_world::*;
use riftphys_core::{vec3, iso, quat_identity, Velocity};
use riftphys_geom::{Shape, MassProps, Material};
use riftphys_gravity::GravitySpec;
use riftphys_viz::DebugSettings;
use riftphys_melee;



fn main() {
    let mut w = WorldBuilder::new().with_capacity(256, 256).build();
    w.set_epoch(1);
    w.set_rng_seed(0xBADC0FFEE);

    // Turn on readable, deterministic telemetry every 10 ticks:
    w.set_debug(DebugSettings {
        print_every: 1,
        show_bodies: true,
        show_contacts: true,
        show_impulses: true,
        show_energy: true,
        max_lines: 50,
    });

    // Ground (static AA box)
    let ground = w.add_body(
        iso(vec3(0.0, 0.0, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::infinite(),
        false,
    );
    w.add_collider(ground, Shape::Box { hx: 10.0, hy: 0.5, hz: 10.0 }, Material::default());
    // Add a static "wall" box that the fast sphere will hit mid-air
    let wall = w.add_body(
        iso(vec3(-3.5, 2.0, 0.0), quat_identity()),  // centered near sphere's flight path
        Velocity::default(),
        MassProps::infinite(),
        false,
    );
    w.add_collider(
        wall,
        Shape::Box { hx: 0.05, hy: 2.0, hz: 2.0 },   // thin vertical slab (10cm thick)
        Material::default(),
    );

    // Falling box
    let b = w.add_body(
        iso(vec3(0.0, 2.0, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::from_box(vec3(0.25,0.25,0.25), 1000.0),
        true,
    );
    w.add_collider(b, Shape::Box { hx: 0.25, hy: 0.25, hz: 0.25 }, Material::default());

    // Fast sphere (CCD exercise)
    let smass = MassProps::from_sphere(0.25, 1000.0);
    let s = w.add_body(
        iso(vec3(-5.0, 2.0, 0.0), quat_identity()),
        Velocity { lin: vec3(20.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
        smass,
        true,
    );
    w.add_collider(s, Shape::Sphere { r: 0.25 }, Material::default());

    // Capsule
    let cmass = MassProps::from_capsule(0.25, 0.5, 1000.0);
    let c = w.add_body(
        iso(vec3(2.0, 1.5, 0.0), quat_identity()),
        Velocity::default(),
        cmass,
        true,
    );
    w.add_collider(c, Shape::Capsule { r: 0.25, hh: 0.5 }, Material::default());

    // Planet gravity (center far below)
    w.queue_gravity_swap(GravitySpec::LayeredPlanet {
        surface_g: 9.81,
        radius:    6_371_000.0,
        center:    [0.0, -6_371_000.0, 0.0],
        min_r:     1_000.0,
    });

    // Pendulum: pivot + distance joint to sphere
    let pivot = w.add_body(
        iso(vec3(-5.0, 4.0, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::infinite(),
        false,
    );
    w.add_distance_joint(pivot, s, 2.0, 1e-6);

    for step in 0..120 {
        let stats = w.step(1.0/60.0);
        if step % 10 == 0 {
            println!("step {step:03}  pairs={} contacts={}", stats.pairs_tested, stats.contacts);
        }
    }
    // After creating the world, install a simple synthetic heightfield:
    use riftphys_terrain::HeightField;
    use glam::{UVec2, Vec2};

    fn make_sine_hf(nx: u32, nz: u32, cell: f32, amp: f32, k: f32) -> HeightField {
        let dims = UVec2::new(nx, nz);
        let cellv = Vec2::new(cell, cell);
        let mut heights = Vec::with_capacity((nx * nz) as usize);
        for z in 0..nz {
            for x in 0..nx {
                let wx = x as f32 * cell;
                let wz = z as f32 * cell;
                let h = amp * (k * wx).sin() * (k * wz).cos();
                heights.push(h);
            }
        }
        HeightField::from_heights(dims, cellv, heights)
    }

    // ...
    let hf = make_sine_hf(256, 256, 0.5, 0.25, 0.15);
    w.set_heightfield(hf, 0.0);

    // show a sample under the fast sphere every 20 ticks
    for step in 0..120 {
        let stats = w.step(1.0/60.0);
        if step % 10 == 0 {
            println!("step {step:03}  pairs={} contacts={}", stats.pairs_tested, stats.contacts);
        }
        if step % 20 == 0 {
            // grab the sphere’s xz and query terrain
            let sphere_pos = w.get_body_pose(s).pos; // add a tiny getter or read via debug for now
            if let Some((h, n)) = w.sample_terrain_height_normal(sphere_pos.x, sphere_pos.z) {
                println!("terrain @ ({:.2},{:.2}): h={:.3}  n=({:.2},{:.2},{:.2})",
                         sphere_pos.x, sphere_pos.z, h, n.x, n.y, n.z
                );
            }
        }
    }
    // after `let stats = w.step(dt);`
    let dt = 1.0/60.0;

    // Build a simple blade from the fast sphere: tip at the sphere, mid 0.5m back on +Y
    let sphere_pose = w.get_body_pose(s);               // add the tiny getter we mentioned earlier
    let sphere_vel  = w.get_body_vel(s);                // add a similar getter returning Velocity

    let tip_p0 = sphere_pose.pos;
    let tip_v  = sphere_vel.lin;

    let mid_p0 = sphere_pose.pos + riftphys_core::types::vec3(0.0, 0.5, 0.0);
    let mid_v  = sphere_vel.lin;                        // same lin vel (good enough for MVP)

    if let Some((box_idx, toi, n)) =
        w.sweep_blade_against_boxes(tip_p0, tip_v, 0.1, mid_p0, mid_v, 0.1, dt)
    {
        println!(
            "blade HIT: box_index={} at t={:.3}  n=({:+.2},{:+.2},{:+.2})",
            box_idx, toi, n.x, n.y, n.z
        );
    }

    let hash = w.step_hash();
    println!("final hash = {:02x?}", hash);
}
