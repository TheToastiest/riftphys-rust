use std::fs;
use anyhow::*;
use std::result::Result::Ok;
use glam::{Vec3};
use riftphys_world::*;

use riftphys_core::{vec3, iso, quat_identity, BodyId};
use riftphys_geom::Shape;
use riftphys_viz::DebugSettings;
use riftphys_gravity::GravitySpec;
use riftphys_core::Velocity;
use riftphys_geom::{MassProps};
use riftphys_materials::materials::*;
use glam::{UVec2, Vec2};
use riftphys_terrain::HeightField;
use riftphys_bench_helpers::bench_loco::{loco_tick_with_heading, PathKind, Walker};

use riftphys_io::RigData;
use riftphys_io::rig_physics::{PhysicsRig, humanoid_from_rig, load_into_world, RigMap};

use riftphys_locomotion::loco_state as loco;


fn load_physics_rig(
    physics_path: &str,
    rig_json_fallback: Option<&str>,
) -> Result<PhysicsRig> {
    if let Ok(txt) = fs::read_to_string(physics_path) {
        let phys: PhysicsRig = serde_json::from_str(&txt)?;
        Ok(phys)
    } else if let Some(rp) = rig_json_fallback {
        let rt = fs::read_to_string(rp)?;
        let rig: RigData = serde_json::from_str(&rt)?;
        let phys = humanoid_from_rig(&rig)?;
        Ok(phys)
    } else {
        bail!("No physics rig at {} and no .rig.json fallback", physics_path);
    }
}
fn spawn_walker_instance(
    world: &mut world::World,
    phys: &PhysicsRig,
    origin_ws: riftphys_core::Vec3,
    phase01: f32,
    path: PathKind,
    gait: loco::GaitSpec,
) -> Result<Walker> {
    let map: RigMap = load_into_world(world, phys)?;

    // Optional: debug mapping
    println!("=== New humanoid ===");
    for (name, id) in map.body.iter() {
        println!("Body {:>3}: {:<16}", id.0, name);

        // --- Deterministic Fix: Disable Sleep ---
        // Force the body to remain awake so the locomotion
        // logic and solver run every single tick.
        world.set_body_sleep_allowed(*id, false);
    }

    let pelvis = *map
        .body
        .get("pelvis")
        .ok_or_else(|| anyhow!("rig missing pelvis"))?;
    let left   = *map
        .body
        .get("l_foot")
        .ok_or_else(|| anyhow!("rig missing l_foot"))?;
    let right  = *map
        .body
        .get("r_foot")
        .ok_or_else(|| anyhow!("rig missing r_foot"))?;

    // Recenter to origin_ws using pelvis
    let p0     = world.get_body_pose(pelvis).pos;
    let delta  = vec3(origin_ws.x - p0.x, origin_ws.y - p0.y, origin_ws.z - p0.z);
    for (_, bid) in map.body.iter() {
        let mut pose = world.get_body_pose(*bid);
        pose.pos += delta;
        world.set_body_pose(*bid, pose);
    }

    let mut state = loco::LocoState::new(pelvis.0, left.0, right.0, gait);
    state.left_clk.t  = phase01 * gait.stance_dur;
    state.right_clk.t = state.left_clk.t + gait.stance_dur;

    let clocks = (state.left_clk, state.right_clk);
    for (name, id) in map.body.iter() {
        let inv_m = world.inv_mass_of(*id);
        println!("Body {}: {} | inv_mass: {}", id.0, name, inv_m);
        world.set_body_sleep_allowed(*id, false);
    }
    Ok(Walker {
        pelvis,
        left,
        right,
        state,
        clocks,
        gait,
        path,
    })

}
fn build_world(print_every: u32) -> (world::World, riftphys_terrain::HeightField) {


    let mut w = world::WorldBuilder::new()
        .with_capacity(1024, 4096)
        .build();

    w.set_epoch(1);
    w.set_rng_seed(0xC0FFEE);
    w.set_debug(DebugSettings {
        print_every,
        show_bodies:   true,
        show_contacts: true,
        show_impulses: true,
        show_energy:   true,
        max_lines:     16,
        ..DebugSettings::default()
    });
    w.queue_gravity_swap(GravitySpec::Uniform {
        g: vec3(0.0, -9.81, 0.0).into(),
    });


    // Ground box at y=0
    let mut mat_ground= material(MaterialId::Default);
    mat_ground.restitution = 0.0;
    mat_ground.mu_s = 1.2;
    mat_ground.mu_k = 1.0;

    let ground = w.add_body(
        iso(vec3(0.0, -0.25, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::infinite(),
        false,
    );
    w.add_collider(
        ground,
        Shape::Box {
            hx: 1000.0,
            hy: 0.25,
            hz: 1000.0,
        },
        mat_ground,
    );

    // Low-amplitude sine heightfield
    fn make_sine_hf(nx: u32, nz: u32, cell: f32, amp: f32, k: f32) -> HeightField {
        let dims = UVec2::new(nx, nz);
        let cellv = Vec2::new(cell, cell);
        let mut heights = Vec::with_capacity((nx * nz) as usize);
        for z in 0..nz {
            for x in 0..nx {
                let wx = x as f32 * cell;
                let wz = z as f32 * cell;
                heights.push(amp * (k * wx).sin() * (k * wz).cos());
            }
        }
        HeightField::from_heights(dims, cellv, heights, MaterialId::Default)
    }

    let hf = make_sine_hf(256, 256, 0.5, 0.25, 0.15);
    // w.set_heightfield(hf.clone(), 0.0);

    (w, hf)
}
fn spawn_walker_grid(
    world: &mut world::World,
    phys: &PhysicsRig,
) -> Result<Vec<Walker>> {
    let gait = loco::GaitSpec {
        stance_dur: 0.45,
        swing_dur:  0.45,
        step_len:   0.30,
        step_h:     0.10,
    };

    let rows = 3;
    let cols = 4;
    let spacing = 3.0;

    let mut walkers = Vec::new();
    let mut idx = 0;

    for r in 0..rows {
        for c in 0..cols {
            let x = (c as f32 - (cols as f32 - 1.0) * 0.5) * spacing;
            let z = (r as f32 - (rows as f32 - 1.0) * 0.5) * spacing;
            let origin = vec3(x, 0.0, z);

            let phase01 = ((idx as f32) * 0.15) % 1.0;

            let path = if (idx & 1) == 0 {
                PathKind::Straight {
                    dir_ws: Vec3::new(1.0, 0.0, 0.0),
                }
            } else {
                PathKind::Circle {
                    center: Vec3::new(x, 0.0, z),
                    radius: 2.0,
                    ang_vel: 0.5,
                    angle: 0.0,
                }
            };

            let w = spawn_walker_instance(world, phys, origin, phase01, path, gait)?;
            walkers.push(w);
            idx += 1;
        }
    }

    Ok(walkers)
}
fn env_u32(key: &str, default: u32) -> u32 {
    std::env::var(key)
        .ok()
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(default)
}

fn main() -> Result<()> {
    let hz = env_u32("RPHYS_HZ", 120).clamp(60, 240);
    let dt = std::env::var("RPHYS_DT")
        .ok()
        .and_then(|s| s.parse::<f32>().ok())
        .unwrap_or(1.0 / hz as f32);

    let ticks        = env_u32("RPHYS_TICKS", 3600);
    let print_every  = env_u32("RPHYS_PRINT_EVERY", 120);

    let (mut world, _hf) = build_world(print_every);
    let phys = load_physics_rig(
        "assets/superhuman.physics.json",
        Some("assets/superhuman.rig.json"),
    )?;

    let mut walkers = spawn_walker_grid(&mut world, &phys)?;

    for step in 0..ticks {
        for w in walkers.iter_mut() {
            w.step(&mut world, dt);
        }

        let _stats = world.step(dt);

        if print_every != 0 && (step as u32) % print_every == 0 {
            for (i, w) in walkers.iter().enumerate() {
                let p = world.get_body_pose(w.pelvis).pos;
                println!(
                    "tick {:6} | walker {:2} pelvis=({:+.3}, {:+.3}, {:+.3})",
                    step, i, p.x, p.y, p.z
                );
            }
        }
    }

    println!("final hash = {:02x?}", world.step_hash());
    Ok(())
}
