use riftphys_world::*;
use riftphys_core::{vec3, iso, quat_identity, Velocity};
use riftphys_geom::{Shape, MassProps, Material};

fn main() {
    let mut w = WorldBuilder::new().with_capacity(64, 64).build();

    // Ground (static)
    let ground = w.add_body(iso(vec3(0.0, 0.0, 0.0), quat_identity()), Velocity::default(), MassProps::infinite(), false);
    w.add_collider(ground, Shape::Box { hx: 10.0, hy: 0.5, hz: 10.0 }, Material::default());

    // Falling box
    let b = w.add_body(iso(vec3(0.0, 2.0, 0.0), quat_identity()), Velocity::default(),
                       MassProps::from_box(vec3(0.25,0.25,0.25), 1000.0), true);
    w.add_collider(b, Shape::Box { hx: 0.25, hy: 0.25, hz: 0.25 }, Material::default());

    for step in 0..60 {
        let stats = w.step(1.0/60.0);
        let hash = w.step_hash();
        println!("step {step:02}  pairs={}  hash={:02x?}", stats.pairs_tested, hash);
    }
}
