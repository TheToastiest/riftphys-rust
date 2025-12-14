use riftphys_world::*;
use riftphys_core::{vec3, iso, quat_identity, Velocity, EpochDescriptor};
use riftphys_geom::{Shape, MassProps};
use riftphys_materials::materials::{material, MaterialId};

fn main() {
    let mut w = world::WorldBuilder::new()
        .with_capacity(256, 256)
        .build();
    w.set_rng_seed(0xABCD1234);

    // Start on “Earth” gravity
    w.queue_epoch_swap(EpochDescriptor { gravity_g: [0.0, -9.81, 0.0] });

    // Ground: concrete slab
    let ground = w.add_body(
        iso(vec3(0.0, 0.0, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::infinite(),
        false,
    );
    let ground_mat = material(MaterialId::Concrete);
    w.add_collider(
        ground,
        Shape::Box { hx: 10.0, hy: 0.5, hz: 10.0 },
        ground_mat,
    );

    // Falling box: default solid
    let b = w.add_body(
        iso(vec3(0.0, 2.0, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::from_box(vec3(0.25, 0.25, 0.25), MaterialId::Default),
        true,
    );
    let box_mat = material(MaterialId::Default);
    w.add_collider(
        b,
        Shape::Box { hx: 0.25, hy: 0.25, hz: 0.25 },
        box_mat,
    );

    let dt = 1.0 / 60.0;
    for step in 0..200 {
        if step == 100 {
            // Swap to “Mars” gravity at boundary
            w.queue_epoch_swap(EpochDescriptor { gravity_g: [0.0, -3.711, 0.0] });
            println!("-- queued Mars epoch at start of next tick --");
        }

        let stats = w.step(dt);
        if step % 20 == 0 {
            println!(
                "step {step:03}  pairs={} contacts={}  epoch_id=0x{:016x}",
                stats.pairs_tested,
                stats.contacts,
                w.epoch_id
            );
        }
    }

    let hash = w.step_hash();
    println!("final hash = {:02x?}", hash);
}
