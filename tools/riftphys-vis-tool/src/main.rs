//! riftphys-vis-tool — minimal top-down viewer (winit 0.28 + pixels 0.13)

use std::sync::Arc;
use std::time::Instant;

use glam::{Vec2, Vec3};
use pixels::{Pixels, SurfaceTexture};
use winit::{
    dpi::LogicalSize,
    event::{ElementState, Event, KeyboardInput, VirtualKeyCode, WindowEvent},
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

use riftphys_viz::DebugSettings;

use riftphys_world::*;
use riftphys_core::{iso, vec3, quat_identity, Velocity};
use riftphys_geom::{aabb_of, Aabb, MassProps, Material, Shape};
use riftphys_gravity::GravitySpec;

use riftphys_aero::{CombinedAero, FlatPlateDrag, ISA, SimpleWing};
use riftphys_acceleration::{SimplePropulsion, ThrottleCurve};
use riftphys_controllers::BalanceParams;

/* ---------------- env helpers ---------------- */
fn env_u32(key: &str, default: u32) -> u32 {
    std::env::var(key).ok().and_then(|s| s.parse().ok()).unwrap_or(default)
}
fn env_f32(key: &str, default: f32) -> f32 {
    std::env::var(key).ok().and_then(|s| s.parse().ok()).unwrap_or(default)
}

/* ---------------- camera ---------------- */
#[derive(Clone, Copy)]
struct Cam {
    center: Vec2, // world (x,z)
    ppm: f32,     // pixels per meter
}
impl Cam {
    fn world_to_screen(&self, p: Vec2, w: u32, h: u32) -> (i32, i32) {
        let sx = (w as f32 * 0.5) + (p.x - self.center.x) * self.ppm;
        let sy = (h as f32 * 0.5) - (p.y - self.center.y) * self.ppm;
        (sx.round() as i32, sy.round() as i32)
    }
}

/* ---------------- tiny raster helpers ---------------- */
fn put(px: &mut [u8], w: u32, h: u32, x: i32, y: i32, rgba: [u8; 4]) {
    if x < 0 || y < 0 { return; }
    let (x, y) = (x as u32, y as u32);
    if x >= w || y >= h { return; }
    let i = ((y * w + x) * 4) as usize;
    px[i..i + 4].copy_from_slice(&rgba);
}
fn line(px: &mut [u8], w: u32, h: u32, mut x0: i32, mut y0: i32, x1: i32, y1: i32, rgba: [u8; 4]) {
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    loop {
        put(px, w, h, x0, y0, rgba);
        if x0 == x1 && y0 == y1 { break; }
        let e2 = 2 * err;
        if e2 >= dy { err += dy; x0 += sx; }
        if e2 <= dx { err += dx; y0 += sy; }
    }
}
fn rect_outline(px: &mut [u8], w: u32, h: u32, x0: i32, y0: i32, x1: i32, y1: i32, rgba: [u8; 4]) {
    line(px, w, h, x0, y0, x1, y0, rgba);
    line(px, w, h, x1, y0, x1, y1, rgba);
    line(px, w, h, x1, y1, x0, y1, rgba);
    line(px, w, h, x0, y1, x0, y0, rgba);
}
fn draw_grid(px: &mut [u8], w: u32, h: u32, cam: Cam, step_m: f32) {
    let rgba = [40, 40, 40, 255];
    let half = 50.0;
    for i in -50..=50 {
        let x = i as f32 * step_m + cam.center.x;
        let z0 = cam.center.y - half * step_m;
        let z1 = cam.center.y + half * step_m;
        let (x0, y0) = cam.world_to_screen(Vec2::new(x, z0), w, h);
        let (x1, y1) = cam.world_to_screen(Vec2::new(x, z1), w, h);
        line(px, w, h, x0, y0, x1, y1, rgba);
    }
    for k in -50..=50 {
        let z = k as f32 * step_m + cam.center.y;
        let x0 = cam.center.x - half * step_m;
        let x1 = cam.center.x + half * step_m;
        let (u0, v0) = cam.world_to_screen(Vec2::new(x0, z), w, h);
        let (u1, v1) = cam.world_to_screen(Vec2::new(x1, z), w, h);
        line(px, w, h, u0, v0, u1, v1, rgba);
    }
}
fn draw_aabb_xz(px: &mut [u8], w: u32, h: u32, cam: Cam, bb: &Aabb, rgba: [u8; 4]) {
    let (sx0, sy0) = cam.world_to_screen(Vec2::new(bb.min.x, bb.min.z), w, h);
    let (sx1, sy1) = cam.world_to_screen(Vec2::new(bb.max.x, bb.max.z), w, h);
    rect_outline(px, w, h, sx0, sy0, sx1, sy1, rgba);
}

/* ---------------- store shape for re-AABB each frame ---------------- */
#[derive(Clone, Copy)]
struct DrawCollider {
    body: riftphys_core::BodyId,
    shape: Shape,
}

/* ---------------- scene ---------------- */
fn build_scene() -> (World, Vec<DrawCollider>, riftphys_core::BodyId) {
    let mut w = WorldBuilder::new().with_capacity(256, 256).build();
    w.set_epoch(1);
    w.set_rng_seed(0xBADC0FFEE);
    w.set_debug(DebugSettings {
        print_every: 20,
        show_bodies: false,
        show_contacts: false,
        show_impulses: false,
        show_energy: false,
        max_lines: 10,
        ..DebugSettings::default()
    });

    let mut drawlist: Vec<DrawCollider> = Vec::new();

    // Ground
    let ground = w.add_body(iso(vec3(0.0, 0.0, 0.0), quat_identity()), Velocity::default(), MassProps::infinite(), false);
    let ground_shape = Shape::Box { hx: 10.0, hy: 0.5, hz: 10.0 };
    w.add_collider(ground, ground_shape, Material::default());
    drawlist.push(DrawCollider { body: ground, shape: ground_shape });

    // Wall
    let wall = w.add_body(iso(vec3(-3.5, 2.0, 0.0), quat_identity()), Velocity::default(), MassProps::infinite(), false);
    let wall_shape = Shape::Box { hx: 0.05, hy: 2.0, hz: 2.0 };
    w.add_collider(wall, wall_shape, Material::default());
    drawlist.push(DrawCollider { body: wall, shape: wall_shape });

    // Sphere
    let s = w.add_body(iso(vec3(-5.0, 2.0, 0.0), quat_identity()),
                       Velocity { lin: vec3(20.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
                       MassProps::from_sphere(0.25, 1000.0), true);
    let sphere_shape = Shape::Sphere { r: 0.25 };
    w.add_collider(s, sphere_shape, Material::default());
    drawlist.push(DrawCollider { body: s, shape: sphere_shape });

    // Capsule
    let c = w.add_body(iso(vec3(2.0, 1.5, 0.0), quat_identity()),
                       Velocity { lin: vec3(250.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
                       MassProps::from_capsule(0.25, 0.5, 750.0), true);
    let capsule_shape = Shape::Capsule { r: 0.25, hh: 0.5 };
    w.add_collider(c, capsule_shape, Material::default());
    drawlist.push(DrawCollider { body: c, shape: capsule_shape });

    // Balance demo
    let pelvis = w.add_body(iso(vec3(0.0, 1.20, 0.50), quat_identity()), Velocity::default(), MassProps::from_capsule(0.15, 0.35, 80.0), true);
    let pelvis_shape = Shape::Capsule { r: 0.15, hh: 0.35 };
    w.add_collider(pelvis, pelvis_shape, Material::default());
    drawlist.push(DrawCollider { body: pelvis, shape: pelvis_shape });

    let left_foot = w.add_body(iso(vec3(-0.37, 0.55, 0.50), quat_identity()), Velocity::default(), MassProps::from_box(vec3(0.12, 0.05, 0.25), 20.0), true);
    let right_foot = w.add_body(iso(vec3( 0.37, 0.55, 0.50), quat_identity()), Velocity::default(), MassProps::from_box(vec3(0.12, 0.05, 0.25), 20.0), true);
    let foot_shape = Shape::Box { hx: 0.12, hy: 0.051, hz: 0.25 };
    w.add_collider(left_foot,  foot_shape, Material::default());
    w.add_collider(right_foot, foot_shape, Material::default());
    drawlist.push(DrawCollider { body: left_foot,  shape: foot_shape });
    drawlist.push(DrawCollider { body: right_foot, shape: foot_shape });

    w.add_balance_controller(pelvis, left_foot, right_foot,
                             BalanceParams { k_accel: 25.0, max_accel: 40.0, com_height: 1.0, quantize: 1e-6 });

    // Aero/prop on capsule (+ sphere drag)
    let isa = ISA::default();
    let drag = Arc::new(FlatPlateDrag { cd: 1.0, area_m2: 0.20, isa });
    let drag_h = w.models_mut().register_aero(drag);

    let aero_capsule = Arc::new(CombinedAero::new(vec![
        Arc::new(FlatPlateDrag { cd: 1.0, area_m2: 0.60, isa: ISA::default() }),
        Arc::new(SimpleWing { cl_per_rad: 6.0, stall_rad: 0.8, area_m2: 0.60, lift_dir_body: Vec3::Z,
            isa: ISA::default(), cd0: 0.05, k_induced: 0.05 }),
    ]));
    let prop_caps = Arc::new(SimplePropulsion {
        thrust_constant_n: 0.0, thrust_max_n: 3000.0, curve: ThrottleCurve::QuadEaseIn
    });
    let aero_capsule_h = w.models_mut().register_aero(aero_capsule);
    let prop_caps_h    = w.models_mut().register_prop(prop_caps);

    w.set_body_accel(s, Some(drag_h), None, 0.20, 0.0, None);
    w.set_body_accel(c, Some(aero_capsule_h), Some(prop_caps_h), 0.60, 0.55, None);
    let v0 = w.get_body_vel(c);
    w.set_body_vel(c, Velocity { lin: v0.lin + vec3(3.0, 0.0, 0.0), ang: v0.ang });

    // Gravity → epoch
    w.queue_gravity_swap(GravitySpec::LayeredPlanet {
        surface_g: 9.81, radius: 6_371_000.0, center: [0.0, -6_371_000.0, 0.0], min_r: 1_000.0
    });

    (w, drawlist, s)
}

/* ---------------- main ---------------- */
fn main() {
    // determinism clock (120..200 Hz)
    let hz = env_u32("RPHYS_HZ", 120).clamp(120, 200);
    let mut dt = std::env::var("RPHYS_DT").ok().and_then(|s| s.parse::<f32>().ok()).unwrap_or(1.0 / hz as f32);
    if dt <= 0.0 { dt = 1.0 / 120.0; }

    // camera + grid
    let scale = env_f32("VIS_SCALE", 48.0);
    let mut cam = Cam { center: Vec2::new(0.0, 0.0), ppm: scale };
    let mut show_grid = true;

    // window (winit 0.28: build returns Result<Window, OsError> -> unwrap it)
    let el = EventLoop::new();
    let size = LogicalSize::new(1280.0, 800.0);
    let window = WindowBuilder::new()
        .with_title("riftphys-vis-tool")
        .with_inner_size(size)
        .build(&el)
        .expect("failed to create window");

    let size_px = window.inner_size();
    let width = size_px.width.max(1);
    let height = size_px.height.max(1);
    let surface = SurfaceTexture::new(width, height, &window);
    let mut pixels = Pixels::new(width, height, surface).expect("pixels init failed");

    // scene
    let (mut world, drawlist, sphere_id) = build_scene();

    let mut last = Instant::now();
    let mut acc = 0.0f32;
    let mut paused = false;

    el.run(move |event, _, control_flow| {
        *control_flow = ControlFlow::Poll;

        match event {
            Event::WindowEvent { event, .. } => match event {
                WindowEvent::CloseRequested => *control_flow = ControlFlow::Exit,
                WindowEvent::KeyboardInput {
                    input: KeyboardInput { state, virtual_keycode: Some(key), .. }, ..
                } => {
                    let down = state == ElementState::Pressed;
                    match key {
                        VirtualKeyCode::Escape => *control_flow = ControlFlow::Exit,
                        VirtualKeyCode::Space  => if down { paused = !paused; }
                        VirtualKeyCode::G      => if down { show_grid = !show_grid; }
                        VirtualKeyCode::Q      => if down { cam.ppm = (cam.ppm * 0.9).max(8.0); }
                        VirtualKeyCode::E      => if down { cam.ppm = (cam.ppm * 1.1).min(400.0); }
                        VirtualKeyCode::W      => if down { cam.center.y -= 0.25; }
                        VirtualKeyCode::S      => if down { cam.center.y += 0.25; }
                        VirtualKeyCode::A      => if down { cam.center.x -= 0.25; }
                        VirtualKeyCode::D      => if down { cam.center.x += 0.25; }
                        VirtualKeyCode::Left   => if down && paused { world.step(dt); }
                        VirtualKeyCode::Right  => if down && paused { world.step(dt); }
                        VirtualKeyCode::R      => if down { cam.center = Vec2::ZERO; cam.ppm = scale; }
                        _ => {}
                    }
                }
                _ => {}
            },
            Event::MainEventsCleared => {
                // fixed-step update
                let now = Instant::now();
                let dt_real = (now - last).as_secs_f32();
                last = now;

                if !paused {
                    acc += dt_real;
                    if acc > 0.25 { acc = 0.25; } // cap hiccups
                    while acc >= dt {
                        world.step(dt);
                        acc -= dt;
                    }
                }
                window.request_redraw();
            }
            Event::RedrawRequested(_) => {
                // clear
                let frame = pixels.frame_mut();
                for px in frame.chunks_exact_mut(4) { px.copy_from_slice(&[12, 12, 16, 255]); }

                if show_grid { draw_grid(frame, width, height, cam, 1.0); }

                // draw colliders (recompute AABB each frame)
                for dc in &drawlist {
                    let pose = world.get_body_pose(dc.body);
                    let bb = aabb_of(&dc.shape, &pose);
                    let color = match dc.shape {
                        Shape::Box { .. }     => [100, 200, 255, 255],
                        Shape::Sphere { .. }  => [220, 160, 80, 255],
                        Shape::Capsule { .. } => [170, 120, 240, 255],
                    };
                    draw_aabb_xz(frame, width, height, cam, &bb, color);
                }

                // mark sphere
                let pose_s = world.get_body_pose(sphere_id);
                let p = Vec2::new(pose_s.pos.x, pose_s.pos.z);
                let (sx, sy) = cam.world_to_screen(p, width, height);
                put(frame, width, height, sx, sy, [255,255,255,255]);
                line(frame, width, height, sx-4, sy, sx+4, sy, [255,255,255,255]);
                line(frame, width, height, sx, sy-4, sx, sy+4, [255,255,255,255]);

                let _ = pixels.render();
            }
            _ => {}
        }
    });
}
