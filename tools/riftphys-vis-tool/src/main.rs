//! riftphys-vis-tool — 3D Visual Debugger (winit 0.30 + riftphys-render/wgpu 0.20)

mod riftnet_client;
use parking_lot::Mutex;
//
// Controls:
//   Mouse Right-Button + move  -> look around (360° yaw, ±89° pitch)
//   W/A/S/D                    -> move forward/left/back/right
//   Q / E                      -> move down / up
//   R                          -> reset camera
//   Space                      -> pause/unpause physics
//   Esc                        -> exit
use std::sync::Arc;
use std::{collections::HashSet, time::Instant};
use glam::{Vec3, Quat};
use winit::{
    application::ApplicationHandler,
    dpi::{LogicalSize, PhysicalPosition, PhysicalSize},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowAttributes, WindowId, CursorGrabMode},
};
use winit::event::{ElementState, WindowEvent, KeyEvent, MouseButton, DeviceEvent, DeviceId};
use winit::event_loop::{ActiveEventLoop, EventLoop, DeviceEvents};
use wgpu::SurfaceError;
use riftphys_render::{Renderer, Instance, ShapeKind, trs};

use riftphys_viz::DebugSettings;
use riftphys_world::*;
use riftphys_core::{iso, vec3, quat_identity, Velocity, BodyId};
use riftphys_geom::{MassProps, Material, Shape};
use riftphys_gravity::GravitySpec;

use riftphys_aero::{CombinedAero, FlatPlateDrag, ISA, SimpleWing};
use riftphys_acceleration::{SimplePropulsion, ThrottleCurve};
use riftphys_controllers::BalanceParams;
/* ---------------- env helpers ---------------- */
fn env_u32(key: &str, default: u32) -> u32 {
    std::env::var(key).ok().and_then(|s| s.parse().ok()).unwrap_or(default)
}

/* ---------------- draw collider ---------------- */
#[derive(Clone, Copy)]
struct DrawCollider {
    body: BodyId,
    shape: Shape,
}

/* ---------------- scene ---------------- */
fn build_scene() -> (World, Vec<DrawCollider>, BodyId) {
    use std::sync::Arc;

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

    // Capsule (with aero/prop)
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

    // Aero/Prop
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

/* ---------------- instance build ---------------- */
fn build_instances(world: &World, drawlist: &[DrawCollider]) -> Vec<Instance> {
    fn color_rgb(r: u8, g: u8, b: u8) -> glam::Vec3 {
        glam::Vec3::new(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0)
    }
    let c_box  = color_rgb(100, 200, 255);
    let c_sph  = color_rgb(220, 160,  80);
    let c_caps = color_rgb(170, 120, 240);

    let mut v = Vec::with_capacity(drawlist.len());
    for dc in drawlist {
        let pose = world.get_body_pose(dc.body);
        let p = glam::Vec3::new(pose.pos.x, pose.pos.y, pose.pos.z);
        // Assuming pose.rot is glam::Quat
        let q: Quat = pose.rot;

        match dc.shape {
            Shape::Box { hx, hy, hz } => {
                v.push(Instance {
                    model: trs(p, q, glam::Vec3::new(hx, hy, hz)),
                    color: c_box,
                    kind: ShapeKind::Box { hx, hy, hz },
                });
            }
            Shape::Sphere { r } => {
                v.push(Instance {
                    model: trs(p, q, glam::Vec3::splat(r)),
                    color: c_sph,
                    kind: ShapeKind::Sphere { r },
                });
            }
            Shape::Capsule { r, hh } => {
                // renderer's unit capsule has radius=1 and cylinder half-height=1,
                // so half-length_unit = 1 + 1 = 2. Desired half-length = hh + r.
                // Scale Y by (hh + r) / 2; XZ by r.
                let sy = (hh + r) / 2.0;
                v.push(Instance {
                    model: trs(p, q, glam::Vec3::new(r, sy, r)),
                    color: c_caps,
                    kind: ShapeKind::CapsuleY { r, hh },
                });
            }
        }
    }
    v
}

/* ---------------- app ---------------- */
struct App {
    renderer: Option<Renderer>,
    window:   Option<Arc<Window>>,
    world: Option<World>,
    drawlist: Vec<DrawCollider>,
    paused: bool,
    last: Instant,
    acc: f32,
    dt_fixed: f32,
    pressed: HashSet<KeyCode>,
    rmb_down: bool,
    last_cursor: Option<PhysicalPosition<f64>>,
    move_speed: f32,
    sphere_id: BodyId,
    net: Option<riftnet_client::NetViewer>,
    net_ents: Arc<Mutex<Vec<riftphys_riftnet::wire::NetEnt>>>,
}

impl App {
    fn new() -> Self {
        let hz = env_u32("RPHYS_HZ", 120).clamp(120, 200);
        let dt_fixed = (1.0 / hz as f32).max(1.0 / 200.0);

        let (world, drawlist, sphere_id) = build_scene();

        Self {
            window: None,
            renderer: None,
            world: Some(world),
            drawlist,
            paused: false,
            last: Instant::now(),
            acc: 0.0,
            dt_fixed,
            pressed: HashSet::new(),
            rmb_down: false,
            last_cursor: None,
            move_speed: 5.0, // m/s base
            sphere_id,
            net: None,
            net_ents: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn on_resize(&mut self, size: PhysicalSize<u32>) {
        if let Some(r) = self.renderer.as_mut() {
            r.resize(size.width.max(1), size.height.max(1));
        }
    }

    fn handle_key(&mut self, ke: &KeyEvent, el: &ActiveEventLoop) {
        if let PhysicalKey::Code(code) = ke.physical_key {
            match ke.state {
                ElementState::Pressed => {
                    // on-press actions
                    match code {
                        KeyCode::Escape => { el.exit(); return; }
                        KeyCode::Space  => { self.paused = !self.paused; }
                        KeyCode::KeyR   => {
                            if let Some(r) = self.renderer.as_mut() {
                                r.camera.eye   = Vec3::new(-6.0, 4.0, 6.0);
                                r.camera.yaw   = 45_f32.to_radians();
                                r.camera.pitch = (-15_f32).to_radians();
                            }
                        }
                        _ => {}
                    }
                    self.pressed.insert(code);
                }
                ElementState::Released => {
                    self.pressed.remove(&code);
                }
            }
        }
    }

    fn handle_mouse(&mut self, event: &WindowEvent) {
        let window = match self.window.as_ref() { Some(w) => w, None => return };
        match *event {
            WindowEvent::MouseInput { state, button: MouseButton::Right, .. } => {
                self.rmb_down = state == ElementState::Pressed;
                let _ = window.set_cursor_grab(if self.rmb_down { CursorGrabMode::Locked } else { CursorGrabMode::None });
                window.set_cursor_visible(!self.rmb_down);
                self.last_cursor = None; // optional: no longer needed
            }
            _ => {}
        }
    }


    fn update_camera(&mut self, dt: f32) {
        let mut dir = Vec3::ZERO;
        if let Some(r) = self.renderer.as_ref() {
            if self.pressed.contains(&KeyCode::KeyW) { dir += r.camera.forward(); }
            if self.pressed.contains(&KeyCode::KeyS) { dir -= r.camera.forward(); }
            if self.pressed.contains(&KeyCode::KeyD) { dir += r.camera.right();   }
            if self.pressed.contains(&KeyCode::KeyA) { dir -= r.camera.right();   }
            if self.pressed.contains(&KeyCode::KeyE) { dir += Vec3::Y; }
            if self.pressed.contains(&KeyCode::KeyQ) { dir -= Vec3::Y; }
        }
        if let Some(r) = self.renderer.as_mut() {
            if dir.length_squared() > 1e-6 {
                r.camera.eye += dir.normalize() * self.move_speed * dt;
            }
        }
    }

    fn step_world(&mut self, dt_real: f32) {
        self.acc += dt_real.min(0.25);
        if self.paused { return; }
        if let Some(w) = self.world.as_mut() {
            while self.acc >= self.dt_fixed {
                w.step(self.dt_fixed);
                self.acc -= self.dt_fixed;
            }
        }
    }

    fn render(&mut self) {
        let r = match self.renderer.as_mut() { Some(r) => r, None => return };
        fn color_for(kind: u32, scale: glam::Vec3, id: u32) -> glam::Vec3 {
            // Heuristic: very large spheres => planet
            if kind == 1 && scale.max_element() > 1.0e5 {
                return glam::Vec3::new(0.35, 0.45, 0.95); // planet: cool blue
            }
            match kind {
                1 => glam::Vec3::new(0.95, 0.85, 0.45),       // small spheres: warm gold
                2 => glam::Vec3::new(0.30, 0.82, 0.62),       // boxes: teal
                3 => glam::Vec3::new(0.86, 0.42, 0.88),       // capsules: magenta
                _ => { // fallback: hash id to a pastel
                    let h = id.wrapping_mul(2654435761);
                    let r = ((h >>  0) & 255) as f32 / 255.0;
                    let g = ((h >>  8) & 255) as f32 / 255.0;
                    let b = ((h >> 16) & 255) as f32 / 255.0;
                    glam::Vec3::new(0.6 + 0.4*r, 0.6 + 0.4*g, 0.6 + 0.4*b)
                }
            }
        }
    
        // try net instances first
        let mut net_instances = Vec::new();
        {
            let ents = self.net_ents.lock();
            if !ents.is_empty() {
                net_instances.reserve(ents.len());
                for e in ents.iter() {
                    let p = glam::Vec3::new(e.px, e.py, e.pz);
                    let q = glam::Quat::from_xyzw(e.qx, e.qy, e.qz, e.qw);
                    let scale = glam::Vec3::new(e.sx, e.sy, e.sz);
                    let kind = match e.kind {
                        1 => ShapeKind::Sphere { r: e.sx },
                        2 => ShapeKind::Box    { hx: e.sx, hy: e.sy, hz: e.sz },
                        3 => {
                            let hh = 2.0 * e.sy - e.sx;             // sy = (hh + r)/2
                            ShapeKind::CapsuleY { r: e.sx, hh }
                        }
                        _ => ShapeKind::Sphere { r: 0.25 },
                    };
                    let color = color_for(e.kind, scale, e.id);     // <— use the helper
                    net_instances.push(Instance {
                        model: trs(p, q, scale),
                        color,
                        kind,
                    });

                }
            }
        }


        let instances =
            if !net_instances.is_empty() {
                net_instances
            } else {
                // fall back to local world (your existing path)
                let w = match self.world.as_ref() { Some(w) => w, None => return };
                build_instances(w, &self.drawlist)
            };

        match r.render(&instances) {
            Ok(()) => {}
            Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                if let Some(win) = self.window.as_ref() {
                    let size = win.inner_size();
                    r.resize(size.width.max(1), size.height.max(1));
                }
            }
            Err(wgpu::SurfaceError::OutOfMemory) => std::process::exit(1),
            Err(_) => {}
        }
    }

}

impl ApplicationHandler for App {
    fn device_event(&mut self, _el: &ActiveEventLoop, _id: DeviceId, event: DeviceEvent) {
        if !self.rmb_down { return; }
        if let DeviceEvent::MouseMotion { delta: (dx, dy) } = event {
            if let Some(r) = self.renderer.as_mut() {
                let sens = 0.0018; // tune
                r.camera.add_yaw_pitch(-(dx as f32) * sens, -(dy as f32) * sens);
            }
        }
    }


    fn resumed(&mut self, el: &ActiveEventLoop) {
        let attrs = WindowAttributes::default()
            .with_title("riftphys-vis-tool (3D)")
            .with_inner_size(LogicalSize::new(1280.0, 800.0));
        self.net = riftnet_client::NetViewer::connect("127.0.0.1", 49111, self.net_ents.clone());

        let window = Arc::new(el.create_window(attrs).expect("window"));
        let size = window.inner_size();
        el.listen_device_events(DeviceEvents::Always);
        // Arc clone here:
        let mut renderer = pollster::block_on(Renderer::new(window.clone()));
        renderer.resize(size.width.max(1), size.height.max(1));

        renderer.camera.eye   = Vec3::new(-6.0, 4.0, 6.0);
        renderer.camera.yaw   = 45_f32.to_radians();
        renderer.camera.pitch = (-15_f32).to_radians();

        self.renderer = Some(renderer);
        self.window   = Some(window);
        self.last = Instant::now();

        if let Some(w) = self.window.as_ref() { w.request_redraw(); }
    }


    fn window_event(&mut self, el: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        if self.window.as_ref().map(|w| w.id()) != Some(id) {
            return;
        }
        match event {
            WindowEvent::CloseRequested => el.exit(),
            WindowEvent::Resized(sz) => self.on_resize(sz),
            WindowEvent::ScaleFactorChanged { .. } => {
                if let Some(win) = self.window.as_ref() {
                    self.on_resize(win.inner_size());
                }
            }
            WindowEvent::RedrawRequested => {
                // dt for camera/world
                let now = Instant::now();
                let dt_real = (now - self.last).as_secs_f32();
                self.last = now;

                self.update_camera(dt_real);
                self.step_world(dt_real);
                self.render();
            }
            WindowEvent::KeyboardInput { event: ref ke, .. } => self.handle_key(ke, el),
            _ => {
                self.handle_mouse(&event);
            }
        }
    }

    fn about_to_wait(&mut self, _el: &ActiveEventLoop) {
        if let Some(win) = self.window.as_ref() {
            win.request_redraw();
        }
        if let Some(n) = &self.net { n.poll(); }
        if let Some(win) = self.window.as_ref() { win.request_redraw(); }

    }
}

fn main() -> anyhow::Result<()> {
    let event_loop = EventLoop::new()?;
    let mut app = App::new();
    event_loop.run_app(&mut app)?;
    Ok(())
}
