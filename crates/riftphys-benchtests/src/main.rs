use anyhow::{bail, ensure, Result};
use std::env;
use std::ffi::{c_void, CString};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::{Duration, Instant};

use riftphys_core::{iso, quat_identity, vec3, BodyId, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_viz::DebugSettings;
use riftphys_world::world;

use riftphys_riftnet::sys;
use riftphys_riftnet::wire::{pack_snapshot, NetEnt};

#[derive(Clone, Copy)]
struct RunCfg {
    hz: u32,
    ticks: u32,
    print_every: u32,
    runs: u32,
    realtime_hz: Option<f32>,
    net: bool,
    net_port: u16,
}

fn env_u32(key: &str, default: u32) -> u32 {
    env::var(key).ok().and_then(|s| s.parse().ok()).unwrap_or(default)
}

fn env_u16(key: &str, default: u16) -> u16 {
    env::var(key).ok().and_then(|s| s.parse().ok()).unwrap_or(default)
}

fn env_bool(key: &str, default: bool) -> bool {
    match env::var(key).ok().as_deref() {
        Some("1") | Some("true") | Some("TRUE") | Some("yes") | Some("YES") | Some("on") | Some("ON") => true,
        Some("0") | Some("false") | Some("FALSE") | Some("no") | Some("NO") | Some("off") | Some("OFF") => false,
        None => default,
        _ => default,
    }
}

fn env_f32_opt(key: &str) -> Option<f32> {
    env::var(key).ok().and_then(|s| s.parse().ok())
}

unsafe extern "C" fn on_server_event(_ev: *const sys::RiftEvent, _user: *mut c_void) {}

struct NetServer {
    server: *mut sys::RiftServer,
    _host_keepalive: CString,
}

impl NetServer {
    fn new(port: u16) -> Result<Self> {
        let host = CString::new("0.0.0.0")?;
        let cfg = sys::RiftServerConfig {
            host_address: host.as_ptr(),
            port,
            event_callback: Some(on_server_event),
            user_data: std::ptr::null_mut(),
        };
        let server = unsafe { sys::rift_server_create(&cfg) };
        if server.is_null() {
            bail!("rift_server_create failed");
        }
        let res = unsafe { sys::rift_server_start(server) };
        ensure!(
            matches!(res, sys::RiftResult::RIFT_SUCCESS),
            "rift_server_start failed: {:?}",
            res
        );

        Ok(Self { server, _host_keepalive: host })
    }

    fn broadcast_snapshot(&self, world: &world::World, tick_u64: u64) {
        if self.server.is_null() {
            return;
        }

        // Epoch is useful to the viewer but not required for correctness.
        let epoch_u64 = world.epoch_id as u64;

        let n = world.num_bodies() as u32;
        let mut ents = Vec::with_capacity(n as usize);

        for id in 0..n {
            let bid = BodyId(id);
            let pose = world.get_body_pose(bid);

            // Keep it simple: always send as sphere unless you care.
            // If you want exact shapes, use your existing `primary_shape` API here.
            ents.push(NetEnt {
                id,
                px: pose.pos.x,
                py: pose.pos.y,
                pz: pose.pos.z,
                qx: pose.rot.x,
                qy: pose.rot.y,
                qz: pose.rot.z,
                qw: pose.rot.w,
                sx: 0.25,
                sy: 0.25,
                sz: 0.25,
                kind: 1,
            });
        }

        for payload in pack_snapshot(tick_u64, epoch_u64, &ents) {
            unsafe {
                let _ = sys::rift_server_broadcast(
                    self.server,
                    payload.as_ptr(),
                    payload.len() as sys::size_t,
                );
            }
        }
    }
}

#[derive(Clone, Copy)]
struct CcdHandles {
    wall_center_x: f32,
    wall_hx: f32,
    sphere_r: f32,
    capsule_r: f32,

    sphere: BodyId,
    capsule: BodyId,
}

fn build_ccd_scene(print_every: u32) -> (world::World, CcdHandles) {
    let mut w = world::WorldBuilder::new().with_capacity(64, 64).build();
    w.set_epoch(1);
    w.set_rng_seed(0xBADC0FFEE);

    w.set_debug(DebugSettings {
        print_every,
        show_bodies: false,
        show_contacts: false,
        show_impulses: false,
        show_energy: false,
        max_lines: 10,
        ..DebugSettings::default()
    });

    let mat_dyn = material(MaterialId::RubberSoft);
    let mat_ground = material(MaterialId::Grit);
    let mat_wall = material(MaterialId::Grit);

    // Ground
    let ground = w.add_body(
        iso(vec3(0.0, -0.25, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::infinite(),
        false,
    );
    w.add_collider(ground, Shape::Box { hx: 100.0, hy: 0.25, hz: 100.0 }, mat_ground);

    // Wall (thin box)
    let wall_center_x = 0.0f32;
    let wall_hx = 0.05f32;
    let wall = w.add_body(
        iso(vec3(wall_center_x, 1.5, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::infinite(),
        false,
    );
    w.add_collider(wall, Shape::Box { hx: wall_hx, hy: 2.0, hz: 2.0 }, mat_wall);

    // Fast sphere (CCD)
    let sphere_r = 0.25f32;
    let sphere = w.add_body(
        iso(vec3(-5.0, 2.0, 0.0), quat_identity()),
        Velocity { lin: vec3(20.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
        MassProps::from_sphere(sphere_r, MaterialId::Steel),
        true,
    );
    w.add_collider(sphere, Shape::Sphere { r: sphere_r }, mat_dyn);

    // Fast capsule (CCD) — intentionally NOT first collider (regression target)
    let capsule_r = 0.25f32;
    let capsule_hh = 0.50f32;

    let capsule = w.add_body(
        iso(vec3(-5.0, 1.25, 0.0), quat_identity()),
        Velocity { lin: vec3(80.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
        MassProps::from_capsule(capsule_r, capsule_hh, MaterialId::Steel),
        true,
    );

    // First collider is NOT the capsule.
    w.add_collider(capsule, Shape::Sphere { r: 0.10 }, mat_dyn);
    // Capsule comes second.
    w.add_collider(capsule, Shape::Capsule { r: capsule_r, hh: capsule_hh }, mat_dyn);

    let h = CcdHandles {
        wall_center_x,
        wall_hx,
        sphere_r,
        capsule_r,
        sphere,
        capsule,
    };
    (w, h)
}

#[derive(Clone)]
struct RunOut {
    hash_hex: String,
    sphere_max_x: f32,
    capsule_max_x: f32,
}

fn run_once(cfg: RunCfg, running: &Arc<AtomicBool>, net: Option<&NetServer>) -> Result<RunOut> {
    let dt = 1.0f32 / (cfg.hz as f32);

    let (mut w, h) = build_ccd_scene(cfg.print_every);

    let wall_left_face_x = h.wall_center_x - h.wall_hx;
    let sphere_limit = wall_left_face_x - h.sphere_r;
    let capsule_limit = wall_left_face_x - h.capsule_r;

    let mut sphere_max_x = f32::NEG_INFINITY;
    let mut capsule_max_x = f32::NEG_INFINITY;

    let mut pacer = cfg
        .realtime_hz
        .map(|hz| (Instant::now(), Duration::from_secs_f32(1.0 / hz)));

    for t in 0..cfg.ticks {
        if !running.load(Ordering::SeqCst) {
            break;
        }

        let _ = w.step(dt);

        let sp = w.get_body_pose(h.sphere).pos;
        let cp = w.get_body_pose(h.capsule).pos;

        sphere_max_x = sphere_max_x.max(sp.x);
        capsule_max_x = capsule_max_x.max(cp.x);

        if cfg.print_every != 0 && (t % cfg.print_every) == 0 {
            let sv = w.get_body_vel(h.sphere).lin;
            let cv = w.get_body_vel(h.capsule).lin;
            println!(
                "t={}  sphere x={:+.4} v={:+.3} | capsule x={:+.4} v={:+.3}",
                t, sp.x, sv.x, cp.x, cv.x
            );
        }

        if let Some(ns) = net {
            ns.broadcast_snapshot(&w, t as u64);
        }

        if let Some((ref mut last, frame)) = pacer {
            let now = Instant::now();
            let next = *last + frame;
            if now < next {
                std::thread::sleep(next - now);
            }
            *last = Instant::now();
        }
    }

    // Tolerances: allow tiny solver penetration, but not tunneling.
    // If either max_x crosses the wall face by centimeters, that’s a regression.
    let tol = 1.0e-2f32;
    ensure!(
        sphere_max_x <= sphere_limit + tol,
        "SPHERE TUNNELED: max_x={:+.6} limit={:+.6} (tol={})",
        sphere_max_x,
        sphere_limit,
        tol
    );
    ensure!(
        capsule_max_x <= capsule_limit + tol,
        "CAPSULE TUNNELED: max_x={:+.6} limit={:+.6} (tol={})",
        capsule_max_x,
        capsule_limit,
        tol
    );

    let hash_hex = format!("{:02x?}", w.step_hash());

    Ok(RunOut { hash_hex, sphere_max_x, capsule_max_x })
}

fn main() -> Result<()> {
    let cfg = RunCfg {
        hz: env_u32("RPHYS_HZ", 120).clamp(30, 1000),
        ticks: env_u32("RPHYS_TICKS", 360).max(1),
        print_every: env_u32("RPHYS_PRINT_EVERY", 0),
        runs: env_u32("RPHYS_RUNS", 2).max(1),
        realtime_hz: env_f32_opt("RPHYS_REALTIME_HZ"),
        net: env_bool("RPHYS_NET", false),
        net_port: env_u16("RPHYS_PORT", 49111),
    };

    let running = Arc::new(AtomicBool::new(true));
    {
        let r = running.clone();
        ctrlc::set_handler(move || {
            r.store(false, Ordering::SeqCst);
        })?;
    }

    let net = if cfg.net { Some(NetServer::new(cfg.net_port)?) } else { None };

    let mut first: Option<RunOut> = None;

    for run_idx in 0..cfg.runs {
        println!("--- run {}/{} ---", run_idx + 1, cfg.runs);

        let out = run_once(cfg, &running, net.as_ref())?;

        println!(
            "PASS: sphere_max_x={:+.6}  capsule_max_x={:+.6}\nhash={}",
            out.sphere_max_x, out.capsule_max_x, out.hash_hex
        );

        if let Some(ref prev) = first {
            ensure!(
                out.hash_hex == prev.hash_hex,
                "NON-DETERMINISTIC: hash mismatch\nprev={}\nnow ={}",
                prev.hash_hex,
                out.hash_hex
            );
        } else {
            first = Some(out);
        }
    }

    Ok(())
}
