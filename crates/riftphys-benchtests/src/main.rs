use anyhow::{Result, ensure};
use std::sync::{atomic::{AtomicBool, Ordering}, Arc};
use std::time::{Duration, Instant};
use std::io::{self, Write};

use crossterm::event::{poll, read, Event, KeyCode, KeyModifiers, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};

use riftphys_core::{iso, quat_identity, vec3, Velocity};
use riftphys_core::rng::XorShift64;
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::{World, WorldBuilder};

struct TerminalGuard;
impl TerminalGuard {
    fn new() -> Result<Self> { enable_raw_mode()?; Ok(Self) }
}
impl Drop for TerminalGuard {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        println!("\r\n[System] Terminal restored. Simulation ended.");
    }
}

struct PerfMetrics {
    ema_step_time_ms: f32,
    tick_count: u64,
}

impl PerfMetrics {
    fn new() -> Self { Self { ema_step_time_ms: 0.0, tick_count: 0 } }

    fn update(&mut self, ms: f32) {
        if self.tick_count == 0 {
            self.ema_step_time_ms = ms;
        } else {
            self.ema_step_time_ms = (self.ema_step_time_ms * 0.90) + (ms * 0.10);
        }
        self.tick_count += 1;
    }
}

fn spawn_batch(w: &mut World, rng: &mut XorShift64, count: u32) {
    let mat_dyn = material(MaterialId::Skin);
    for _ in 0..count {
        let r = 0.2 + rng.next_f32_01() * 0.3;

        // THE FIX: Drop them strictly between 100.0 and 300.0 on the 400x400 map
        let px = 100.0 + rng.next_f32_01() * 200.0;
        let py = 5.0 + rng.next_f32_01();
        let pz = 100.0 + rng.next_f32_01() * 200.0;

        let b = w.add_body(
            iso(vec3(px, py, pz), quat_identity()),
            Velocity::default(),
            MassProps::from_sphere(r, MaterialId::Skin),
            true,
        );
        w.add_collider(b, Shape::Sphere { r }, mat_dyn);
    }
}

fn handle_input(running: &Arc<AtomicBool>, paused: &Arc<AtomicBool>) -> Result<()> {
    if poll(Duration::from_millis(0))? {
        if let Event::Key(key) = read()? {
            if key.kind == KeyEventKind::Press {
                match key.code {
                    KeyCode::Char(' ') => {
                        let state = paused.load(Ordering::Relaxed);
                        paused.store(!state, Ordering::Relaxed);
                        println!("\r[Control] Simulation {}", if !state { "PAUSED" } else { "RESUMED" });
                    }
                    KeyCode::Esc => { running.store(false, Ordering::SeqCst); }
                    KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                        running.store(false, Ordering::SeqCst);
                    }
                    _ => {}
                }
            }
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    let _guard = TerminalGuard::new()?;
    let running = Arc::new(AtomicBool::new(true));
    let paused = Arc::new(AtomicBool::new(false));

    let r = running.clone();
    ctrlc::set_handler(move || { r.store(false, Ordering::SeqCst); })?;

    let target_hz = 65.0;
    let dt = 1.0 / target_hz;
    let frame_budget_ms = dt * 1000.0;

    let mut w = WorldBuilder::new().with_capacity(80000, 80000).build();
    let mut rng = XorShift64::new(0xDEADBEEF);
    let mut metrics = PerfMetrics::new();

    let hf = riftphys_terrain::terrain::HeightField::from_heights(
        glam::UVec2::new(4096, 4096),
        glam::Vec2::new(4.0, 4.0),
        vec![0.0; 16777216],
        MaterialId::Grit
    );
    w.add_environment(Box::new(hf));

    let mut batch_size = 1;
    let mut ticks_since_spawn = 0;
    let spawn_interval_ticks = target_hz as u64;
    let mut spawning_active = true;

    // Spawn the Kinematic Player and capture the index
    let player_start = iso(vec3(200.0, 5.0, 200.0), quat_identity());
    let p_idx = w.add_player(player_start);

    println!("Starting Stress Test. [SPACE] Pause | [ESC/CTRL+C] Quit");

    let mut target_time = Instant::now();
    let frame_duration = Duration::from_secs_f32(dt);

    while running.load(Ordering::SeqCst) {
        handle_input(&running, &paused)?;

        if paused.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(16));
            continue;
        }

        if spawning_active && ticks_since_spawn >= spawn_interval_ticks {
            spawn_batch(&mut w, &mut rng, batch_size);
            // Overwrite line smoothly
            print!("\r\x1B[K");
            println!("[Spawn] +{} | Total: {}", batch_size, w.num_bodies());
            io::stdout().flush()?;
            batch_size *= 2;
            ticks_since_spawn = 0;
        }
        ticks_since_spawn += 1;

        let step_start = Instant::now();
        w.step(dt);
        let actual_ms = step_start.elapsed().as_secs_f32() * 1000.0;
        metrics.update(actual_ms);

        if metrics.tick_count % spawn_interval_ticks == 0 {
            // THE CANARY: Fetch player's Y position and grounded state
            let player_y = w.body_pose(w.players[p_idx].body).pos.y;
            let is_grounded = w.players[p_idx].grounded;

            print!("\r\x1B[K"); // Clear the line to prevent glitchy terminal artifacts
            println!(
                "Tick: {:05} | Bodies: {:05} (Sleep: {:05}) | EMA: {:.2}ms | Player Y: {:.2} (Gnd: {})",
                metrics.tick_count, w.num_bodies(), w.num_sleeping(), metrics.ema_step_time_ms, player_y, is_grounded
            );
            io::stdout().flush()?;

            if metrics.ema_step_time_ms > (frame_budget_ms * 0.85) {
                if spawning_active {
                    println!("\n[Performance] Breached budget. Engine settling... Spawning paused.");
                    spawning_active = false;
                }
            } else if metrics.ema_step_time_ms < (frame_budget_ms * 0.70) {
                if !spawning_active {
                    println!("\n[Performance] Engine recovered! Avalanche sleeping. Spawning resumed.");
                    spawning_active = true;
                }
            }
        }

        target_time += frame_duration;
        let now = Instant::now();
        if target_time > now {
            let remaining = target_time - now;
            if remaining > Duration::from_millis(2) {
                std::thread::sleep(remaining - Duration::from_millis(1));
            }
            while Instant::now() < target_time { std::hint::spin_loop(); }
        } else if now - target_time > Duration::from_millis(100) {
            target_time = now;
        }
    }

    Ok(())
}