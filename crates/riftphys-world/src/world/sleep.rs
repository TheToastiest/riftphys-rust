// crates/riftphys-world/src/world/sleep.rs
use std::sync::atomic::{AtomicU8, Ordering};
use riftphys_dynamics::Bodies;

pub struct SleepManager {
    pub awake: Vec<AtomicU8>,
    pub sleep_ticks: Vec<u16>,
    pub lin_threshold_sq: f32,
    pub ang_threshold_sq: f32,
    pub ticks_to_sleep: u16,
}

impl SleepManager {
    pub fn new(capacity: usize) -> Self {
        let mut awake = Vec::with_capacity(capacity);
        for _ in 0..capacity { awake.push(AtomicU8::new(1)); }

        Self {
            awake,
            sleep_ticks: vec![0; capacity],
            // Forgive up to 0.2 m/s of jitter (0.04 squared)
            lin_threshold_sq: 0.10,
            ang_threshold_sq: 0.2,
            // Put to bed after 0.5 seconds of rest instead of 0.75
            ticks_to_sleep: 60,
        }
    }

    pub fn ensure_capacity(&mut self, id: usize) {
        if id >= self.awake.len() {
            self.awake.resize_with(id + 1, || AtomicU8::new(1));
            self.sleep_ticks.resize(id + 1, 0);
        }
    }

    #[inline(always)]
    pub fn is_awake(&self, id: usize) -> bool {
        self.awake[id].load(Ordering::Relaxed) == 1
    }

    #[inline(always)]
    pub fn wake(&self, id: usize) {
        self.awake[id].store(1, Ordering::Relaxed);
    }

    /// Evaluates kinetic energy. Freezes bodies that have stopped moving.
    pub fn evaluate(&mut self, num_bodies: u32, bodies: &Bodies) {
        for i in 0..num_bodies {
            let idx = i as usize;
            if !self.is_awake(idx) || !bodies.is_dynamic(i) { continue; }

            let vel = bodies.vel(i);
            if vel.lin.length_squared() < self.lin_threshold_sq && vel.ang.length_squared() < self.ang_threshold_sq {
                self.sleep_ticks[idx] += 1;
                if self.sleep_ticks[idx] > self.ticks_to_sleep {
                    self.awake[idx].store(0, Ordering::Relaxed); // Put to sleep
                }
            } else {
                self.sleep_ticks[idx] = 0; // Reset if moving
            }
        }
    }
}