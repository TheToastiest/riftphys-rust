use riftphys_world::{World};
use riftphys_core::BodyId;
use glam::Quat;

/// Tolerances for near-equality between active/shadow worlds
#[derive(Copy, Clone)]
pub struct Eps {
    pub pos: f32,        // meters
    pub vel: f32,        // m/s
    pub quat_deg: f32,   // degrees
    pub contacts: u32,   // allowed per-tick contact-count delta
}

pub struct EpochShadow {
    pub active: World,
    pub shadow: World,
    eq_ticks: u32,
    promote_after: u32,
    epoch_counter: u64,
}

impl EpochShadow {
    pub fn new(active: World, shadow: World, epoch_start: u64, promote_after: u32) -> Self {
        Self { active, shadow, eq_ticks: 0, promote_after, epoch_counter: epoch_start }
    }

    /// Bit-compare path (exact hash equality)
    pub fn step(&mut self, dt: f32) -> Option<u64> {
        let _sa = self.active.step(dt);
        let _ss = self.shadow.step(dt);

        if self.active.step_hash() == self.shadow.step_hash() {
            self.eq_ticks += 1;
            if self.eq_ticks >= self.promote_after {
                self.promote();
                return Some(self.epoch_counter);
            }
        } else {
            self.eq_ticks = 0;
        }
        None
    }

    /// Epsilon-compare path (pose/vel/orientation + contact count within eps)
    pub fn step_eps(&mut self, dt: f32, eps: Eps) -> Option<u64> {
        let sa = self.active.step(dt);
        let ss = self.shadow.step(dt);

        if near_equal_worlds(&self.active, &self.shadow, eps)
            && contacts_close(sa.contacts, ss.contacts, eps.contacts)
        {
            self.eq_ticks += 1;
            if self.eq_ticks >= self.promote_after {
                self.promote();
                return Some(self.epoch_counter);
            }
        } else {
            self.eq_ticks = 0;
        }
        None
    }

    fn promote(&mut self) {
        std::mem::swap(&mut self.active, &mut self.shadow);
        self.epoch_counter += 1;
        self.active.set_epoch(self.epoch_counter);
        self.eq_ticks = 0;
    }
}

fn near_equal_worlds(a: &World, b: &World, eps: Eps) -> bool {
    let n = a.num_bodies().min(b.num_bodies());
    for i in 0..n {
        let id = BodyId(i);
        let pa = a.get_body_pose(id);
        let pb = b.get_body_pose(id);

        // position
        if (pa.pos - pb.pos).length() > eps.pos { return false; }

        // orientation
        if quat_angle_deg(pa.rot, pb.rot) > eps.quat_deg { return false; }

        // linear velocity
        let va = a.get_body_vel(id).lin;
        let vb = b.get_body_vel(id).lin;
        if (va - vb).length() > eps.vel { return false; }
    }
    true
}

fn contacts_close(a: u32, b: u32, tol: u32) -> bool {
    if a > b { a - b <= tol } else { b - a <= tol }
}

fn quat_angle_deg(a: Quat, b: Quat) -> f32 {
    let d = a.dot(b).abs().min(1.0);
    (2.0 * d.acos()).to_degrees()
}
