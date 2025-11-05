use glam::{Vec3, Quat};

#[derive(Copy, Clone)]
pub struct GaitSpec {
    pub stance_dur: f32,   // seconds
    pub swing_dur:  f32,   // seconds
    pub step_len:   f32,   // meters in +X
    pub step_h:     f32,   // lift height
}

// Per-foot phase: 0..stance_dur then 0..swing_dur (wrap)
#[derive(Copy, Clone)]
pub struct FootClock { pub t: f32, pub stance_dur: f32, pub swing_dur: f32 }
impl FootClock {
    pub fn new(stance: f32, swing: f32) -> Self { Self { t: 0.0, stance_dur: stance, swing_dur: swing } }
    pub fn step(&mut self, dt: f32) -> Phase {
        self.t += dt;
        let cycle = self.stance_dur + self.swing_dur;
        if self.t >= cycle { self.t -= cycle; }
        if self.t < self.stance_dur { Phase::Stance(self.t / self.stance_dur) }
        else { Phase::Swing((self.t - self.stance_dur) / self.swing_dur) }
    }
}
#[derive(Copy, Clone)]
pub enum Phase { Stance(f32), Swing(f32) }

#[inline]
fn hermite(p0: Vec3, v0: Vec3, p1: Vec3, v1: Vec3, s: f32) -> Vec3 {
    let s2 = s*s; let s3 = s2*s;
    let h00 =  2.0*s3 - 3.0*s2 + 1.0;
    let h10 =       s3 - 2.0*s2 + s;
    let h01 = -2.0*s3 + 3.0*s2;
    let h11 =       s3 -     s2;
    h00*p0 + h10*v0 + h01*p1 + h11*v1
}

// Deterministic swing arc in world space
pub fn swing_target(p_stance: Vec3, step_len: f32, lift: f32, s: f32) -> Vec3 {
    let p0 = p_stance;
    let p1 = p_stance + Vec3::new(step_len, 0.0, 0.0);
    let mid = p_stance + Vec3::new(step_len*0.5, lift, 0.0);

    // two Hermites: p0→mid (0..0.5), mid→p1 (0.5..1)
    if s <= 0.5 {
        let ss = s*2.0;
        hermite(p0, Vec3::ZERO, mid, Vec3::ZERO, ss)
    } else {
        let ss = (s-0.5)*2.0;
        hermite(mid, Vec3::ZERO, p1, Vec3::ZERO, ss)
    }
}

pub struct LocoState {
    pub pelvis: u32, pub left: u32, pub right: u32, // body indices
    pub left_clk:  FootClock,
    pub right_clk: FootClock,
    pub spec: GaitSpec,
    pub left_stance_anchor_ws: Vec3,
    pub right_stance_anchor_ws: Vec3,
}
impl LocoState {
    pub fn new(pelvis: u32, left: u32, right: u32, spec: GaitSpec) -> Self {
        Self {
            pelvis, left, right,
            left_clk:  FootClock::new(spec.stance_dur, spec.swing_dur),
            right_clk: FootClock::new(spec.stance_dur, spec.swing_dur),
            spec,
            left_stance_anchor_ws: Vec3::ZERO,
            right_stance_anchor_ws: Vec3::ZERO,
        }
    }
}
