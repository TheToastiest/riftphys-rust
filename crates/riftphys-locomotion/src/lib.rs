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
    // --- Phase 21 additions ---
    pub heading_yaw_rad: f32,             // current pelvis/path heading (quantized)
    pub pending: Option<TransitionPlan>,  // armed plan, applied at next double-stance
    prev_l_stance: bool,
    prev_r_stance: bool,
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
            heading_yaw_rad: 0.0,
            pending: None,
            prev_l_stance: false,
            prev_r_stance: false,
        }
    }
    /// Arm a transition; will only take effect at next double-stance to remove races.
    pub fn enqueue(&mut self, plan: TransitionPlan) {
        self.pending = Some(plan);
    }

    /// Step both clocks and, if we enter a double-stance window, realize one slice of the plan.
    /// Returns a per-step directive your bench/world code can consume deterministically.
    pub fn step_and_plan(&mut self, dt: f32) -> StepDirective {
        let lp = self.left_clk.step(dt);
        let rp = self.right_clk.step(dt);

        // current stance flags + “just entered stance” in a small deterministic window
        let (mut l_stance_now, mut r_stance_now) = (false, false);
        let mut l_enter = false;
        let mut r_enter = false;

        if let Phase::Stance(s) = lp {
            l_stance_now = true;
            if !self.prev_l_stance && s <= 0.05 { l_enter = true; } // first 5% of stance
        }
        if let Phase::Stance(s) = rp {
            r_stance_now = true;
            if !self.prev_r_stance && s <= 0.05 { r_enter = true; }
        }

        // Gate fires when EITHER foot touches down, not only double-stance
        let gate = l_enter || r_enter;

        let mut out = StepDirective::default();

        if gate {
            if let Some(plan) = self.pending {
                match plan {
                    TransitionPlan::Turn(TurnPlan { mut yaw_total_rad, mut steps }) => {
                        steps = steps.max(1);
                        yaw_total_rad = q6(yaw_total_rad);
                        let per_step = q6(yaw_total_rad / steps as f32);

                        out.yaw_delta_rad = per_step;
                        self.heading_yaw_rad = q6(self.heading_yaw_rad + per_step);

                        let remaining = steps - 1;
                        if remaining > 0 {
                            self.pending = Some(TransitionPlan::Turn(TurnPlan {
                                yaw_total_rad: q6(yaw_total_rad - per_step),
                                steps: remaining,
                            }));
                        } else {
                            self.pending = None;
                        }
                        out.left_step_len  = self.spec.step_len;
                        out.right_step_len = self.spec.step_len;
                    }
                    TransitionPlan::Start(StartPlan { first_step_len }) => {
                        out.left_step_len  = first_step_len;
                        out.right_step_len = first_step_len;
                        self.pending = None;
                    }
                    TransitionPlan::Stop(_) => {
                        out.left_step_len  = 0.0;
                        out.right_step_len = 0.0;
                        out.stop_after = true;
                        self.pending = None;
                    }
                }
            } else {
                out.left_step_len  = self.spec.step_len;
                out.right_step_len = self.spec.step_len;
            }
        } else {
            out.left_step_len  = self.spec.step_len;
            out.right_step_len = self.spec.step_len;
        }

        // quantize for hash stability
        out.yaw_delta_rad = q6(out.yaw_delta_rad);
        out.left_step_len = q6(out.left_step_len);
        out.right_step_len = q6(out.right_step_len);

        // update prev flags (must be last)
        self.prev_l_stance = l_stance_now;
        self.prev_r_stance = r_stance_now;

        out
    }


    /// World/bench helper: compute the forward direction from heading.
    pub fn heading_dir(&self) -> Vec3 {
        // Y-up convention
        let q = Quat::from_rotation_y(self.heading_yaw_rad);
        q * Vec3::X
    }
}
#[inline]
fn q6(x: f32) -> f32 {
    // quantize to 1e-6 in f32
    (x * 1.0e6_f32).round() * 1.0e-6_f32
    // alternatively:
    // const INV_1E6: f32 = 1.0 / 1_000_000.0;
    // (x * 1_000_000.0_f32).round() * INV_1E6
}

#[derive(Copy, Clone)]
pub struct TurnPlan {
    pub yaw_total_rad: f32,   // total yaw to distribute
    pub steps: u32,           // number of steps over which to distribute
}
#[derive(Copy, Clone)]
pub struct StartPlan { pub first_step_len: f32 }
#[derive(Copy, Clone)]
pub struct StopPlan;

#[derive(Copy, Clone)]
pub enum TransitionPlan { Turn(TurnPlan), Start(StartPlan), Stop(StopPlan) }

/// What to do *this* step; bench_loco/world code can consume this.
#[derive(Copy, Clone, Default)]
pub struct StepDirective {
    pub yaw_delta_rad: f32,   // apply to pelvis facing / path heading this step
    pub left_step_len:  f32,  // in local +X of the heading
    pub right_step_len: f32,  // "
    pub stop_after: bool,     // end in double-stance; no further steps until re-armed
}
pub fn swing_target_dir(p_stance: Vec3, dir_ws: Vec3, step_len: f32, lift: f32, s: f32) -> Vec3 {
    let dir = dir_ws.normalize_or_zero();
    let p0 = p_stance;
    let p1 = p_stance + dir * step_len;
    let mid = p_stance + dir * (step_len * 0.5) + Vec3::new(0.0, lift, 0.0);
    if s <= 0.5 { hermite(p0, Vec3::ZERO, mid, Vec3::ZERO, s * 2.0) }
    else        { hermite(mid, Vec3::ZERO, p1, Vec3::ZERO, (s - 0.5) * 2.0) }
}
