use riftphys_locomotion::loco_state as loco;
use riftphys_world::world::World;
use riftphys_core::{Velocity, vec3, BodyId};
use glam::Vec3;

/// PD -> velocity toward a target position (deterministic, fixed gains)
#[inline]
fn pd_vel_to_target(cur: Vec3, cur_v: Vec3, tgt: Vec3, kp: f32, kd: f32, vmax: f32) -> Vec3 {
    let v = (tgt - cur) * kp - cur_v * kd;
    let s = v.length();
    if s > vmax { v * (vmax / s) } else { v }
}

pub fn loco_tick_with_heading(
    world: &mut World,
    pelvis: u32, left: u32, right: u32,
    clocks: &mut (loco::FootClock, loco::FootClock),
    spec: &loco::GaitSpec,
    heading_dir_ws: glam::Vec3,
    left_len: f32, right_len: f32,
    dt: f32
) {
    // 1. Pelvis Height & Forward Velocity
    {
        let id = BodyId(pelvis);
        let pos = world.get_body_pose(id).pos;
        let mut vel = world.get_body_vel(id);

        // Target COM height
        let y_tgt = 1.20_f32;

        // BOOSTED PD GAINS: Necessary to pop up from resting radius
        let kp_y = 40.0;
        let kd_y = 5.0;
        let v_y_des = (y_tgt - pos.y) * kp_y - vel.lin.y * kd_y;

        // Desired forward speed from cadence
        let period = (spec.stance_dur + spec.swing_dur).max(1e-6);
        let v_forward = left_len.max(right_len) / period;
        let h = if heading_dir_ws.length_squared() < 1e-12 { Vec3::X } else { heading_dir_ws.normalize() };
        let v_fwd_des = h * v_forward;

        vel.lin.y += v_y_des * dt;
        vel.lin.x = v_fwd_des.x;
        vel.lin.z = v_fwd_des.z;

        world.set_body_vel(id, vel);
    }

    // 2. Foot Controls
    let idL = BodyId(left);
    let idR = BodyId(right);
    let pL: Vec3 = world.get_body_pose(idL).pos.into();
    let pR: Vec3 = world.get_body_pose(idR).pos.into();

    let phaseL = clocks.0.step(dt);
    let phaseR = clocks.1.step(dt);

    let h = if heading_dir_ws.length_squared() < 1e-12 { Vec3::X } else { heading_dir_ws.normalize() };

    // Determine Targets
    let mut tgtL = pL;
    let mut tgtR = pR;

    match phaseL {
        loco::Phase::Swing(s) => {
            tgtL = loco::swing_target_dir(pL, h, left_len, spec.step_h, s);
        }
        loco::Phase::Stance(_) => {
            // DETERMINISTIC FIX: In a real architecture, we'd store a 'stance_anchor'.
            // For this bench, we ensure the Y velocity is clamped so it doesn't
            // 'float' away while the pelvis lifts.
        }
    }

    match phaseR {
        loco::Phase::Swing(s) => {
            tgtR = loco::swing_target_dir(pR, h, right_len, spec.step_h, s);
        }
        _ => {}
    }

    // Foot PD Gains
    let kp = 15.0;   // Increased to overcome friction
    let kd = 2.0;
    let vmax = 5.0;

    // Apply Left
    let vL = world.get_body_vel(idL).lin.into();
    let mut vdL = pd_vel_to_target(pL, vL, tgtL, kp, kd, vmax);
    if matches!(phaseL, loco::Phase::Stance(_)) { vdL.y = 0.0; }
    world.set_body_vel(idL, Velocity { lin: vdL.into(), ang: world.get_body_vel(idL).ang });

    // Apply Right
    let vR = world.get_body_vel(idR).lin.into();
    let mut vdR = pd_vel_to_target(pR, vR, tgtR, kp, kd, vmax);
    if matches!(phaseR, loco::Phase::Stance(_)) { vdR.y = 0.0; }
    world.set_body_vel(idR, Velocity { lin: vdR.into(), ang: world.get_body_vel(idR).ang });
}
// Add PathKind to the helper so MeleeWalker can use it
pub enum PathKind {
    Straight { dir_ws: Vec3 },
    Circle   { center: Vec3, radius: f32, ang_vel: f32, angle: f32 },
}

pub struct Walker {
    pub pelvis: BodyId,
    pub left:   BodyId,
    pub right:  BodyId,

    pub state:  loco::LocoState,
    pub clocks: (loco::FootClock, loco::FootClock),
    pub gait:   loco::GaitSpec,
    pub path:   PathKind,
}
pub struct MeleeWalker {
    pub base: Walker,
    pub weapon_hand: BodyId,
    pub target_enemy: Option<BodyId>,
    pub strike_time: f32,
}

impl MeleeWalker {
    pub fn update_combat(&mut self, world: &mut World, dt: f32) {
        if self.strike_time > 0.0 {
            // Sinusoidal swing: rapid acceleration then deceleration
            let swing_force = (self.strike_time * 15.0).cos() * 100.0;
            let torque = Vec3::new(0.0, swing_force, 0.0);

            world.apply_torque(self.weapon_hand, torque);
            self.strike_time -= dt;
        }
    }
}

// In crates/riftphys-bench-helpers/src/bench_loco.rs

impl Walker {
    pub fn step(&mut self, world: &mut World, dt: f32) {
        // 1) Update desired heading
        match &mut self.path {
            PathKind::Straight { dir_ws } => {
                let mut d = *dir_ws;
                if d.length_squared() < 1.0e-12 { d = Vec3::X; }
                let d = d.normalize();
                self.state.heading_yaw_rad = d.z.atan2(d.x);
            }
            PathKind::Circle { center, radius, ang_vel, angle } => {
                *angle += *ang_vel * dt;
                let r = *radius;
                let x = center.x + r * angle.cos();
                let z = center.z + r * angle.sin();
                let pos = world.get_body_pose(self.pelvis).pos;
                let to_target = Vec3::new(x - pos.x, 0.0, z - pos.z);
                let d = if to_target.length_squared() < 1.0e-12 { Vec3::X } else { to_target.normalize() };
                self.state.heading_yaw_rad = d.z.atan2(d.x);
            }
        }

        // 2) Advance clocks and plan
        let directive = self.state.step_and_plan(dt);
        let h = self.state.heading_dir();

        let l_len = if directive.left_step_len == 0.0 { self.gait.step_len } else { directive.left_step_len };
        let r_len = if directive.right_step_len == 0.0 { self.gait.step_len } else { directive.right_step_len };

        // 3) Apply foot placements
        loco_tick_with_heading(
            world, self.pelvis.0, self.left.0, self.right.0,
            &mut self.clocks, &self.gait, h, l_len, r_len, dt,
        );

        self.state.left_clk = self.clocks.0;
        self.state.right_clk = self.clocks.1;
    }
}