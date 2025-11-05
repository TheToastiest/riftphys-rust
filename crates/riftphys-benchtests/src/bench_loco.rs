use riftphys_locomotion as loco;
use riftphys_world::World;
use riftphys_core::{Velocity, vec3};
use glam::Vec3;

/// PD -> velocity toward a target position (deterministic, fixed gains)
#[inline]
fn pd_vel_to_target(cur: Vec3, cur_v: Vec3, tgt: Vec3, kp: f32, kd: f32, vmax: f32) -> Vec3 {
    let v = (tgt - cur) * kp - cur_v * kd;
    let s = v.length();
    if s > vmax { v * (vmax / s) } else { v }
}

/// Integrate one tick of “left swings / right stance” minimal gait.
/// - pelvis, left, right: body indices in the current world.
/// - spec: gait timings and arc parameters.
pub fn loco_tick(world: &mut World,
                 pelvis: u32, left: u32, right: u32,
                 clocks: &mut (loco::FootClock, loco::FootClock),
                 spec: &loco::GaitSpec,
                 dt: f32)
{
    // Read current poses
    let pL = world.get_body_pose(riftphys_core::BodyId(left)).pos.into();
    let pR = world.get_body_pose(riftphys_core::BodyId(right)).pos.into();

    // Foot phases (advance clocks deterministically)
    let phaseL = clocks.0.step(dt);
    let phaseR = clocks.1.step(dt);

    // Stance anchors (keep last stance XZ; Y follows ground via contacts naturally)
    // For a minimal demo: use current positions as stance references when in stance.
    // (You can persist anchors across frames if you want exact footprints.)
    let mut tgtL = pL;
    let mut tgtR = pR;

    // One-foot swing: left swings while right stances (or vice-versa)
    match phaseL {
        loco::Phase::Swing(s) => {
            tgtL = loco::swing_target(pL,  spec.step_len, spec.step_h, s);
        }
        loco::Phase::Stance(_s) => { /* keep tgtL ~ stance point */ }
    }
    match phaseR {
        loco::Phase::Swing(s) => {
            tgtR = loco::swing_target(pR,  spec.step_len, spec.step_h, s);
        }
        loco::Phase::Stance(_s) => { /* keep tgtR ~ stance point */ }
    }

    // Convert to desired velocities with small, fixed PD gains (deterministic)
    let kp = 6.0;  // m/s per meter error
    let kd = 1.2;  // damping
    let vmax = 2.5; // clamp

    // Left foot
    {
        let id = riftphys_core::BodyId(left);
        let curv = world.get_body_vel(id).lin.into();
        let v_des = pd_vel_to_target(pL, curv, tgtL, kp, kd, vmax);
        let mut v = world.get_body_vel(id);
        v.lin = vec3(v_des.x, v_des.y, v_des.z);
        world.set_body_vel(id, v);
    }
    // Right foot
    {
        let id = riftphys_core::BodyId(right);
        let curv = world.get_body_vel(id).lin.into();
        let v_des = pd_vel_to_target(pR, curv, tgtR, kp, kd, vmax);
        let mut v = world.get_body_vel(id);
        v.lin = vec3(v_des.x, v_des.y, v_des.z);
        world.set_body_vel(id, v);
    }

    // Optional: nudge pelvis forward a touch so CoM remains between feet (keeps Balance happy)
    {
        let id = riftphys_core::BodyId(pelvis);
        let mut v = world.get_body_vel(id);
        v.lin.x += spec.step_len * 0.2 * dt; // tiny forward bias
        world.set_body_vel(id, v);
    }
}
