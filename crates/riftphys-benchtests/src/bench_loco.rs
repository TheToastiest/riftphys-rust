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
    let kp = 8.0;  // m/s per meter error
    let kd = 1.6;  // da    mping
    let vmax = 4.0; // clamp

    // Left foot
    {
        let id = riftphys_core::BodyId(left);
        let curv = world.get_body_vel(id).lin.into();
        let mut v_des = pd_vel_to_target(pL, curv, tgtL, kp, kd, vmax);
        if matches!(phaseL, loco::Phase::Stance(_)) { v_des.y = 0.0; }    // plant Y during stance
        let mut v = world.get_body_vel(id);
        v.lin = vec3(v_des.x, v_des.y, v_des.z);
        world.set_body_vel(id, v);
    }
    // Right foot
    {
        let id = riftphys_core::BodyId(right);
        let curv = world.get_body_vel(id).lin.into();
        let mut v_des = pd_vel_to_target(pR, curv, tgtR, kp, kd, vmax);
        if matches!(phaseR, loco::Phase::Stance(_)) { v_des.y = 0.0; }
        let mut v = world.get_body_vel(id);
        v.lin = vec3(v_des.x, v_des.y, v_des.z);
        world.set_body_vel(id, v);
    }

    // Optional: nudge pelvis forward a touch so CoM remains between feet (keeps Balance happy)
    {
        let id = riftphys_core::BodyId(pelvis);
        let mut v = world.get_body_vel(id);
        // v.lin.x += spec.step_len * 1.0 * dt; // tiny forward bias
        world.set_body_vel(id, v);
    }

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
    {
        use riftphys_core::BodyId;
        let id  = BodyId(pelvis);
        let pos = world.get_body_pose(id).pos;
        let mut vel = world.get_body_vel(id);

        // Choose a target COM height (your initial was ~1.20).
        let y_tgt = 1.20_f32;

        // Simple PD on height (units: velocity, applied incrementally)
        let kp_y = 12.0; // m/s per m of height error
        let kd_y = 2.5;  // damping on current vertical speed
        let v_y_des = (y_tgt - pos.y) * kp_y - vel.lin.y * kd_y;

        // Apply as an incremental “accel-style” velocity change
        vel.lin.y += v_y_des * dt;
        world.set_body_vel(id, vel);
    }

    use riftphys_core::BodyId;
    use glam::Vec3;
    // normalize / default heading to +X for safety
    let mut h = heading_dir_ws;
    if h.length_squared() < 1.0e-12 { h = Vec3::new(1.0, 0.0, 0.0); } else { h = h.normalize(); }

    let pL = world.get_body_pose(BodyId(left)).pos.into();
    let pR = world.get_body_pose(BodyId(right)).pos.into();

    let phaseL = clocks.0.step(dt);
    let phaseR = clocks.1.step(dt);

    let mut tgtL = pL;
    let mut tgtR = pR;

    match phaseL {
        loco::Phase::Swing(s) => {
            tgtL = loco::swing_target_dir(pL, h, left_len,  spec.step_h, s);
        }
        _ => {}
    }
    match phaseR {
        loco::Phase::Swing(s) => {
            tgtR = loco::swing_target_dir(pR, h, right_len, spec.step_h, s);
        }
        _ => {}
    }

    let kp = 8.0;   // was 6.0
    let kd = 1.6;   // was 1.2
    let vmax = 4.0; // was 2.5

    let idL = BodyId(left);
    let vL  = world.get_body_vel(idL).lin.into();
    let vdL = pd_vel_to_target(pL, vL, tgtL, kp, kd, vmax);
    let mut nvL = world.get_body_vel(idL);
    let mut vdL2 = vdL;
    if matches!(phaseL, loco::Phase::Stance(_)) { vdL2.y = 0.0; }   // keep planted
    nvL.lin = vec3(vdL2.x, vdL2.y, vdL2.z);

    world.set_body_vel(idL, nvL);

    let idR = BodyId(right);
    let vR  = world.get_body_vel(idR).lin.into();
    let vdR = pd_vel_to_target(pR, vR, tgtR, kp, kd, vmax);

    let mut nvR = world.get_body_vel(idR);
    let mut vdR2 = vdR;
    if matches!(phaseR, loco::Phase::Stance(_)) { vdR2.y = 0.0; }
    nvR.lin = vec3(vdR2.x, vdR2.y, vdR2.z);
    world.set_body_vel(idR, nvR);

    // same tiny pelvis bias  -> REPLACE with heading-aware speed target
    {
        use riftphys_core::BodyId;
        let id = BodyId(pelvis);
        let mut v = world.get_body_vel(id);

        // desired forward speed from gait cadence
        // desired forward speed from cadence
        let period = (spec.stance_dur + spec.swing_dur).max(1e-6);
        let v_forward = (left_len.max(right_len) / period);

        // world forward along heading
        let fwd = vec3(h.x, 0.0, h.z);
        let v_des = fwd * v_forward;

        // hard-set pelvis horizontal velocity (keep Y from balance/contacts)
        let id = BodyId(pelvis);
        let mut v = world.get_body_vel(id);
        v.lin.x = v_des.x;
        v.lin.z = v_des.z;
        world.set_body_vel(id, v);
    }

}
