use glam::{Mat3, Quat};
use riftphys_core::{Scalar, Vec3, Isometry, BodyId, JointId};
use riftphys_dynamics::Bodies;

/* ─────────────────────────  Distance Joint (keep yours) ───────────────────────── */

#[derive(Copy, Clone, Debug)]
pub struct DistanceJoint {
    pub a: BodyId,
    pub b: BodyId,
    pub rest: Scalar,
    pub compliance: Scalar,
}

/* ─────────────────────────  D6 types (translation + rotation rows) ────────────── */

#[derive(Copy, Clone, Debug, Default)]
pub struct Drive {
    pub target: Scalar,      // position target (m or rad) if vel_mode=false; velocity target if vel_mode=true
    pub kp: Scalar,          // “stiffness” (convert to XPBD bias via kd)
    pub kd: Scalar,          // “damping” (bias term)
    pub max_impulse: Scalar, // clamp per-row (optional; not used in this minimal pass)
    pub vel_mode: bool,      // true => velocity motor, false => position servo
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Limit { pub min: Scalar, pub max: Scalar }

#[derive(Copy, Clone, Debug)]
pub struct D6Axis {
    pub enabled: bool,
    pub compliance: Scalar,      // XPBD α (0 => rigid)
    pub drive: Option<Drive>,    // optional motor/servo
    pub limit: Option<Limit>,    // optional min/max
    pub lambda_acc: Scalar,      // warmstart accumulator
}
impl Default for D6Axis {
    fn default() -> Self {
        Self { enabled: false, compliance: 0.0, drive: None, limit: None, lambda_acc: 0.0 }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Generic6Dof {
    pub a: BodyId, pub b: BodyId,
    pub fa: Isometry, pub fb: Isometry,   // joint frames (local to A/B)
    pub t: [D6Axis; 3],                   // Tx, Ty, Tz
    pub r: [D6Axis; 3],                   // Rx, Ry, Rz
}

/* ─────────────────────────  Joint union + container ───────────────────────────── */

#[derive(Copy, Clone, Debug)]
pub enum JointKind { Distance(DistanceJoint), D6(Generic6Dof) }

#[derive(Default)]
pub struct Joints { kinds: Vec<JointKind> }

impl Joints {
    pub fn new() -> Self { Self { kinds: Vec::new() } }

    /* Distance */
    pub fn set_distance_params(&mut self, id: JointId, rest: f32, compliance: f32) {
        if let Some(JointKind::Distance(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.rest = rest; j.compliance = compliance;
        }
    }
    pub fn add_distance_joint(&mut self, a: BodyId, b: BodyId, rest: Scalar, compliance: Scalar) -> JointId {
        self.kinds.push(JointKind::Distance(DistanceJoint { a, b, rest, compliance }));
        JointId((self.kinds.len() as u32) - 1)
    }

    /* D6 + presets */
    pub fn add_d6(&mut self, j: Generic6Dof) -> JointId {
        self.kinds.push(JointKind::D6(j));
        JointId((self.kinds.len() as u32) - 1)
    }
    pub fn add_ball(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default() };
        for ax in 0..3 { j.t[ax].enabled = true; } // lock translation
        for ax in 0..3 { j.r[ax].enabled = true; } // all rotations enabled (limits can be set)
        self.add_d6(j)
    }
    /// hinge_axis = 0->Rx, 1->Ry, 2->Rz; locks the other two
    pub fn add_hinge(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, hinge_axis: usize) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default() };
        for ax in 0..3 { j.t[ax].enabled = true; }
        for ax in 0..3 { j.r[ax].enabled = ax == hinge_axis; }
        self.add_d6(j)
    }
    pub fn add_universal(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, ax0: usize, ax1: usize) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default() };
        for ax in 0..3 { j.t[ax].enabled = true; }
        j.r[ax0].enabled = true;
        j.r[ax1].enabled = true;
        self.add_d6(j)
    }

    /* D6 setters */
    pub fn set_d6_drive_t(&mut self, id: JointId, axis: usize, d: Option<Drive>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) { j.t[axis].drive = d; }
    }
    pub fn set_d6_drive_r(&mut self, id: JointId, axis: usize, d: Option<Drive>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) { j.r[axis].drive = d; }
    }
    pub fn set_d6_limit_t(&mut self, id: JointId, axis: usize, lim: Option<Limit>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) { j.t[axis].limit = lim; }
    }
    pub fn set_d6_limit_r(&mut self, id: JointId, axis: usize, lim: Option<Limit>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) { j.r[axis].limit = lim; }
    }

    /* Main solve (XPBD positional) */
    pub fn solve(&mut self, bodies: &mut Bodies, dt: Scalar, iterations: u32) {
        if self.kinds.is_empty() { return; }
        let alpha_dt2 = |c: Scalar| if c <= 0.0 { 0.0 } else { c / (dt * dt) };
        for _ in 0..iterations {
            for k in &mut self.kinds {
                match k {
                    JointKind::Distance(j) => solve_distance_row(bodies, j, dt),
                    JointKind::D6(j)       => solve_d6(bodies, j, dt, &alpha_dt2),
                }
            }
        }
    }
}

/* ─────────────────────────  Solvers  ───────────────────────── */

fn solve_distance_row(bodies: &mut Bodies, j: &DistanceJoint, dt: Scalar) {
    let ia = j.a.0; let ib = j.b.0;
    let wa = bodies.inv_mass_of(ia); let wb = bodies.inv_mass_of(ib);
    let w_sum = wa + wb; if w_sum == 0.0 { return; }

    let pa = bodies.pose(ia).pos; let pb = bodies.pose(ib).pos;
    let d  = pb - pa; let len = d.length(); if len <= 1.0e-6 { return; }

    let n = d / len;
    let c_val = len - j.rest;
    if c_val.abs() <= 1.0e-6 { return; }

    let alpha = if j.compliance <= 0.0 { 0.0 } else { j.compliance / (dt*dt) };
    let lambda = -c_val / (w_sum + alpha);

    let dp_a = -n * (lambda * wa);
    let dp_b =  n * (lambda * wb);
    bodies.apply_position_delta(ia, dp_a);
    bodies.apply_position_delta(ib, dp_b);
}

fn solve_d6(bodies: &mut Bodies, j: &mut Generic6Dof, dt: Scalar, alpha_dt2: &dyn Fn(Scalar)->Scalar) {
    // world joint frames
    let pa = bodies.pose(j.a.0); let pb = bodies.pose(j.b.0);
    let ra = pa.rot * j.fa.rot;  let rb = pb.rot * j.fb.rot;
    let xa = pa.pos + (pa.rot * j.fa.pos);
    let xb = pb.pos + (pb.rot * j.fb.pos);

    // joint axes in world (from A’s joint frame)
    let ax = ra * Vec3::X; let ay = ra * Vec3::Y; let az = ra * Vec3::Z;
    let axes = [ax, ay, az];

    let (wa, wb) = (bodies.inv_mass_of(j.a.0), bodies.inv_mass_of(j.b.0));
    let Ia = bodies.inv_inertia_world(j.a.0);
    let Ib = bodies.inv_inertia_world(j.b.0);

    /* Translation rows: (xb - xa) · u -> target (usually 0) */
    let d = xb - xa;
    for i in 0..3 {
        let row = &mut j.t[i]; if !row.enabled { continue; }
        let u = axes[i];
        let mut c_val = d.dot(u);
        let mut target = 0.0;
        if let Some(lim) = row.limit { c_val = c_val.clamp(lim.min, lim.max); target = c_val; }
        let mut bias = 0.0;
        if let Some(drive) = row.drive {
            if drive.vel_mode { bias += drive.target; } else { target = drive.target; }
        }
        let alpha = alpha_dt2(row.compliance);
        let ra_u = (xa - pa.pos).cross(u);
        let rb_u = (xb - pb.pos).cross(u);
        let w = wa + wb + ra_u.dot(Ia * ra_u) + rb_u.dot(Ib * rb_u) + alpha;
        if w <= 0.0 { continue; }
        let lambda = -((c_val - target) + bias * dt) / w;

        bodies.apply_position_delta(j.a.0, -u * (lambda * wa));
        bodies.apply_position_delta(j.b.0,  u * (lambda * wb));
        bodies.apply_orientation_delta(j.a.0, -(Ia * ra_u) * lambda);
        bodies.apply_orientation_delta(j.b.0,  (Ib * rb_u) * lambda);

        row.lambda_acc += lambda;
    }

    /* Rotation rows: align rb to ra with small-angle error */
    let q_err = ra.conjugate() * rb; // A->B in joint space
    let e = 2.0 * Vec3::new(q_err.x, q_err.y, q_err.z); // small-angle approx

    for i in 0..3 {
        let row = &mut j.r[i]; if !row.enabled { continue; }
        let u = axes[i];
        let mut c_val = e.dot(u);
        let mut target = 0.0;
        if let Some(lim) = row.limit { c_val = c_val.clamp(lim.min, lim.max); target = c_val; }
        let mut bias = 0.0;
        if let Some(drive) = row.drive {
            if drive.vel_mode { bias += drive.target; } else { target = drive.target; }
        }
        let alpha = alpha_dt2(row.compliance);
        let w = (u.dot(Ia * u) + u.dot(Ib * u)) + alpha;
        if w <= 0.0 { continue; }
        let lambda = -((c_val - target) + bias * dt) / w;

        bodies.apply_orientation_delta(j.a.0, -(Ia * u) * lambda);
        bodies.apply_orientation_delta(j.b.0,  (Ib * u) * lambda);

        row.lambda_acc += lambda;
    }
}

/* ─────────────────────────  Re-exports for world crate convenience ───────────── */

pub use D6Axis as D6AxisRow;
pub use Generic6Dof as D6Joint;
pub use Drive as D6Drive;
pub use Limit as D6Limit;
