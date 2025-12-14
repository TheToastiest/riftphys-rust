use riftphys_core::{BodyId, Isometry, JointId, Mat3, Quat, Scalar, Vec3};
use riftphys_dynamics::Bodies;

/* ─────────────────────────  Quantization ───────────────────────── */

#[inline]
fn q6(x: f32) -> f32 {
    if !x.is_finite() { return 0.0; }
    (x * 1.0e6_f32).round() * 1.0e-6_f32
}

#[inline]
fn q6v(v: Vec3) -> Vec3 {
    Vec3::new(q6(v.x), q6(v.y), q6(v.z))
}

#[inline]
fn normalize_q6(v: Vec3) -> Vec3 {
    let v = q6v(v);
    let len2 = v.length_squared();
    if !len2.is_finite() || len2 <= 1.0e-20 {
        return Vec3::ZERO;
    }
    q6v(v * (1.0 / len2.sqrt()))
}

/* ─────────────────────────  Distance Joint (kept, hardened) ─────────────────── */

#[derive(Copy, Clone, Debug)]
pub struct DistanceJoint {
    pub a: BodyId,
    pub b: BodyId,
    pub rest: Scalar,
    pub compliance: Scalar,
}

/* ─────────────────────────  D6 types (translation + rotation rows) ──────────── */

#[derive(Copy, Clone, Debug, Default)]
pub struct Drive {
    pub target: Scalar,      // position target (m or rad) if vel_mode=false; velocity target if vel_mode=true (bias)
    pub kp: Scalar,          // currently unused in this minimal XPBD pass
    pub kd: Scalar,          // currently unused in this minimal XPBD pass
    pub max_impulse: Scalar, // currently unused in this minimal XPBD pass
    pub vel_mode: bool,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Limit {
    pub min: Scalar,
    pub max: Scalar,
}

#[derive(Copy, Clone, Debug)]
pub struct D6Axis {
    pub enabled: bool,           // "row active"
    pub compliance: Scalar,      // XPBD compliance (0 => rigid)
    pub drive: Option<Drive>,    // optional motor/servo
    pub limit: Option<Limit>,    // optional min/max
    pub lambda_acc: Scalar,      // XPBD accumulated lambda (warmstart + compliance term)
}

impl Default for D6Axis {
    fn default() -> Self {
        Self {
            enabled: false,
            compliance: 0.0,
            drive: None,
            limit: None,
            lambda_acc: 0.0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Generic6Dof {
    pub a: BodyId,
    pub b: BodyId,
    pub fa: Isometry,
    pub fb: Isometry,
    pub t: [D6Axis; 3], // Tx, Ty, Tz
    pub r: [D6Axis; 3], // Rx, Ry, Rz
}

/* ─────────────────────────  Joint union + container ─────────────────────────── */

#[derive(Copy, Clone, Debug)]
pub enum JointKind {
    Distance(DistanceJoint),
    D6(Generic6Dof),
}

#[derive(Default)]
pub struct Joints {
    kinds: Vec<JointKind>,
}

impl Joints {
    pub fn new() -> Self {
        Self { kinds: Vec::new() }
    }

    /* Distance */

    pub fn set_distance_params(&mut self, id: JointId, rest: f32, compliance: f32) {
        if let Some(JointKind::Distance(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.rest = q6(rest);
            j.compliance = q6(compliance).max(0.0);
        }
    }

    pub fn add_distance_joint(&mut self, a: BodyId, b: BodyId, rest: Scalar, compliance: Scalar) -> JointId {
        self.kinds.push(JointKind::Distance(DistanceJoint {
            a,
            b,
            rest: q6(rest),
            compliance: q6(compliance).max(0.0),
        }));
        JointId((self.kinds.len() as u32) - 1)
    }

    /* D6 + presets */

    pub fn add_d6(&mut self, j: Generic6Dof) -> JointId {
        self.kinds.push(JointKind::D6(j));
        JointId((self.kinds.len() as u32) - 1)
    }

    /// Ball joint: lock translation, leave rotation free by default.
    /// (You can enable r-axes later if you add limits/drives.)
    pub fn add_ball(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default() };
        for ax in 0..3 { j.t[ax].enabled = true; }  // lock Tx/Ty/Tz to 0
        for ax in 0..3 { j.r[ax].enabled = false; } // free rotations
        self.add_d6(j)
    }

    /// Hinge: lock translation, lock two rotation axes, leave hinge axis free unless you set a limit/drive.
    /// hinge_axis = 0->Rx, 1->Ry, 2->Rz
    pub fn add_hinge(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, hinge_axis: usize) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default() };
        for ax in 0..3 { j.t[ax].enabled = true; }
        for ax in 0..3 { j.r[ax].enabled = ax != hinge_axis; } // lock non-hinge axes
        // hinge axis stays free unless you later enable it with a limit/drive
        self.add_d6(j)
    }

    /// Universal: lock translation, lock exactly one rotation axis (the one not in ax0/ax1).
    pub fn add_universal(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, ax0: usize, ax1: usize) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default() };
        for ax in 0..3 { j.t[ax].enabled = true; }
        for ax in 0..3 { j.r[ax].enabled = !(ax == ax0 || ax == ax1); } // lock the remaining axis
        self.add_d6(j)
    }

    /* D6 setters (setting a drive/limit auto-activates that row) */

    pub fn set_d6_drive_t(&mut self, id: JointId, axis: usize, d: Option<Drive>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.t[axis].drive = d;
            if j.t[axis].drive.is_some() { j.t[axis].enabled = true; }
        }
    }

    pub fn set_d6_drive_r(&mut self, id: JointId, axis: usize, d: Option<Drive>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.r[axis].drive = d;
            if j.r[axis].drive.is_some() { j.r[axis].enabled = true; }
        }
    }

    pub fn set_d6_limit_t(&mut self, id: JointId, axis: usize, lim: Option<Limit>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.t[axis].limit = lim;
            if j.t[axis].limit.is_some() { j.t[axis].enabled = true; }
        }
    }

    pub fn set_d6_limit_r(&mut self, id: JointId, axis: usize, lim: Option<Limit>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.r[axis].limit = lim;
            if j.r[axis].limit.is_some() { j.r[axis].enabled = true; }
        }
    }

    /* Main solve (XPBD positional) */

    pub fn solve(&mut self, bodies: &mut Bodies, dt: Scalar, iterations: u32) {
        if self.kinds.is_empty() { return; }

        let dt = q6(dt);
        if !(dt > 0.0) { return; }
        let dt2 = q6((dt * dt).max(1.0e-12));

        #[inline]
        fn alpha_dt2(c: Scalar, dt2: Scalar) -> Scalar {
            let c = q6(c).max(0.0);
            if c <= 0.0 { 0.0 } else { q6(c / dt2) }
        }

        for _ in 0..iterations.max(1) {
            for k in &mut self.kinds {
                match k {
                    JointKind::Distance(j) => solve_distance_row(bodies, j, dt, dt2),
                    JointKind::D6(j)       => solve_d6(bodies, j, dt, dt2, alpha_dt2),
                }
            }
        }
    }
}

/* ─────────────────────────  Solvers  ───────────────────────── */

fn solve_distance_row(bodies: &mut Bodies, j: &DistanceJoint, dt: Scalar, dt2: Scalar) {
    let ia = j.a.0;
    let ib = j.b.0;

    let wa = bodies.inv_mass_of(ia);
    let wb = bodies.inv_mass_of(ib);
    let w_sum = q6(wa + wb);
    if w_sum <= 0.0 { return; }

    let pa = bodies.pose(ia).pos;
    let pb = bodies.pose(ib).pos;

    let d = pb - pa;
    let len2 = d.length_squared();
    if !len2.is_finite() || len2 <= 1.0e-20 { return; }

    let len = len2.sqrt();
    if len <= 1.0e-6 { return; }

    let n = normalize_q6(d / len);
    if n == Vec3::ZERO { return; }

    let c_val = q6(len - q6(j.rest));
    if c_val.abs() <= 1.0e-6 { return; }

    let alpha = if j.compliance <= 0.0 { 0.0 } else { q6(q6(j.compliance).max(0.0) / dt2) };
    let denom = q6(w_sum + alpha);
    if denom <= 0.0 { return; }

    // Minimal XPBD (no lambda accumulator for DistanceJoint to keep struct unchanged)
    let lambda = q6(-c_val / denom);

    bodies.apply_position_delta(ia, q6v(-n * q6(lambda * wa)));
    bodies.apply_position_delta(ib, q6v( n * q6(lambda * wb)));

    let _ = dt; // keep signature stable; dt used via dt2 already
}

fn solve_d6(
    bodies: &mut Bodies,
    j: &mut Generic6Dof,
    dt: Scalar,
    dt2: Scalar,
    alpha_dt2: fn(Scalar, Scalar) -> Scalar,
) {
    // World poses
    let pa = bodies.pose(j.a.0);
    let pb = bodies.pose(j.b.0);

    // World joint frames
    let ra: Quat = pa.rot * j.fa.rot;
    let rb: Quat = pb.rot * j.fb.rot;

    // Joint anchor points in world
    let xa = q6v(pa.pos + pa.rot.mul_vec3a(j.fa.pos));
    let xb = q6v(pb.pos + pb.rot.mul_vec3a(j.fb.pos));

    // Axes in world from A joint frame (quantized + normalized)
    let ax = normalize_q6(ra.mul_vec3a(Vec3::X));
    let ay = normalize_q6(ra.mul_vec3a(Vec3::Y));
    let az = normalize_q6(ra.mul_vec3a(Vec3::Z));
    let axes = [ax, ay, az];

    let wa = q6(bodies.inv_mass_of(j.a.0));
    let wb = q6(bodies.inv_mass_of(j.b.0));

    let iaw: Mat3 = bodies.inv_inertia_world(j.a.0);
    let ibw: Mat3 = bodies.inv_inertia_world(j.b.0);

    // Translation rows: coordinate along axis = (xb - xa)·u
    let d = q6v(xb - xa);

    for i in 0..3 {
        let row = &mut j.t[i];
        if !row.enabled { continue; }

        let u = axes[i];
        if u == Vec3::ZERO { continue; }

        let coord = q6(d.dot(u));

        // Determine constraint error C and target for this row
        let mut target_pos = 0.0f32;
        let mut c_err: Option<f32> = None;

        if let Some(lim) = row.limit {
            let lo = q6(lim.min);
            let hi = q6(lim.max);
            if coord < lo {
                c_err = Some(q6(coord - lo)); // below min -> push up to lo
                target_pos = lo;
            } else if coord > hi {
                c_err = Some(q6(coord - hi)); // above max -> push down to hi
                target_pos = hi;
            } else {
                // within limits: only act if there's a drive (servo)
                c_err = None;
                target_pos = coord;
            }
        }

        if let Some(drive) = row.drive {
            if !drive.vel_mode {
                target_pos = q6(drive.target);
                if let Some(lim) = row.limit {
                    target_pos = target_pos.clamp(q6(lim.min), q6(lim.max));
                }
                c_err = Some(q6(coord - target_pos));
            } else {
                // velocity-mode in a positional XPBD solve: treat as a bias term (hack but deterministic)
                if c_err.is_none() {
                    c_err = Some(q6(coord - 0.0));
                    target_pos = 0.0;
                }
            }
        }

        // If we’re within limits and no drive, skip the row
        let Some(mut c) = c_err else { continue; };

        // Velocity-bias term (only used for vel_mode drives)
        let mut bias_vel = 0.0f32;
        if let Some(drive) = row.drive {
            if drive.vel_mode {
                bias_vel = q6(drive.target);
            }
        }

        // Jacobians
        let ra_u = q6v((xa - pa.pos).cross(u));
        let rb_u = q6v((xb - pb.pos).cross(u));

        // Effective mass
        let w_rot_a = q6(ra_u.dot(iaw * ra_u));
        let w_rot_b = q6(rb_u.dot(ibw * rb_u));
        let w = q6(wa + wb + w_rot_a + w_rot_b);

        let alpha = alpha_dt2(row.compliance, dt2);
        let denom = q6(w + alpha);
        if denom <= 0.0 { continue; }

        // XPBD delta_lambda
        // C_eff = C + bias*dt
        c = q6(c + q6(bias_vel * dt));
        let lambda_old = q6(row.lambda_acc);
        let dl = q6(-(c + q6(alpha * lambda_old)) / denom);
        row.lambda_acc = q6(lambda_old + dl);

        // Apply corrections with delta lambda
        bodies.apply_position_delta(j.a.0, q6v(-u * q6(dl * wa)));
        bodies.apply_position_delta(j.b.0, q6v( u * q6(dl * wb)));

        bodies.apply_orientation_delta(j.a.0, q6v(-(iaw * ra_u) * dl));
        bodies.apply_orientation_delta(j.b.0, q6v( (ibw * rb_u) * dl));

        let _ = target_pos; // kept to make intent obvious; used via c_err in servo path
    }

    // Rotation rows: small-angle error from ra to rb, projected on u
    let mut q_err = ra.conjugate() * rb; // A->B in joint space
    if q_err.w < 0.0 { q_err = -q_err; } // canonicalize sign

    let e = q6v(2.0 * Vec3::new(q_err.x, q_err.y, q_err.z));

    for i in 0..3 {
        let row = &mut j.r[i];
        if !row.enabled { continue; }

        let u = axes[i];
        if u == Vec3::ZERO { continue; }

        let angle = q6(e.dot(u));

        let mut target_ang = 0.0f32;
        let mut c_err: Option<f32> = Some(q6(angle - 0.0)); // default lock-to-zero

        if let Some(lim) = row.limit {
            let lo = q6(lim.min);
            let hi = q6(lim.max);
            if angle < lo {
                c_err = Some(q6(angle - lo));
                target_ang = lo;
            } else if angle > hi {
                c_err = Some(q6(angle - hi));
                target_ang = hi;
            } else {
                c_err = None; // within range => no correction unless drive
                target_ang = angle;
            }
        }

        if let Some(drive) = row.drive {
            if !drive.vel_mode {
                target_ang = q6(drive.target);
                if let Some(lim) = row.limit {
                    target_ang = target_ang.clamp(q6(lim.min), q6(lim.max));
                }
                c_err = Some(q6(angle - target_ang));
            } else {
                if c_err.is_none() {
                    c_err = Some(q6(angle - 0.0));
                    target_ang = 0.0;
                }
            }
        }

        let Some(mut c) = c_err else { continue; };

        let mut bias_vel = 0.0f32;
        if let Some(drive) = row.drive {
            if drive.vel_mode {
                bias_vel = q6(drive.target);
            }
        }

        // Effective mass for angular row
        let w = q6(u.dot(iaw * u) + u.dot(ibw * u));
        let alpha = alpha_dt2(row.compliance, dt2);
        let denom = q6(w + alpha);
        if denom <= 0.0 { continue; }

        c = q6(c + q6(bias_vel * dt));

        let lambda_old = q6(row.lambda_acc);
        let dl = q6(-(c + q6(alpha * lambda_old)) / denom);
        row.lambda_acc = q6(lambda_old + dl);

        bodies.apply_orientation_delta(j.a.0, q6v(-(iaw * u) * dl));
        bodies.apply_orientation_delta(j.b.0, q6v( (ibw * u) * dl));

        let _ = target_ang;
    }
}

/* ─────────────────────────  Re-exports for world crate convenience ───────────── */

pub use D6Axis as D6AxisRow;
pub use Generic6Dof as D6Joint;
pub use Drive as D6Drive;
pub use Limit as D6Limit;
