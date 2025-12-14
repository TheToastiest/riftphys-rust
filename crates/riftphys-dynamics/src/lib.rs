use riftphys_core::types::{Isometry, Mat3, Velocity, Vec3};
use riftphys_core::{Quat, Scalar};
use glam::Vec3 as GVec3; // add near the top of riftphys-dynamics/src/lib.rs

const EPS_ANG2: f32 = 1.0e-24; // (1e-12)^2

/* ─────────────────────────  Quantization + sanitation  ───────────────────────── */

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
fn quat_canon(mut q: Quat) -> Quat {
    // Canonical sign for stable serialization/hashing.
    if q.w < 0.0 { q = -q; }
    q
}

#[inline]
fn q6q(q: Quat) -> Quat {
    // Quantize components, then renormalize + canonicalize.
    let mut qq = Quat::from_xyzw(q6(q.x), q6(q.y), q6(q.z), q6(q.w));
    // If quantization nuked it, fall back deterministically.
    if !qq.is_finite() { return Quat::IDENTITY; }
    qq = qq.normalize();
    quat_canon(qq)
}

#[inline]
fn safe_pos(v: Vec3) -> Vec3 { if v.is_finite() { q6v(v) } else { Vec3::ZERO } }

#[inline]
fn safe_vec(v: Vec3) -> Vec3 { if v.is_finite() { q6v(v) } else { Vec3::ZERO } }

#[inline]
fn safe_quat(q: Quat) -> Quat {
    if !q.is_finite() { return Quat::IDENTITY; }
    q6q(q)
}

/* ─────────────────────────  Bodies  ───────────────────────── */

/// Input descriptor when creating a body.
#[derive(Copy, Clone, Debug)]
pub struct BodyDesc {
    pub pose: Isometry,
    pub vel: Velocity,
    pub inv_mass: Scalar,
    pub dynamic: bool,
}

/// SoA body storage with deterministic ID = index semantics.
pub struct Bodies {
    pos: Vec<Vec3>,
    rot: Vec<Quat>,
    linvel: Vec<Vec3>,
    angvel: Vec<Vec3>,
    inv_mass: Vec<Scalar>,
    dynamic: Vec<bool>,
    /// Local-space inverse inertia tensor per body.
    inv_inertia_local: Vec<Mat3>,
}

impl Bodies {
    pub fn with_capacity(cap: usize) -> Self {
        Self {
            pos: Vec::with_capacity(cap),
            rot: Vec::with_capacity(cap),
            linvel: Vec::with_capacity(cap),
            angvel: Vec::with_capacity(cap),
            inv_mass: Vec::with_capacity(cap),
            dynamic: Vec::with_capacity(cap),
            inv_inertia_local: Vec::with_capacity(cap),
        }
    }

    #[inline]
    pub fn len(&self) -> usize { self.pos.len() }

    #[inline]
    fn idx(&self, id: u32) -> usize {
        let i = id as usize;
        debug_assert!(i < self.len());
        i
    }

    /// Legacy constructor used by earlier scaffold; safe to keep.
    pub fn add(&mut self, desc: BodyDesc) -> u32 {
        // Enforce invariants: non-dynamic or inv_mass<=0 => static.
        let is_dyn = desc.dynamic && desc.inv_mass.is_finite() && desc.inv_mass > 0.0;
        let inv_m = if is_dyn { q6(desc.inv_mass) } else { 0.0 };

        self.pos.push(safe_pos(desc.pose.pos));
        self.rot.push(safe_quat(desc.pose.rot));

        self.linvel.push(safe_vec(desc.vel.lin));
        self.angvel.push(safe_vec(desc.vel.ang));

        self.inv_mass.push(inv_m);
        self.dynamic.push(is_dyn);

        // Fallback: isotropic inverse inertia in LOCAL space.
        let inv_i = if inv_m > 0.0 {
            Mat3::from_diagonal(GVec3::splat(inv_m))
        } else {
            Mat3::ZERO
        };
        self.inv_inertia_local.push(inv_i);

        (self.pos.len() as u32) - 1
    }

    pub fn add_legacy(&mut self, pose: Isometry, vel: Velocity, inv_mass: Scalar, dynamic: bool) -> u32 {
        self.add(BodyDesc { pose, vel, inv_mass, dynamic })
    }

    // -------- Accessors used by world/solver/hash --------

    #[inline]
    pub fn pose(&self, id: u32) -> Isometry {
        let i = self.idx(id);
        Isometry { pos: self.pos[i], rot: self.rot[i] }
    }

    #[inline]
    pub fn set_pose(&mut self, id: u32, iso: Isometry) {
        let i = self.idx(id);
        if iso.pos.is_finite() { self.pos[i] = q6v(iso.pos); }
        if iso.rot.is_finite() { self.rot[i] = safe_quat(iso.rot); }
    }

    #[inline]
    pub fn vel(&self, id: u32) -> Velocity {
        let i = self.idx(id);
        Velocity { lin: self.linvel[i], ang: self.angvel[i] }
    }

    #[inline]
    pub fn set_vel(&mut self, id: u32, v: Velocity) {
        let i = self.idx(id);
        if v.lin.is_finite() { self.linvel[i] = q6v(v.lin); }
        if v.ang.is_finite() { self.angvel[i] = q6v(v.ang); }
    }

    #[inline] pub fn inv_mass_of(&self, id: u32) -> Scalar { self.inv_mass[self.idx(id)] }
    #[inline] pub fn is_dynamic(&self, id: u32) -> bool { self.dynamic[self.idx(id)] }

    #[inline]
    pub fn position(&self, id: u32) -> Vec3 { self.pos[self.idx(id)] }

    #[inline]
    pub fn rotation(&self, id: u32) -> Quat { self.rot[self.idx(id)] }

    // -------- Inertia helpers --------

    pub fn set_inv_inertia_local(&mut self, id: u32, inv_i_local: Mat3) {
        let i = self.idx(id);
        self.inv_inertia_local[i] = if inv_i_local.is_finite() { inv_i_local } else { Mat3::ZERO };
    }

    #[inline]
    pub fn inertia_inv_local(&self, id: u32) -> Mat3 {
        self.inv_inertia_local[self.idx(id)]
    }

    /// World-space inverse inertia: R * I^-1_local * R^T.
    pub fn inv_inertia_world(&self, id: u32) -> Mat3 {
        if self.inv_mass_of(id) == 0.0 { return Mat3::ZERO; }
        let i = self.idx(id);
        let r = self.rot[i];
        let rot_m = Mat3::from_quat(r);
        rot_m * self.inv_inertia_local[i] * rot_m.transpose()
    }


    // -------- Integration --------

    #[inline]
    fn integrate_rot(q: Quat, w_world: Vec3, dt: Scalar) -> Quat {
        // Deterministic, no trig:
        // qdot = 0.5 * Ω(w) * q   (world-frame angular velocity)
        let w2 = w_world.length_squared();
        if !w2.is_finite() || w2 <= EPS_ANG2 || !dt.is_finite() || dt == 0.0 {
            return q;
        }

        let omega = Quat::from_xyzw(w_world.x, w_world.y, w_world.z, 0.0);
        let qdot = omega * q;

        let s = 0.5 * dt;
        let qn = Quat::from_xyzw(
            q.x + qdot.x * s,
            q.y + qdot.y * s,
            q.z + qdot.z * s,
            q.w + qdot.w * s,
        );

        // Normalize + canonicalize + quantize for stable state.
        q6q(qn.normalize())
    }

    pub fn integrate_all(&mut self, gravity: Vec3, dt: Scalar) {
        let dt = q6(dt);
        if !dt.is_finite() || dt <= 0.0 { return; }

        let g = if gravity.is_finite() { q6v(gravity) } else { Vec3::ZERO };

        for i in 0..self.len() {
            if !self.dynamic[i] || self.inv_mass[i] == 0.0 { continue; }

            self.linvel[i] = q6v(self.linvel[i] + g * dt);
            self.pos[i]    = q6v(self.pos[i] + self.linvel[i] * dt);

            // Angular integration (solver updates angvel; we integrate orientation here).
            let q = self.rot[i];
            let w = q6v(self.angvel[i]);
            self.rot[i] = Self::integrate_rot(q, w, dt);
        }
    }

    // -------- Impulses / deltas (linear + angular) --------

    #[inline]
    pub fn apply_impulse(&mut self, id: u32, j: Vec3) {
        if !j.is_finite() { return; }
        let i = self.idx(id);
        let im = self.inv_mass[i];
        if im != 0.0 {
            self.linvel[i] = q6v(self.linvel[i] + q6v(j) * im);
        }
    }

    /// Apply linear+angular impulse at a world point.
    #[inline]
    pub fn apply_impulse_at_world_point(&mut self, id: u32, j: Vec3, p_world: Vec3) {
        if !(j.is_finite() && p_world.is_finite()) { return; }
        let i = self.idx(id);
        if self.inv_mass[i] == 0.0 { return; }

        self.apply_impulse(id, j);

        let r = q6v(p_world - self.pos[i]);
        let tau = q6v(r.cross(q6v(j)));
        self.apply_angular_impulse(id, tau);
    }

    /// Apply an angular impulse τ_impulse (world space): Δω = I^-1_world * τ.
    pub fn apply_angular_impulse(&mut self, id: u32, tau_impulse: Vec3) {
        if !tau_impulse.is_finite() { return; }
        let i = self.idx(id);
        if self.inv_mass[i] == 0.0 { return; }

        let inv_i_w = self.inv_inertia_world(id);
        let dw = inv_i_w * q6v(tau_impulse);
        if dw.is_finite() {
            self.angvel[i] = q6v(self.angvel[i] + q6v(dw));
        }
    }

    /// Add a position delta (already scaled for this body).
    #[inline]
    pub fn apply_position_delta(&mut self, id: u32, dp: Vec3) {
        if !dp.is_finite() { return; }
        let i = self.idx(id);
        if self.inv_mass[i] == 0.0 { return; } // do not drift statics
        self.pos[i] = q6v(self.pos[i] + q6v(dp));
    }

    /// Small-angle orientation correction (world space). Deterministic, stable.
    pub fn apply_orientation_delta(&mut self, id: u32, dtheta_world: Vec3) {
        if !dtheta_world.is_finite() { return; }
        let i = self.idx(id);
        if self.inv_mass[i] == 0.0 { return; } // do not drift statics

        let d = q6v(dtheta_world);
        let ang2 = d.length_squared();
        if !ang2.is_finite() || ang2 <= 0.0 { return; }

        let dq = Quat::from_xyzw(
            d.x * 0.5,
            d.y * 0.5,
            d.z * 0.5,
            1.0,
        ).normalize();

        self.rot[i] = q6q((dq * self.rot[i]).normalize());
    }

    // Iterator for hashing in stable order
    pub fn indices(&self) -> impl ExactSizeIterator<Item = u32> + '_ {
        0..(self.len() as u32)
    }
}

impl Default for Bodies {
    fn default() -> Self { Self::with_capacity(0) }
}
