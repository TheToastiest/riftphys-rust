use riftphys_core::types::{Isometry, Velocity, Vec3};
use riftphys_core::{Scalar, Quat};
use glam::{Mat3, Vec3 as GVec3};

/// Input descriptor when creating a body.
#[derive(Copy, Clone, Debug)]
pub struct BodyDesc {
    pub pose: Isometry,
    pub vel: Velocity,
    pub inv_mass: Scalar,
    pub dynamic: bool,
    // NOTE: keep this minimal legacy surface. We infer a default inv-inertia; you can
    //       override later with set_inv_inertia_local(..) once you compute true inertia.
}

/// SoA body storage with deterministic ID = index semantics.
pub struct Bodies {
    pos: Vec<Vec3>,
    rot: Vec<Quat>,
    linvel: Vec<Vec3>,
    angvel: Vec<Vec3>,
    inv_mass: Vec<Scalar>,
    dynamic: Vec<bool>,
    // NEW: local-space inverse inertia tensor per body (fallback: isotropic inv_mass * I)
    inv_inertia_local: Vec<Mat3>,
}

impl Bodies {
    pub fn with_capacity(cap: usize) -> Self {
        Self {
            pos:     Vec::with_capacity(cap),
            rot:     Vec::with_capacity(cap),
            linvel:  Vec::with_capacity(cap),
            angvel:  Vec::with_capacity(cap),
            inv_mass: Vec::with_capacity(cap),
            dynamic: Vec::with_capacity(cap),
            inv_inertia_local: Vec::with_capacity(cap), // NEW
        }
    }

    /// Legacy constructor used by earlier scaffold; safe to keep.
    pub fn add(&mut self, desc: BodyDesc) -> u32 {
        self.pos.push(desc.pose.pos);
        self.rot.push(desc.pose.rot);
        self.linvel.push(desc.vel.lin);
        self.angvel.push(desc.vel.ang);
        self.inv_mass.push(desc.inv_mass);
        self.dynamic.push(desc.dynamic);

        // Fallback: isotropic inverse inertia in LOCAL space.
        // You can override per-body later via set_inv_inertia_local(..).
        let inv_i = if desc.inv_mass > 0.0 {
            Mat3::from_diagonal(GVec3::splat(desc.inv_mass))
        } else {
            Mat3::ZERO
        };

        self.inv_inertia_local.push(inv_i);

        (self.pos.len() as u32) - 1
    }

    /// Convenience to preserve older call sites that created a "Body" directly.
    pub fn add_legacy(&mut self, pose: Isometry, vel: Velocity, inv_mass: Scalar, dynamic: bool) -> u32 {
        self.add(BodyDesc { pose, vel, inv_mass, dynamic })
    }

    #[inline] pub fn len(&self) -> usize { self.pos.len() }

    pub fn integrate_all(&mut self, gravity: Vec3, dt: Scalar) {
        for i in 0..self.len() {
            if !self.dynamic[i] || self.inv_mass[i] == 0.0 { continue; }
            self.linvel[i] += gravity * dt;
            self.pos[i]    += self.linvel[i] * dt;
            // (Angular integration still explicit via joints/contacts)
        }
    }

    // -------- Accessors used by world/solver/hash --------
    #[inline] pub fn pose(&self, id: u32) -> Isometry {
        let i = id as usize;
        Isometry { pos: self.pos[i], rot: self.rot[i] }
    }
    #[inline] pub fn set_pose(&mut self, id: u32, iso: Isometry) {
        let i = id as usize;
        self.pos[i] = iso.pos;
        self.rot[i] = iso.rot;
    }

    #[inline] pub fn vel(&self, id: u32) -> Velocity {
        let i = id as usize;
        Velocity { lin: self.linvel[i], ang: self.angvel[i] }
    }
    #[inline] pub fn set_vel(&mut self, id: u32, v: Velocity) {
        let i = id as usize;
        self.linvel[i] = v.lin;
        self.angvel[i] = v.ang;
    }

    #[inline] pub fn inv_mass_of(&self, id: u32) -> Scalar { self.inv_mass[id as usize] }
    #[inline] pub fn is_dynamic(&self, id: u32) -> bool { self.dynamic[id as usize] }

    // -------- Inertia helpers (NEW) --------
    /// Set local-space inverse inertia tensor for body `id`. Deterministic.
    pub fn set_inv_inertia_local(&mut self, id: u32, inv_i_local: Mat3) {
        let i = id as usize;
        self.inv_inertia_local[i] = inv_i_local;
    }
    /// Get local-space inverse inertia tensor.
    #[inline] pub fn inertia_inv_local(&self, id: u32) -> Mat3 {
        self.inv_inertia_local[id as usize]
    }
    /// World-space inverse inertia: R * I^-1_local * R^T.
    pub fn inv_inertia_world(&self, id: u32) -> Mat3 {
        if self.inv_mass_of(id) == 0.0 { return Mat3::ZERO; }
        let r = self.rot[id as usize];
        let R = Mat3::from_quat(r);
        R * self.inv_inertia_local[id as usize] * R.transpose()
    }

    // -------- Impulses / deltas (linear + angular) --------
    #[inline] pub fn apply_impulse(&mut self, id: u32, j: Vec3) {
        let i = id as usize;
        let im = self.inv_mass[i];
        if im != 0.0 { self.linvel[i] += j * im; }
    }

    /// Add a position delta (already scaled for this body).
    #[inline] pub fn apply_position_delta(&mut self, id: u32, dp: Vec3) {
        let i = id as usize;
        self.pos[i] += dp;
    }

    /// Apply an angular impulse τ_impulse (world space): Δω = I^-1_world * τ.
    pub fn apply_angular_impulse(&mut self, id: u32, tau_impulse: Vec3) {
        let i = id as usize;
        if self.inv_mass[i] == 0.0 { return; }
        let inv_i_w = self.inv_inertia_world(id);
        self.angvel[i] += inv_i_w * tau_impulse;
    }

    /// Small-angle orientation correction (world space). Deterministic, stable.
    pub fn apply_orientation_delta(&mut self, id: u32, dtheta_world: Vec3) {
        let i = id as usize;
        let ang2 = dtheta_world.length_squared();
        if ang2 <= 0.0 { return; }
        // Small-angle quaternion: (v*0.5, 1) normalized.
        let dq = Quat::from_xyzw(dtheta_world.x * 0.5, dtheta_world.y * 0.5, dtheta_world.z * 0.5, 1.0).normalize();
        self.rot[i] = (dq * self.rot[i]).normalize();
    }

    // Iterator for hashing in stable order
    pub fn indices(&self) -> impl ExactSizeIterator<Item=u32> + '_ {
        0..(self.len() as u32)
    }
}

impl Default for Bodies {
    fn default() -> Self { Self::with_capacity(0) }
}
