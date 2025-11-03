use riftphys_core::types::{Isometry, Velocity, Vec3};
use riftphys_core::{Scalar, Quat};

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
            // (Angular integration deferred to Phase 2.5 / joints work)
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

    // -------- Helpers the solver uses --------
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

    // Iterator for hashing in stable order
    pub fn indices(&self) -> impl ExactSizeIterator<Item=u32> + '_ {
        0..(self.len() as u32)
    }
}

impl Default for Bodies {
    fn default() -> Self { Self::with_capacity(0) }
}
