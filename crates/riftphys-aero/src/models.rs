use glam::{Quat, Vec3};
use riftphys_core::StepCtx;
use riftphys_core::models::{AeroModel, AeroQuery};
use crate::ISA;

/// Simple quadratic drag: Fd = 0.5 * ρ * v² * Cd * A
pub struct FlatPlateDrag {
    pub cd: f32,
    pub area_m2: f32,
    pub isa: ISA,
}

impl AeroModel for FlatPlateDrag {
    fn accel_contrib(&self, _ctx: &StepCtx, q: AeroQuery) -> [f32; 3] {
        let rho = self.isa.density(q.altitude);
        let v   = Vec3::from_array(q.vel_world);
        let s   = v.length();
        if s <= 1e-6 || q.mass <= 0.0 { return [0.0; 3]; }
        let drag_dir = -v / s;
        let f_mag = 0.5 * rho * s * s * self.cd * self.area_m2;
        (drag_dir * (f_mag / q.mass)).to_array()
    }
}

/// Very simple lift model: CL(α) = cl_per_rad * clamp(α, ±stall)
/// Lift dir = body lift axis (e.g., +Z) rotated to world, orthogonalized to V.
pub struct SimpleWing {
    pub cl_per_rad: f32,
    pub stall_rad: f32,
    pub area_m2: f32,
    pub lift_dir_body: Vec3,
    pub isa: ISA,
    // NEW:
    pub cd0: f32,       // parasite base
    pub k_induced: f32, // induced drag coefficient
}

impl AeroModel for SimpleWing {
    fn accel_contrib(&self, _ctx: &StepCtx, q: AeroQuery) -> [f32;3] {
        let rho = self.isa.density(q.altitude);
        let v   = Vec3::from_array(q.vel_world);
        let v2  = v.length_squared();
        if v2 <= 1e-8 || q.mass <= 0.0 { return [0.0;3]; }

        let alpha = q.alpha_rad.clamp(-self.stall_rad, self.stall_rad);
        let cl = self.cl_per_rad * alpha;
        let cd = self.cd0 + self.k_induced * cl * cl;

        // lift direction
        let vhat = v.normalize();
        let lift_world_nominal = q.orientation * self.lift_dir_body;
        let mut lift_dir = lift_world_nominal - vhat * lift_world_nominal.dot(vhat);
        let len = lift_dir.length(); if len <= 1e-6 { return [0.0;3]; }
        lift_dir /= len;

        // aerodynamic magnitudes
        let qdyn = 0.5 * rho * v2;
        let l = qdyn * cl * self.area_m2;
        let d = qdyn * cd * self.area_m2;

        let a_lift = (lift_dir * (l / q.mass));
        let a_drag = (-vhat     * (d / q.mass));
        (a_lift + a_drag).to_array()
    }
}