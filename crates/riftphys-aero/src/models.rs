use riftphys_core::{Quat, Vec3, StepCtx};
use riftphys_core::models::{AeroModel, AeroQuery};

use crate::ISA;

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
    let inv_len = 1.0 / len2.sqrt();
    q6v(v * inv_len)
}

/// Simple quadratic drag: Fd = 0.5 * ρ * v² * Cd * A
#[derive(Copy, Clone, Debug)]
pub struct FlatPlateDrag {
    pub cd: f32,
    pub area_m2: f32,
    pub isa: ISA,
}

impl AeroModel for FlatPlateDrag {
    fn accel_contrib(&self, _ctx: &StepCtx, q: AeroQuery) -> [f32; 3] {
        let mass = q6(q.mass);
        if !(mass > 0.0) {
            return [0.0; 3];
        }

        let rho = q6(self.isa.density(q.altitude));
        if rho <= 0.0 {
            return [0.0; 3];
        }

        let v = Vec3::from_array(q.vel_world);
        let v2 = v.length_squared();
        if !v2.is_finite() || v2 <= 1.0e-20 {
            return [0.0; 3];
        }

        let speed = v2.sqrt();
        if speed <= 1.0e-6 {
            return [0.0; 3];
        }

        let vhat = normalize_q6(v / speed);
        if vhat == Vec3::ZERO {
            return [0.0; 3];
        }

        let cd = q6(self.cd).max(0.0);
        let area = q6(self.area_m2).max(0.0);
        if area <= 0.0 {
            return [0.0; 3];
        }

        // F = 0.5*rho*v^2*Cd*A
        let f_mag = q6(0.5 * rho * q6(speed * speed) * cd * area);
        let a = q6v((-vhat) * q6(f_mag / mass));
        [a.x, a.y, a.z]
    }
}

/// Very simple lift model:
/// - CL(α) = cl_per_rad * clamp(α, ±stall)
/// - CD = cd0 + k_induced * CL^2
/// - Lift dir = (orientation * lift_dir_body), orthogonalized against velocity.
#[derive(Copy, Clone, Debug)]
pub struct SimpleWing {
    pub cl_per_rad: f32,
    pub stall_rad: f32,
    pub area_m2: f32,
    pub lift_dir_body: Vec3,
    pub isa: ISA,

    pub cd0: f32,
    pub k_induced: f32,
}

impl AeroModel for SimpleWing {
    fn accel_contrib(&self, _ctx: &StepCtx, q: AeroQuery) -> [f32; 3] {
        let mass = q6(q.mass);
        if !(mass > 0.0) {
            return [0.0; 3];
        }

        let rho = q6(self.isa.density(q.altitude));
        if rho <= 0.0 {
            return [0.0; 3];
        }

        let v = Vec3::from_array(q.vel_world);
        let v2 = v.length_squared();
        if !v2.is_finite() || v2 <= 1.0e-20 {
            return [0.0; 3];
        }

        let speed = v2.sqrt();
        if speed <= 1.0e-6 {
            return [0.0; 3];
        }

        let vhat = normalize_q6(v / speed);
        if vhat == Vec3::ZERO {
            return [0.0; 3];
        }

        // Coefficients
        let stall = q6(self.stall_rad).abs().max(1.0e-6);
        let alpha = q6(q.alpha_rad).clamp(-stall, stall);

        let cl_per = q6(self.cl_per_rad);
        let cl = q6(cl_per * alpha);

        let cd0 = q6(self.cd0).max(0.0);
        let k   = q6(self.k_induced).max(0.0);
        let cd  = q6(cd0 + q6(k * cl * cl));

        let area = q6(self.area_m2).max(0.0);
        if area <= 0.0 {
            return [0.0; 3];
        }

        // Orientation: core Quat alias (glam::Quat under the hood)
        let ori: Quat = q.orientation;

        // Rotate lift axis to world using a method that works with Vec3A.
        let lift_world_nominal = q6v(ori.mul_vec3a(self.lift_dir_body));

        // Orthogonalize against velocity.
        let proj = q6(lift_world_nominal.dot(vhat));
        let mut lift_dir = q6v(lift_world_nominal - vhat * proj);

        let lift_len2 = lift_dir.length_squared();
        if !lift_len2.is_finite() || lift_len2 <= 1.0e-20 {
            return [0.0; 3];
        }
        lift_dir = normalize_q6(lift_dir);
        if lift_dir == Vec3::ZERO {
            return [0.0; 3];
        }

        // qdyn = 0.5 * rho * v^2
        let qdyn = q6(0.5 * rho * q6(speed * speed));
        let l = q6(qdyn * cl * area);
        let d = q6(qdyn * cd * area);

        let a_lift = q6v(lift_dir * q6(l / mass));
        let a_drag = q6v((-vhat)   * q6(d / mass));

        let a = q6v(a_lift + a_drag);
        [a.x, a.y, a.z]
    }
}
