// riftphys-accel/src/propulsion.rs
use glam::Vec3;
use riftphys_core::{StepCtx};
use riftphys_core::models::{PropulsionModel, PropQuery};

pub enum ThrottleCurve { Linear, QuadEaseIn, Custom([f32; 5]) /* poly up to 4th */ }

fn eval_curve(c: &ThrottleCurve, t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    match c {
        ThrottleCurve::Linear => t,
        ThrottleCurve::QuadEaseIn => t*t,
        ThrottleCurve::Custom(p) => { // Horner
            (((p[4]*t + p[3])*t + p[2])*t + p[1])*t + p[0]
        }
    }
}

pub struct SimplePropulsion {
    pub thrust_constant_n: f32,     // baseline thrust
    pub thrust_max_n: f32,          // scale for curve
    pub curve: ThrottleCurve,
}

impl PropulsionModel for SimplePropulsion {
    fn accel_contrib(&self, _ctx: &StepCtx, q: PropQuery) -> [f32; 3] {
        if q.mass <= 0.0 { return [0.0; 3]; }
        let gain = eval_curve(&self.curve, q.throttle01).max(0.0);
        let thrust = self.thrust_constant_n + self.thrust_max_n * gain;
        let a_mag = thrust / q.mass;
        let dir = Vec3::from_array(q.forward_dir_world).normalize_or_zero();
        (dir * a_mag).to_array()
    }
}
