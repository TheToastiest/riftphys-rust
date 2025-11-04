//! Simple wheeled-vehicle helpers (MVP).
//!
//! Defines wheel/suspension parameters, driver input, a `VehicleParams`
//! and a `VehicleInstance` that can be advanced with `step_with_host`.
//!
//! This crate is *world-agnostic*: callers provide three callbacks to
//! read the chassis pose/velocity and to write back the new velocity.
//! Terrain comes from `riftphys_terrain::HeightField`.

use glam::{Affine3A, Quat, Vec3 as GVec3};
use riftphys_core::{BodyId, Scalar, Vec3, Velocity};          // Vec3 is glam::Vec3A alias
use riftphys_core::types::Vec3 as V3a;                          // explicit alias to Vec3A
use riftphys_terrain::HeightField;

/// Wheel geometry and suspension / tire parameters.
#[derive(Copy, Clone, Debug)]
pub struct WheelParams {
    /// Wheel attach point in chassis space.
    pub local_pos: Vec3,
    /// Suspension/down axis in chassis space (e.g., -Y).
    pub susp_dir:  Vec3,
    /// Suspension rest length (m).
    pub rest_len:  f32,
    /// Spring rate (N/m) along `susp_dir`.
    pub k_spring: f32,
    /// Damping (N·s/m) along `susp_dir`.
    pub k_damp:   f32,
    /// Wheel radius (m).
    pub radius:   f32,
    /// Longitudinal friction coefficient.
    pub mu_long:  f32,
    /// Lateral friction coefficient.
    pub mu_lat:   f32,
}

/// Driver input for a vehicle tick.
#[derive(Copy, Clone, Debug)]
pub struct AxleInput {
    /// Throttle in [-1, 1] (negative ≈ brake/reverse in this MVP).
    pub throttle01: f32,
    /// Steering angle (radians, + = steer left around +Y).
    pub steer_rad:  f32,
}

/// Static vehicle description: chassis body and wheel layout.
#[derive(Clone, Debug)]
pub struct VehicleParams {
    /// Chassis body id in the host world.
    pub body: BodyId,
    /// Per-wheel parameters (index 0..N).
    pub wheels: Vec<WheelParams>,
    /// Indices into `wheels` that receive drive torque.
    pub drive_map: Vec<usize>,
    /// Indices into `wheels` that steer.
    pub steer_map: Vec<usize>,
    /// Chassis mass hint (kg) used to convert forces → acceleration.
    pub mass_hint: f32,
}

/// Runtime container you update every tick.
#[derive(Clone, Debug)]
pub struct VehicleInstance {
    /// Static setup.
    pub p: VehicleParams,
    /// Current driver input.
    pub input: AxleInput,
}

#[inline]
fn v3a(v: GVec3) -> V3a { v.into() }

impl VehicleInstance {
    /// Advance one physics tick using world–agnostic callbacks.
    ///
    /// * `pose_of`  – given a body id, return `(position, rotation)`.
    /// * `vel_of`   – given a body id, return current `Velocity`.
    /// * `set_vel`  – write a new `Velocity` for the body.
    /// * `hf`       – optional heightfield for wheel contact.
    /// * `dt`       – timestep (seconds).
    pub fn step_with_host<P, V, S>(
        &mut self,
        mut pose_of: P,
        mut vel_of: V,
        mut set_vel: S,
        hf: Option<&HeightField>,
        dt: Scalar,
    ) where
        P: FnMut(BodyId) -> (GVec3, Quat),
        V: FnMut(BodyId) -> Velocity,
        S: FnMut(BodyId, Velocity),
    {
        let (pos_w, rot_w) = pose_of(self.p.body);
        let pose_aff = Affine3A::from_rotation_translation(rot_w, pos_w);
        let vel = vel_of(self.p.body);

        let mut force_sum_w = GVec3::ZERO;

        for i in 0..self.p.wheels.len() {
            let (hit, _cp, n_core, _pen, f_susp, f_long, f_lat) =
                self.eval_wheel(i, pose_aff, vel, hf);

            if !hit {
                continue;
            }

            let steer_q = Quat::from_axis_angle(GVec3::Y, self.input.steer_rad);
            let fwd_w = steer_q * GVec3::X;
            let lat_w = steer_q * GVec3::Z;
            let steer_q = Quat::from_axis_angle(GVec3::Y, self.input.steer_rad);
            let fwd_w = steer_q * GVec3::X;
            let lat_w = steer_q * GVec3::Z;

            let n_w: GVec3 = n_core.into();
            let f_wheel = n_w * f_susp + fwd_w * f_long + lat_w * f_lat;
            force_sum_w += f_wheel;
        }

        if force_sum_w.length_squared() > 0.0 {
            let mut v = vel;
            let a = force_sum_w / self.p.mass_hint.max(1.0);
            v.lin = v.lin + v3a(a) * dt; // integrate linear velocity (MVP)
            set_vel(self.p.body, v);
        }
    }

    /// Internal wheel query against a heightfield.
    /// Returns `(contact?, contact_point_world, normal_world, penetration, f_susp, f_long, f_lat)`.
    fn eval_wheel(
        &self,
        idx: usize,
        pose_world: Affine3A,
        vel_world: Velocity,
        hf: Option<&HeightField>,
    ) -> (bool, Vec3, Vec3, f32, f32, f32, f32) {
        let wp = self.p.wheels[idx];

        // Anchor and suspension axis in world.
        let p0_w  = pose_world.transform_point3(wp.local_pos.into());
        let dir_w = pose_world
            .transform_vector3(wp.susp_dir.into())   // ✅ correct glam API
            .normalize_or_zero();


        let cast_len = wp.rest_len + wp.radius;

        let Some(hf) = hf else {
            return (false, Vec3::ZERO, Vec3::Y, 0.0, 0.0, 0.0, 0.0);
        };

        // Sample ground directly under the wheel in XZ.
        let (wx, wz) = (p0_w.x, p0_w.z);
        let ground_y = hf.sample_height(wx, wz);
        let n_g: GVec3 = hf.sample_normal(wx, wz);

        // Wheel bottom at full extension.
        let y_bottom = p0_w.y - cast_len;
        if y_bottom > ground_y {
            return (false, Vec3::ZERO, Vec3::Y, 0.0, 0.0, 0.0, 0.0);
        }

        // Compression along suspension axis.
        let x = (ground_y - y_bottom).clamp(0.0, cast_len);

        // Relative velocity along suspension axis.
        let v_anchor = vel_world.lin;              // V3a
        let v_rel = v_anchor.dot(v3a(dir_w));      // f32

        let f_spring = wp.k_spring * x;
        let f_damp   = wp.k_damp * v_rel;
        let f_susp   = f_spring - f_damp;          // along +dir_w

        // Tire forces (very simple).
        let steer = if self.p.steer_map.iter().any(|&k| k == idx) {
            self.input.steer_rad
        } else {
            0.0
        };
        let steer_q = Quat::from_axis_angle(GVec3::Y, self.input.steer_rad);
        let fwd_w: V3a = v3a(steer_q * GVec3::X);
        let lat_w: V3a = v3a(steer_q * GVec3::Z);

        let v = vel_world.lin;
        let v_long = v.dot(fwd_w);
        let v_lat  = v.dot(lat_w);

        let g = 9.81_f32;
        let fmax_long = wp.mu_long * self.p.mass_hint * g;
        let fmax_lat  = wp.mu_lat  * self.p.mass_hint * g;

        let mut f_long = (self.input.throttle01 * fmax_long).clamp(-fmax_long, fmax_long);
        // super-MVP lateral “cornering spring”
        let mut f_lat = (-v_lat * self.p.mass_hint).clamp(-fmax_lat, fmax_lat);

        (
            true,
            Vec3::new( wx, ground_y, wz ),
            v3a(n_g),          // convert to core Vec3 (Vec3A)
            x,
            f_susp,
            f_long,
            f_lat,
        )
    }
}
