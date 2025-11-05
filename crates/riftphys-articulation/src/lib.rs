use riftphys_core::{Scalar, BodyId, JointId};
use riftphys_dynamics::Bodies;

/// Simple XPBD distance joint between body centers.
/// Rest length L, compliance α (0 = rigid).
#[derive(Copy, Clone, Debug)]
pub struct DistanceJoint {
    pub a: BodyId,
    pub b: BodyId,
    pub rest: Scalar,
    pub compliance: Scalar,
}

#[derive(Default)]
pub struct Joints {
    distance: Vec<DistanceJoint>,
}

impl Joints {
    pub fn new() -> Self { Self { distance: Vec::new() } }
    pub fn set_distance_params(&mut self, id: JointId, rest: f32, compliance: f32) {
        if let Some(j) = self.distance.get_mut(id.0 as usize) {
            j.rest = rest;
            j.compliance = compliance;
        }
    }
    pub fn add_distance_joint(
        &mut self,
        a: BodyId, b: BodyId,
        rest: Scalar,
        compliance: Scalar
    ) -> JointId {
        self.distance.push(DistanceJoint { a, b, rest, compliance });
        JointId((self.distance.len() as u32) - 1)
    }

    /// XPBD positional solve; does not assume angular DOFs (fine for pendulum/rope).
    pub fn solve(&self, bodies: &mut Bodies, dt: Scalar, iterations: u32) {
        if self.distance.is_empty() { return; }
        let alpha_dt2 = |c: Scalar| if c <= 0.0 { 0.0 } else { c / (dt * dt) };

        for _ in 0..iterations {
            for j in &self.distance {
                let ia = j.a.0;
                let ib = j.b.0;

                let wa = bodies.inv_mass_of(ia);
                let wb = bodies.inv_mass_of(ib);
                let w_sum = wa + wb;
                if w_sum == 0.0 { continue; }

                let pa = bodies.pose(ia).pos;
                let pb = bodies.pose(ib).pos;
                let d  = pb - pa;
                let len = d.length();
                if len <= 1.0e-6 { continue; }

                let n = d / len;
                let c_val = len - j.rest;             // C(p) = |pb-pa| - L
                if c_val.abs() <= 1.0e-6 { continue; }

                let alpha = alpha_dt2(j.compliance);   // compliance term
                let lambda = -c_val / (w_sum + alpha); // XPBD closed form for scalar constraint

                let dp_a = -n * (lambda * wa);
                let dp_b =  n * (lambda * wb);

                bodies.apply_position_delta(ia, dp_a);
                bodies.apply_position_delta(ib, dp_b);
                // (Optional: velocity correction from position delta can be added later)
            }
        }
    }
}
