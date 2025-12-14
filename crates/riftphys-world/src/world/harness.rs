use super::world::World;
use crate::det_harness::{Inputs, InputEvent, SimWorld, StepReport};

impl SimWorld for World {
    fn step_dt(&mut self, dt: f32) -> StepReport {
        let stats = self.step(dt);
        let (imp, ccd, aero, prop) = self.provenance_sums();

        StepReport {
            dt,
            epoch: self.epoch_id,
            hash: World::step_hash(self), // explicit inherent call; avoids recursion ambiguity
            pairs_tested: stats.pairs_tested,
            contacts: stats.contacts,
            impulses_sum: imp,
            ccd_hits: ccd,
            aero_sum: aero,
            prop_sum: prop,
        }
    }

    fn epoch_id(&self) -> u64 { self.epoch_id }

    fn step_hash(&self) -> [u8; 32] {
        World::step_hash(self)
    }

    fn apply_inputs(&mut self, inputs: &Inputs) {
        use InputEvent::*;
        use riftphys_core::{vec3, Velocity};

        for ev in &inputs.events {
            match ev {
                SetThrottle { body, throttle01 } => {
                    self.set_body_throttle(*body, *throttle01);
                }
                SetVelocity { body, lin, ang } => {
                    self.set_body_vel(
                        *body,
                        Velocity {
                            lin: vec3(lin[0], lin[1], lin[2]),
                            ang: vec3(ang[0], ang[1], ang[2]),
                        },
                    );
                }
                SetBodyAccel { body, aero, prop, ref_area, throttle01 } => {
                    self.set_body_accel(*body, *aero, *prop, *ref_area, *throttle01, None);
                }
                GravityLayeredPlanet { surface_g, radius, center, min_r } => {
                    self.queue_gravity_swap(riftphys_gravity::GravitySpec::LayeredPlanet {
                        surface_g: *surface_g,
                        radius: *radius,
                        center: *center,
                        min_r: *min_r,
                    });
                }
            }
        }
    }
}
