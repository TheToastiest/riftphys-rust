use super::world::World;

use riftphys_core::{Isometry, Scalar, Vec3, Velocity};
use riftphys_geom::{Aabb, Shape};
use riftphys_query as query;
use riftphys_viz::LedgerEvent;

impl World {
    pub(super) fn ccd_integrate_capsule(&mut self, id: u32, r: f32, hh: f32, dt: f32) -> bool {
        // collect box AABBs
        let mut aabbs: Vec<Aabb> = Vec::new();
        for c in &self.colliders {
            if let Shape::Box { .. } = c.shape { aabbs.push(c.aabb); }
        }
        if aabbs.is_empty() { return false; }

        let pose = self.bodies.pose(id);
        let vel  = self.bodies.vel(id).lin;

        // capsule endpoints at start of frame
        let tip0  = pose.pos + (pose.rot * Vec3::new(0.0,  hh, 0.0));
        let base0 = pose.pos + (pose.rot * Vec3::new(0.0, -hh, 0.0));

        if let Some(hit) = query::sweep_two_spheres_aabbs(
            tip0,  vel, r,
            base0, vel, r,
            &aabbs, dt
        ) {
            self.record(LedgerEvent::CCDHit { id, toi: hit.toi });

            let t = hit.toi.clamp(0.0, 1.0);
            let n = hit.normal;
            let mut v = vel;

            // advance to impact
            let p_impact = pose.pos + v * (t * dt);

            // remove incoming normal component
            let vn = v.dot(n);
            if vn < 0.0 { v -= n * vn; }

            // advance remainder with accel
            let (a_grav, a_extra) = Self::eval_extra_accel(
                &self.models, &self.bodies, &self.accel_comps,
                self.epoch_id, self.tick, id, pose, self.bodies.vel(id), &self.gravity_proc, dt
            );
            let a_total = a_grav + a_extra;
            v += a_total * dt;

            let p_after = p_impact + v * (dt * (1.0 - t));
            self.bodies.set_pose(id, Isometry { pos: p_after, rot: pose.rot });
            self.bodies.set_vel (id, Velocity { lin: v, ang: self.bodies.vel(id).ang });
            return true;
        }
        false
    }

    pub(super) fn ccd_integrate_sphere(&mut self, id: u32, dt: Scalar) -> bool {
        // find radius for this body's sphere collider
        let mut radius = None::<f32>;
        for c in &self.colliders {
            if c.body.0 == id {
                if let Shape::Sphere { r } = c.shape { radius = Some(r); }
                break;
            }
        }
        let r = match radius { Some(r) => r, None => return false };

        let (aabbs, _map) = self.gather_box_aabbs_with_map();
        if aabbs.is_empty() { return false; }

        let pose = self.bodies.pose(id);
        let vel  = self.bodies.vel(id).lin;

        let hit = match query::sweep_sphere_aabbs(pose.pos, vel, r, &aabbs, dt) {
            Some(h) => h,
            None => return false,
        };

        let toi = hit.toi;
        let normal = hit.normal;

        self.record(LedgerEvent::CCDHit { id, toi });

        let mid_dt = toi.clamp(0.0, 1.0) * dt;
        let mut p = pose.pos + vel * mid_dt;

        let vn = vel.dot(normal);
        let mut new_vel = vel;
        if vn < 0.0 { new_vel -= normal * vn; }

        let (a_grav, a_extra) = Self::eval_extra_accel(
            &self.models, &self.bodies, &self.accel_comps,
            self.epoch_id, self.tick, id, pose, self.bodies.vel(id), &self.gravity_proc, dt
        );
        let a_total = a_grav + a_extra;

        let rem_dt = dt - mid_dt;
        new_vel += a_total * dt;
        p += new_vel * rem_dt;

        self.bodies.set_pose(id, Isometry { pos: p, rot: pose.rot });
        self.bodies.set_vel (id, Velocity { lin: new_vel, ang: self.bodies.vel(id).ang });
        true
    }
}
