use super::world::{Contact, World, CCD_MAX_MARGIN};

use riftphys_core::{StepStage, StepStats, Scalar, Vec3};
use riftphys_geom::{aabb_of, Aabb, Shape};
use riftphys_collision::pairs_sap;
use riftphys_gravity::eval as grav_eval;
use riftphys_viz::LedgerEvent;

impl World {
    pub fn step(&mut self, dt: Scalar) -> StepStats {
        self.schedule.clear();
        self.tick = self.tick.wrapping_add(1);
        self.ledger.clear();
        self.reset_provenance();

        // boundary swaps
        self.apply_pending_epoch_if_any();

        // Integrate (CCD-aware)
        self.schedule.push(StepStage::Integrate);
        let count = self.bodies.len() as u32;

        for i in 0..count {
            if !self.bodies.is_dynamic(i) || self.bodies.inv_mass_of(i) == 0.0 { continue; }

            // Capsule CCD first
            // Capsule CCD first
            let mut cap_rhh: Option<(f32, f32)> = None;
            for c in &self.colliders {
                if c.body.0 != i { continue; }
                if let Shape::Capsule { r, hh } = c.shape {
                    cap_rhh = Some((r, hh));
                    break; // only break once we actually found a capsule
                }
            }

            if let Some((r, hh)) = cap_rhh {
                if self.ccd_integrate_capsule(i, r, hh, dt) {
                    continue;
                }
            }


            // Sphere CCD
            if self.ccd_integrate_sphere(i, dt) {
                continue;
            }

            // Fallback: gravity + accel integrate
            let pose = self.bodies.pose(i);
            let mut vel = self.bodies.vel(i);

            let mut a_grav = grav_eval(&self.gravity_proc, pose.pos);
            let gmag = a_grav.length();

            if gmag > 1.0e-6 {
                // Planet sphere snapping (static sphere colliders)
                let mut best_gap: f32 = f32::INFINITY;
                let mut radial_down: Option<Vec3> = None;

                for col in &self.colliders {
                    if let Shape::Sphere { r } = col.shape {
                        if self.bodies.inv_mass_of(col.body.0) == 0.0 {
                            let center = self.bodies.pose(col.body.0).pos;
                            let rv = pose.pos - center;
                            let dist = rv.length();
                            if dist > 1.0e-6 {
                                let gap = dist - r;
                                if gap < best_gap && gap < 5.0 {
                                    best_gap = gap;
                                    radial_down = Some((-rv / dist).normalize());
                                }
                            }
                        }
                    }
                }

                if let Some(rd) = radial_down {
                    a_grav = rd * gmag;
                } else if let Some((h_world, n_hf)) = self.sample_terrain_height_normal(pose.pos.x, pose.pos.z) {
                    let height_above = pose.pos.y - h_world;
                    if height_above.is_finite() {
                        let t = (1.0 - (height_above / 50.0)).clamp(0.0, 1.0);
                        if t > 0.0 {
                            let dir_spec = a_grav / gmag;
                            let dir_hf   = (-n_hf).normalize();
                            let mix      = (dir_spec * (1.0 - t) + dir_hf * t).normalize();
                            a_grav = mix * gmag;
                        }
                    }
                }
            }

            // Phase 11 accel: aero + prop
            let mut a_extra = Vec3::ZERO;
            if let Some(Some(ac)) = self.accel_comps.get(i as usize) {
                let inv_m = self.bodies.inv_mass_of(i);
                if inv_m > 0.0 {
                    let mass = 1.0 / inv_m;
                    let fwd_w = ac.forward_dir_world.unwrap_or_else(|| pose.rot * Vec3::new(1.0, 0.0, 0.0));
                    let v_w = vel.lin;
                    let speed = v_w.length();
                    let vhat = if speed > 1e-6 { v_w / speed } else { fwd_w };
                    let cos_a = fwd_w.dot(vhat).clamp(-1.0, 1.0);
                    let alpha_rad = cos_a.acos();

                    let aq = riftphys_core::models::AeroQuery {
                        vel_world: [v_w.x, v_w.y, v_w.z],
                        ang_vel_world: [vel.ang.x, vel.ang.y, vel.ang.z],
                        orientation: pose.rot,
                        area: ac.ref_area_m2,
                        mass,
                        altitude: pose.pos.y,
                        alpha_rad,
                    };
                    let pq = riftphys_core::models::PropQuery {
                        throttle01: ac.throttle01,
                        forward_dir_world: [fwd_w.x, fwd_w.y, fwd_w.z],
                        mass,
                    };

                    let mut a_aero = Vec3::ZERO;
                    let mut a_prop = Vec3::ZERO;

                    if let Some(h) = ac.aero {
                        let a = self.models.aero(h).accel_contrib(
                            &riftphys_core::StepCtx { dt, tick: self.tick, epoch: riftphys_core::EpochId(self.epoch_id) }, aq);
                        a_aero = Vec3::new(a[0], a[1], a[2]);
                    }
                    if let Some(h) = ac.prop {
                        let a = self.models.prop(h).accel_contrib(
                            &riftphys_core::StepCtx { dt, tick: self.tick, epoch: riftphys_core::EpochId(self.epoch_id) }, pq);
                        a_prop = Vec3::new(a[0], a[1], a[2]);
                    }

                    let v = vel.lin;
                    let speed = v.length();
                    if speed > 1e-6 {
                        let vhat = v / speed;
                        let a_drag_along = -a_aero.dot(vhat);
                        let t_est_n = a_prop.length() * mass;
                        let d_est_n = (a_drag_along.max(0.0)) * mass;
                        self.record(LedgerEvent::AeroProp { id: i, t_n: t_est_n, d_n: d_est_n, speed });
                    }

                    a_extra += a_aero + a_prop;
                }
            }

            let a_total = a_grav + a_extra;
            vel.lin += a_total * dt;
            let new_pos = pose.pos + vel.lin * dt;

            self.bodies.set_vel(i, vel);
            self.record(LedgerEvent::Integrate { id: i, a: a_total, dv: a_total * dt });
            self.bodies.set_pose(i, riftphys_core::Isometry { pos: new_pos, rot: pose.rot });
        }

        // Update AABBs (pre)
        self.schedule.push(StepStage::UpdateAabbsPre);
        for idx in 0..self.colliders.len() {
            let b = self.colliders[idx].body;
            let shape = self.colliders[idx].shape;
            let pose = self.bodies.pose(b.0);
            self.colliders[idx].aabb = aabb_of(&shape, &pose);
        }

        // Broadphase (SAP) with speculative CCD margin
        self.schedule.push(StepStage::BroadphaseSap);
        let mut aabbs: Vec<Aabb> = self.colliders.iter().map(|c| c.aabb).collect();
        for (i, c) in self.colliders.iter().enumerate() {
            let speed = self.bodies.vel(c.body.0).lin.length();
            let margin = (speed * dt).min(CCD_MAX_MARGIN);
            if margin > 0.0 { aabbs[i].expand_by(margin); }
        }
        let pairs = pairs_sap(&aabbs);

        // Narrowphase
        self.schedule.push(StepStage::Narrowphase);
        let mut contacts = Vec::new();
        for (i, j) in pairs.iter().copied() {
            if let Some(c) = self.contact_box_box(i, j)       { contacts.push(c); continue; }
            if let Some(c) = self.contact_sphere_sphere(i, j) { contacts.push(c); continue; }
            if let Some(c) = self.contact_sphere_box(i, j)    { contacts.push(c); continue; }
            if let Some(c) = self.contact_capsule_box(i, j)   { contacts.push(c); continue; }
        }

        // Deterministic cull: keep at most 4 contacts per collider pair
        let axis_code = |n: Vec3| -> u8 {
            let ax = n.x.abs(); let ay = n.y.abs(); let az = n.z.abs();
            if ax >= ay && ax >= az { 0 } else if ay >= az { 1 } else { 2 }
        };

        let mut buckets: std::collections::BTreeMap<(u32,u32), Vec<Contact>> = std::collections::BTreeMap::new();
        for c in contacts.drain(..) {
            let key = if c.a_collider <= c.b_collider { (c.a_collider as u32, c.b_collider as u32) }
            else                             { (c.b_collider as u32, c.a_collider as u32) };
            buckets.entry(key).or_default().push(c);
        }

        let mut contacts_culled: Vec<Contact> = Vec::new();
        for (_key, mut v) in buckets {
            v.sort_by(|c1, c2| {
                let ac1 = axis_code(c1.normal);
                let ac2 = axis_code(c2.normal);
                ac1.cmp(&ac2)
                    .then_with(|| c2.depth.partial_cmp(&c1.depth).unwrap())
                    .then_with(|| c1.a_collider.cmp(&c2.a_collider))
                    .then_with(|| c1.b_collider.cmp(&c2.b_collider))
            });
            v.truncate(4);
            contacts_culled.extend(v.into_iter());
        }
        let mut contacts = contacts_culled;

        // Ensure final orientation is A -> B
        for c in &mut contacts {
            let a = self.colliders[c.a_collider].body;
            let b = self.colliders[c.b_collider].body;
            let pa = self.bodies.pose(a.0).pos;
            let pb = self.bodies.pose(b.0).pos;
            if c.normal.dot(pb - pa) < 0.0 { c.normal = -c.normal; }
        }

        // Quantize normals and depths (kill ulp jitter)
        let q = 1.0e-6f32;
        for c in &mut contacts {
            let x = (c.normal.x / q).round() * q;
            let y = (c.normal.y / q).round() * q;
            let z = (c.normal.z / q).round() * q;
            let len = (x*x + y*y + z*z).sqrt();
            c.normal = if len > 1.0e-20 { riftphys_core::vec3(x/len, y/len, z/len) }
            else { riftphys_core::vec3(0.0, 1.0, 0.0) };
            c.depth  = (c.depth / q).round() * q;
        }

        // Guard controllers
        {
            let (colliders, _bodies, guards, joints, ledger) = (
                &self.colliders,
                &self.bodies,
                &mut self.guards,
                &mut self.joints,
                &mut self.ledger,
            );

            for g in guards.iter_mut() {
                let mut hit = false;
                for c in &contacts {
                    let a = colliders[c.a_collider].body;
                    let b = colliders[c.b_collider].body;
                    if a == g.eff || b == g.eff { hit = true; break; }
                }

                if hit { g.ctrl.on_contact(); }
                let (rest, comp) = g.ctrl.step(dt);

                joints.set_distance_params(g.joint, rest, comp);
                ledger.push(LedgerEvent::JointDistance {
                    a: g.eff.0,
                    b: 0,
                    lambda: comp,
                    compliance: comp,
                });
            }
        }

        // Balance controllers (support center)
        {
            let (colliders, bodies, balances, ledger) = (
                &self.colliders,
                &mut self.bodies,
                &mut self.balances,
                &mut self.ledger,
            );

            for b in balances.iter_mut() {
                let mut acc = 0usize;
                let mut sx = 0.0f32;
                let mut sz = 0.0f32;

                for c in &contacts {
                    let a = colliders[c.a_collider].body;
                    let d = colliders[c.b_collider].body;

                    if a == b.left || a == b.right {
                        let p = bodies.pose(a.0).pos;
                        sx += p.x; sz += p.z; acc += 1;
                    }
                    if d == b.left || d == b.right {
                        let p = bodies.pose(d.0).pos;
                        sx += p.x; sz += p.z; acc += 1;
                    }
                }
                if acc == 0 { continue; }

                let support_xz = glam::Vec2::new(sx / acc as f32, sz / acc as f32);
                let pelvis_p   = bodies.pose(b.pelvis.0).pos;
                let a_xz       = b.ctrl.step(glam::Vec3::new(pelvis_p.x, pelvis_p.y, pelvis_p.z), support_xz);

                let mut v = bodies.vel(b.pelvis.0);
                v.lin.x += a_xz.x * dt;
                v.lin.z += a_xz.y * dt;
                bodies.set_vel(b.pelvis.0, v);

                ledger.push(LedgerEvent::BalanceAccel { id: b.pelvis.0, ax: a_xz.x, az: a_xz.y });
                ledger.push(LedgerEvent::Integrate {
                    id: b.pelvis.0,
                    a: Vec3::new(a_xz.x, 0.0, a_xz.y),
                    dv: Vec3::new(a_xz.x * dt, 0.0, a_xz.y * dt),
                });
            }
        }


        // Joints
        self.joints.solve(&mut self.bodies, dt, 4);

        // Solve
        self.schedule.push(StepStage::Solve);
        let contacts_len = contacts.len() as u32;
        if contacts_len > 0 {
            self.solve_contacts(&contacts);

            // Update AABBs (post)
            self.schedule.push(StepStage::UpdateAabbsPost);
            for idx in 0..self.colliders.len() {
                let b = self.colliders[idx].body;
                let shape = self.colliders[idx].shape;
                let pose = self.bodies.pose(b.0);
                self.colliders[idx].aabb = aabb_of(&shape, &pose);
            }
        }

        // Debug output
        if self.debug.print_every != 0 && (self.tick as u32) % self.debug.print_every == 0 {
            self.print_debug_block(&contacts);
            let _ = self.ledger.write_jsonl("out", self.tick);
        }
        if self.debug.json_every != 0 && (self.tick as u32) % self.debug.json_every == 0 {
            let _ = self.ledger.write_jsonl("out", self.tick);
        }

        StepStats { pairs_tested: pairs.len() as u32, contacts: contacts_len, islands: 1 }
    }
}
