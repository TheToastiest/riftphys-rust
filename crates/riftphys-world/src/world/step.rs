use rayon::prelude::*;
use super::world::{Contact, World, CCD_MAX_MARGIN};
use riftphys_core::{StepStage, StepStats, Scalar, Quat};
use riftphys_geom::{Aabb};
use riftphys_collision::narrowphase::collide_shapes;
use riftphys_gravity::eval as grav_eval;
use riftphys_viz::LedgerEvent;
use riftphys_core::Isometry;
use riftphys_geom::Shape;
use riftphys_core::Vec3;
pub const TERRAIN_CI: usize = u32::MAX as usize;

// We use a safer, tighter margin for melee to prevent ghosting
// and false speculative contacts from ruining the narrowphase solver.
const MELEE_CCD_MARGIN: f32 = 0.05;

impl World {
    pub fn step(&mut self, dt: Scalar) -> StepStats {
        self.schedule.clear();
        self.tick = self.tick.wrapping_add(1);
        self.ledger.clear();
        self.reset_provenance();
        self.apply_pending_epoch_if_any();
        self.update_players(dt);

        let count = self.bodies.len() as u32;

        // --- STAGE 1: Velocity Integration & CCD DISPATCH ---
        self.schedule.push(StepStage::Integrate);
        for i in 0..count {
            if !self.bodies.is_dynamic(i) || !self.sleep.is_awake(i as usize) { continue; }

            let pose = self.bodies.pose(i);
            let mut vel = self.bodies.vel(i);

            // Apply Gravity
            let mut a_total = grav_eval(&self.gravity_proc, pose.pos);

            // Apply Extra Accelerations (Aero/Prop)
            if let Some(Some(ac)) = self.accel_comps.get(i as usize) {
                let inv_m = self.bodies.inv_mass_of(i);
                if inv_m > 0.0 {
                    let (a_grav, a_extra) = Self::eval_extra_accel(
                        &self.models, &self.bodies, &self.accel_comps,
                        self.epoch_id, self.tick, i, pose, vel, &self.gravity_proc, dt
                    );
                    a_total = a_grav + a_extra;
                }
            }

            vel.lin += a_total * dt;
            self.bodies.set_vel(i, vel);

            // THE CRITICAL FIX: CCD DISPATCH
            // If the body is moving fast, use the sweep logic to prevent tunneling.
            // This is what stops Mr. Box from ignoring the floor.
            if vel.lin.length_squared() > 0.01 { // Only CCD if moving > 0.1m/s
                if let Some(shape) = self.primary_shape(riftphys_core::BodyId(i)) {
                    match shape {
                        Shape::Box { hx, hy, hz } => { self.ccd_integrate_box(i, hx, hy, hz, dt); }
                        Shape::Capsule { r, hh } => { self.ccd_integrate_capsule(i, r, hh, dt); }
                        Shape::Sphere { r } => { self.ccd_integrate_sphere(i, dt); }
                        _ => {}
                    }
                }
            }
        }

        // --- STAGE 2: Broadphase ---
        self.schedule.push(StepStage::UpdateAabbsPre);
        self.update_colliders_aabb(); // Use the existing helper to refresh AABBs

        self.schedule.push(StepStage::BroadphaseSap);
        let mut aabbs: Vec<Aabb> = self.colliders.iter().map(|c| c.aabb).collect();
        for (i, c) in self.colliders.iter().enumerate() {
            if !self.sleep.is_awake(c.body.0 as usize) { continue; }
            let speed = self.bodies.vel(c.body.0).lin.length();
            let margin = (speed * dt).min(MELEE_CCD_MARGIN);
            if margin > 0.0 { aabbs[i].expand_by(margin); }
        }

        let pairs = self.broadphase.run(&aabbs);
        let pairs_tested_count = pairs.len() as u32;

        // --- STAGE 3: Narrowphase ---
        self.schedule.push(StepStage::Narrowphase);
        let colliders = &self.colliders;
        let bodies = &self.bodies;
        let sleep = &self.sleep;

        // Rigid Body vs Rigid Body
        let mut contacts: Vec<Contact> = pairs.par_iter().flat_map(|&(i, j)| {
            let ca = &colliders[i];
            let cb = &colliders[j];
            let mut local = Vec::with_capacity(4);

            let a_awake = sleep.is_awake(ca.body.0 as usize);
            let b_awake = sleep.is_awake(cb.body.0 as usize);
            if !a_awake && !b_awake { return local; }

            let margin_a = if a_awake { bodies.vel(ca.body.0).lin.length() * dt } else { 0.0 };
            let margin_b = if b_awake { bodies.vel(cb.body.0).lin.length() * dt } else { 0.0 };
            let contact_dist = (margin_a + margin_b).min(MELEE_CCD_MARGIN);

            if let Some(manifold) = collide_shapes(&ca.shape, &bodies.pose(ca.body.0), &cb.shape, &bodies.pose(cb.body.0), contact_dist) {
                // Wake logic
                if a_awake && !b_awake && bodies.vel(ca.body.0).lin.length_squared() > sleep.lin_threshold_sq { sleep.wake(cb.body.0 as usize); }
                if !a_awake && b_awake && bodies.vel(cb.body.0).lin.length_squared() > sleep.lin_threshold_sq { sleep.wake(ca.body.0 as usize); }

                for pt_idx in 0..manifold.len {
                    let pt = &manifold.points[pt_idx as usize];
                    local.push(Contact { a_collider: i, b_collider: j, normal: pt.normal, depth: pt.penetration, env_material: None });
                }
            }
            local
        }).collect();

        // Environment Narrowphase (Terrain)
        let env_contacts: Vec<Contact> = colliders.par_iter().enumerate().filter_map(|(ci, c)| {
            if bodies.inv_mass_of(c.body.0) == 0.0 || !sleep.is_awake(c.body.0 as usize) { return None; }

            let mut local = Vec::new();
            let mut env_buffer = Vec::new();

            // THE FIX: Expanded Query to prevent ghosting
            let mut query_aabb = c.aabb;
            query_aabb.expand_by(bodies.vel(c.body.0).lin.length() * dt + 0.1);

            for env in &self.environments {
                env.query_aabb(&query_aabb, &mut env_buffer);
            }

            let contact_dist = (bodies.vel(c.body.0).lin.length() * dt).min(MELEE_CCD_MARGIN);

            for env_col in env_buffer {
                if let Some(manifold) = collide_shapes(&c.shape, &bodies.pose(c.body.0), &env_col.shape, &env_col.transform, contact_dist) {
                    for pt_idx in 0..manifold.len {
                        let pt = &manifold.points[pt_idx as usize];
                        local.push(Contact {
                            a_collider: ci, b_collider: TERRAIN_CI,
                            normal: pt.normal, depth: pt.penetration, env_material: Some(env_col.material)
                        });
                    }
                }
            }
            if local.is_empty() { None } else { Some(local) }
        }).flatten().collect();

        contacts.extend(env_contacts);

        // --- STAGE 4 & 5: Island Solving ---
        let mut contacts = self.cull_and_orient_contacts(contacts);
        self.schedule.push(StepStage::Solve);
        let islands = self.island_manager.build_islands(self.bodies.len(), &self.colliders, &self.bodies, &self.joints, &contacts);

        if !islands.is_empty() {
            self.solve_islands_parallel(&islands, &contacts, dt);
        }

        // --- STAGE 6: Position Integration ---
        for i in 0..count {
            if !self.bodies.is_dynamic(i) || !self.sleep.is_awake(i as usize) { continue; }
            let mut pose = self.bodies.pose(i);
            let vel = self.bodies.vel(i);
            pose.pos += vel.lin * dt;
            if vel.ang.length_squared() > 1e-9 {
                let axis = vel.ang.normalize();
                let angle = vel.ang.length() * dt;
                let rotation_step = Quat::from_axis_angle(axis, angle);
                pose.rot = (rotation_step * pose.rot).normalize();
            }
            self.bodies.set_pose(i, pose);
        }

        // --- STAGE 7: Sleep Evaluation ---
        self.sleep.evaluate(count, &self.bodies);
        self.schedule.push(StepStage::UpdateAabbsPost);
        self.update_colliders_aabb();

        StepStats { pairs_tested: pairs_tested_count, contacts: contacts.len() as u32, islands: islands.len() as u32 }
    }

    fn cull_and_orient_contacts(&self, mut contacts: Vec<Contact>) -> Vec<Contact> {
        let mut buckets: std::collections::BTreeMap<(u32,u32), Vec<Contact>> = std::collections::BTreeMap::new();
        for c in contacts.drain(..) {
            let key = if c.a_collider <= c.b_collider { (c.a_collider as u32, c.b_collider as u32) }
            else { (c.b_collider as u32, c.a_collider as u32) };
            buckets.entry(key).or_default().push(c);
        }

        let mut culled = Vec::new();
        for (_, mut v) in buckets {
            v.sort_by(|a, b| b.depth.partial_cmp(&a.depth).unwrap());
            v.truncate(4);
            culled.extend(v);
        }

        for c in &mut culled {
            if c.b_collider == TERRAIN_CI { continue; }
            let pa = self.bodies.pose(self.colliders[c.a_collider].body.0).pos;
            let pb = self.bodies.pose(self.colliders[c.b_collider].body.0).pos;
            if c.normal.dot(pb - pa) < 0.0 { c.normal = -c.normal; }
        }
        culled
    }
    pub fn update_players(&mut self, dt: f32) {
        for p_idx in 0..self.players.len() {
            let mut player = self.players[p_idx].clone();
            let mut pos = self.bodies.pose(player.body.0).pos;

            // 1. Gravity & Sticky Floor Logic
            if !player.grounded {
                player.vertical_vel -= 9.81 * dt; // Fall
            } else {
                // Apply a continuous sticky downward velocity to hug slopes
                if player.vertical_vel <= 0.0 { player.vertical_vel = -0.5; }
            }

            // 2. Decode the Input Vector from the FFI
            let input_mag_sq = player.input_dir.length_squared();
            let move_dir = if input_mag_sq > 1e-6 {
                player.input_dir / input_mag_sq.sqrt()
            } else {
                Vec3::ZERO
            };

            let mut desired_move = (move_dir * player.speed * dt)
                + Vec3::new(0.0, player.vertical_vel * dt, 0.0);

            player.grounded = false; // Assume falling until we verify floor hit

            // 3. Iterative Sweep/Slide
            for _ in 0..4 {
                if desired_move.length_squared() < 1e-6 { break; }

                let player_xf = Isometry { pos: pos + desired_move, rot: Quat::IDENTITY };
                let player_shape = Shape::Capsule { r: player.radius, hh: player.height * 0.5 };
                let player_aabb = riftphys_geom::aabb_of(&player_shape, &player_xf);

                let mut hit_normal = None;
                let mut hit_penetration = 0.0;
                let mut hit_body = None;

                // A. Check Terrain with 5cm Slop Margin
                let mut env_buffer = Vec::new();
                for env in &self.environments { env.query_aabb(&player_aabb, &mut env_buffer); }
                for env_col in env_buffer {
                    // THE FIX: 0.05 margin ensures continuous floor detection
                    if let Some(m) = riftphys_collision::narrowphase::collide_shapes(&player_shape, &player_xf, &env_col.shape, &env_col.transform, 0.05) {
                        hit_normal = Some(m.points[0].normal);
                        hit_penetration = m.points[0].penetration;
                        break;
                    }
                }

                // B. Check Spheres with 5cm Slop Margin
                if hit_normal.is_none() {
                    for (ci, c) in self.colliders.iter().enumerate() {
                        if c.body != player.body && player_aabb.intersects(&c.aabb) {
                            if let Some(m) = riftphys_collision::narrowphase::collide_shapes(&player_shape, &player_xf, &c.shape, &self.bodies.pose(c.body.0), 0.05) {
                                hit_normal = Some(m.points[0].normal);
                                hit_penetration = m.points[0].penetration;
                                hit_body = Some(c.body);
                                break;
                            }
                        }
                    }
                }

                // C. Geometrically Stable Resolution
                if let Some(normal) = hit_normal {
                    if normal.y > 0.6 { player.grounded = true; player.vertical_vel = 0.0; }

                    // Apply the penetration pushout directly to the target movement vector
                    desired_move += normal * (hit_penetration + 0.001);

                    // Kill only the velocity going directly into the wall/floor
                    let dot = desired_move.dot(normal);
                    if dot < 0.0 {
                        desired_move -= normal * dot;
                    }

                    // Kick dynamic spheres
                    if let Some(b_id) = hit_body {
                        if self.bodies.is_dynamic (b_id.0) {
                            self.sleep.wake(b_id.0 as usize);
                            self.bodies.apply_impulse(b_id.0, -normal * 10.0);
                        }
                    }
                } else {
                    break; // Path is clear, break early
                }
            }

            // Finally, apply the guaranteed-safe movement vector
            pos += desired_move;
            self.bodies.set_pose(player.body.0, Isometry { pos, rot: Quat::IDENTITY });
            self.players[p_idx] = player;
        }
    }
}