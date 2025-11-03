use riftphys_core::{
    Scalar, Vec3, Isometry, Velocity, BodyId, ColliderId, StepStats, StepHasher, hash_vec3, hash_quat,
    StepStage, XorShift64, EpochDescriptor, epoch_id, JointId,
};
use riftphys_geom::{Aabb, Shape, MassProps, Material, aabb_of};
use riftphys_melee::sweep_capsule_vs_aabb_two_spheres;

use riftphys_collision::pairs_sap;
use riftphys_dynamics::{Bodies, BodyDesc};
use riftphys_viz::{ScheduleRecorder, DebugSettings, Ledger, LedgerEvent};
use riftphys_gravity::{GravitySpec, eval as grav_eval, spec_id as gravity_epoch_id};
use riftphys_articulation::Joints;

use std::collections::BTreeMap;
use riftphys_terrain::HeightField;
use riftphys_melee::sweep_sphere_vs_aabb;
use riftphys_controllers::GuardCtrl;

const CCD_MAX_MARGIN: f32 = 1.0e-0; // expand AABBs by min(|v|*dt, CCD_MAX_MARGIN)
struct GuardInstance { joint: JointId, eff: BodyId, ctrl: GuardCtrl } // eff = shield body

/* ---------------- Terrain cache config ---------------- */
use glam::UVec2;
#[derive(Copy, Clone, Debug)]
struct TerrainTileCfg {
    tile_cells: UVec2,  // e.g., 32x32 cells
    y_offset: f32,
}
#[derive(Clone, Debug, Default)]
struct TileEntry { uses: u32 }

/* ---------------- Collider & Contact ---------------- */
#[derive(Copy, Clone, Debug)]
pub struct Collider {
    pub body: BodyId,
    pub shape: Shape,
    pub aabb: Aabb,
    pub material: Material,
}
#[derive(Copy, Clone, Debug)]
struct Contact {
    a_collider: usize,
    b_collider: usize,
    normal: Vec3,  // from A -> B
    depth: Scalar,
}

/* ---------------- Builder ---------------- */
pub struct WorldBuilder {
    pub bodies: usize,
    pub colliders: usize,
}
impl WorldBuilder {
    pub fn new() -> Self { Self { bodies: 128, colliders: 128 } }
    pub fn with_capacity(mut self, bodies: usize, colliders: usize) -> Self {
        self.bodies = bodies; self.colliders = colliders; self
    }
    pub fn build(self) -> World { World::with_capacity(self.bodies, self.colliders) }
}

/* ---------------- World ---------------- */
pub struct World {
    // legacy convenience gravity vector (kept in sync for Uniform spec)
    pub gravity: Vec3,

    pub epoch_id: u64,
    pub rng: XorShift64,
    schedule: ScheduleRecorder,

    bodies: Bodies,          // SoA
    colliders: Vec<Collider>,

    // Phase 3/4: queued swaps
    pending_epoch: Option<EpochDescriptor>,
    pending_gravity: Option<GravitySpec>,

    // Active gravity procedure (Phase 4)
    gravity_proc: GravitySpec,

    // Systems
    joints: Joints,
    tick: u64,
    debug: DebugSettings,
    ledger: Ledger,

    // Terrain (Phase 5)
    terrain: Option<HeightField>,
    tile_cfg: TerrainTileCfg,
    tile_cache: BTreeMap<(i32, i32), TileEntry>,
    guards: Vec<GuardInstance>,
}

impl World {
    pub fn with_capacity(bodies: usize, colliders: usize) -> Self {
        let g = Vec3::new(0.0, -9.81, 0.0);
        Self {
            gravity: g,
            epoch_id: 0,
            rng: XorShift64::new(0xC0FFEE),
            schedule: ScheduleRecorder::new(),
            bodies: Bodies::with_capacity(bodies),
            colliders: Vec::with_capacity(colliders),
            pending_epoch: None,
            pending_gravity: None,
            gravity_proc: GravitySpec::Uniform { g: [g.x, g.y, g.z] },
            joints: Joints::new(),
            tick: 0,
            debug: DebugSettings::default(),
            ledger: Ledger::new(4096),
            terrain: None,
            tile_cfg: TerrainTileCfg { tile_cells: UVec2::new(32, 32), y_offset: 0.0 },
            tile_cache: BTreeMap::new(),
            guards: Vec::new(),
        }
    }

    pub fn add_guard_controller(
        &mut self,
        pivot: BodyId,
        eff: BodyId,
        params: riftphys_controllers::GuardParams,
    ) -> JointId {
        let j = self.joints.add_distance_joint(pivot, eff, params.rest_guard, params.k_guard);
        self.guards.push(GuardInstance { joint: j, eff, ctrl: riftphys_controllers::GuardCtrl::new(params) });
        j
    }

    // ---- FIX: top-level capsule CCD method (not nested) ----
    fn ccd_integrate_capsule(&mut self, id: u32, r: f32, hh: f32, dt: f32) -> bool {
        // collect box AABBs
        let mut aabbs: Vec<Aabb> = Vec::new();
        for c in &self.colliders {
            if let Shape::Box { .. } = c.shape {
                aabbs.push(c.aabb);
            }
        }
        if aabbs.is_empty() { return false; }

        // current state
        let pose = self.bodies.pose(id);
        let vel  = self.bodies.vel(id).lin;

        // capsule endpoints at start of frame
        let tip0  = pose.pos + (pose.rot * Vec3::new(0.0,  hh, 0.0));
        let base0 = pose.pos + (pose.rot * Vec3::new(0.0, -hh, 0.0));

        if let Some(hit) = sweep_capsule_vs_aabb_two_spheres(
            tip0,  vel, r,
            base0, vel, r,
            &aabbs, dt
        ) {
            // mark CCD and advance
            self.ledger.push(LedgerEvent::CCDHit { id, toi: hit.toi });

            let t   = hit.toi.clamp(0.0, 1.0);
            let n   = hit.normal;
            let mut v = vel;

            // advance to impact
            let p_impact = pose.pos + v * (t * dt);

            // remove incoming normal component (inelastic along normal)
            let vn = v.dot(n);
            if vn < 0.0 { v -= n * vn; }

            // advance remainder of the step
            let p_after = p_impact + v * (dt * (1.0 - t));

            self.bodies.set_pose(id, Isometry { pos: p_after, rot: pose.rot });
            self.bodies.set_vel (id, Velocity { lin: v, ang: self.bodies.vel(id).ang });
            return true;
        }
        false
    }

    /* ---------- Debug / helpers ---------- */
    pub fn set_debug(&mut self, cfg: DebugSettings) { self.debug = cfg; }
    pub fn get_body_pose(&self, id: BodyId) -> Isometry { self.bodies.pose(id.0) }
    pub fn get_body_vel(&self, id: BodyId) -> Velocity { self.bodies.vel(id.0) }

    pub fn add_distance_joint(&mut self, a: BodyId, b: BodyId, rest: Scalar, compliance: Scalar) -> JointId {
        self.joints.add_distance_joint(a, b, rest, compliance)
    }

    pub fn set_gravity(&mut self, g: Vec3) {
        self.gravity = g;
        self.gravity_proc = GravitySpec::Uniform { g: [g.x, g.y, g.z] };
    }
    pub fn set_epoch(&mut self, epoch: u64) { self.epoch_id = epoch; }
    pub fn set_rng_seed(&mut self, seed: u64) { self.rng = XorShift64::new(seed); }

    /// Old path: swap via EpochDescriptor (still supported)
    pub fn queue_epoch_swap(&mut self, desc: EpochDescriptor) { self.pending_epoch = Some(desc); }
    /// New path (Phase 4): swap gravity model at boundary; sets a new EpochID.
    pub fn queue_gravity_swap(&mut self, spec: GravitySpec) { self.pending_gravity = Some(spec); }

    fn apply_pending_epoch_if_any(&mut self) {
        if let Some(spec) = self.pending_gravity.take() {
            self.epoch_id = gravity_epoch_id(&spec);
            self.gravity_proc = spec;
            if let GravitySpec::Uniform { g } = spec {
                self.gravity = Vec3::new(g[0], g[1], g[2]);
            }
            return;
        }
        if let Some(desc) = self.pending_epoch.take() {
            self.epoch_id = epoch_id(&desc);
            let g = Vec3::new(desc.gravity_g[0], desc.gravity_g[1], desc.gravity_g[2]);
            self.set_gravity(g);
        }
    }

    /* ---------- World composition ---------- */
    pub fn add_body(&mut self, pose: Isometry, vel: Velocity, mass: MassProps, dynamic: bool) -> BodyId {
        let inv_mass = if dynamic { mass.inv_mass } else { 0.0 };
        let id = self.bodies.add(BodyDesc { pose, vel, inv_mass, dynamic });
        BodyId(id)
    }
    pub fn add_collider(&mut self, body: BodyId, shape: Shape, material: Material) -> ColliderId {
        let pose = self.bodies.pose(body.0);
        let aabb = aabb_of(&shape, &pose);
        let id = self.colliders.len() as u32;
        self.colliders.push(Collider { body, shape, aabb, material });
        ColliderId(id)
    }

    /* ---------- Terrain sampling (Phase 5) ---------- */
    pub fn set_heightfield(&mut self, hf: HeightField, y_offset: f32) {
        self.terrain = Some(hf);
        self.tile_cfg.y_offset = y_offset;
        self.tile_cache.clear(); // deterministic reset
    }
    pub fn sample_terrain_height_normal(&mut self, wx: f32, wz: f32) -> Option<(f32, Vec3)> {
        let hf = self.terrain.as_ref()?;
        let lx = wx; let lz = wz; // no rotation MVP

        // Tile key: each tile = tile_cells * cell_size in world units
        let tile_world_x = hf.cell.x * self.tile_cfg.tile_cells.x as f32;
        let tile_world_z = hf.cell.y * self.tile_cfg.tile_cells.y as f32;
        let tx = (lx / tile_world_x).floor() as i32;
        let tz = (lz / tile_world_z).floor() as i32;

        // Deterministic cache insert/update (BTreeMap keeps order)
        let key = (tx, tz);
        let entry = self.tile_cache.entry(key).or_default();
        entry.uses = entry.uses.wrapping_add(1);

        let h_local = hf.sample_height(lx, lz);
        let n_local = hf.sample_normal(lx, lz);
        let h_world = h_local + self.tile_cfg.y_offset;
        Some((h_world, n_local.into()))  // glam::Vec3 -> engine Vec3 (Vec3A)
    }

    /* ---------- Blade sweep helper (Phase 9) ---------- */
    pub fn sweep_blade_against_boxes(
        &self,
        tip_p0: Vec3, tip_v: Vec3, tip_r: f32,
        mid_p0: Vec3, mid_v: Vec3, mid_r: f32,
        dt: f32,
    ) -> Option<(usize, f32, Vec3)> {
        let mut boxes: Vec<Aabb> = Vec::new();
        for c in &self.colliders {
            if let Shape::Box { .. } = c.shape { boxes.push(c.aabb); }
        }
        if boxes.is_empty() { return None; }
        let hit = riftphys_melee::sweep_blade_points(tip_p0, tip_v, tip_r, mid_p0, mid_v, mid_r, &boxes, dt)?;
        Some((hit.target_index, hit.toi, hit.normal))
    }

    /* ---------- CCD: sphere vs boxes (TOI) ---------- */
    fn ccd_integrate_sphere(&mut self, id: u32, dt: Scalar) -> bool {
        // find radius for this body's sphere collider
        let mut radius = None::<f32>;
        for c in &self.colliders {
            if c.body.0 == id {
                if let Shape::Sphere { r } = c.shape { radius = Some(r); }
                break;
            }
        }
        let r = match radius { Some(r) => r, None => return false };

        // collect box AABBs
        let mut aabbs: Vec<Aabb> = Vec::new();
        for c in &self.colliders {
            if let Shape::Box { .. } = c.shape { aabbs.push(c.aabb); }
        }
        if aabbs.is_empty() { return false; }

        // current state
        let pose = self.bodies.pose(id);
        let vel  = self.bodies.vel(id).lin;

        // earliest hit across boxes
        let mut best: Option<(f32, Vec3, usize)> = None; // (toi, normal, collider_index)
        for (ci, aabb) in aabbs.iter().enumerate() {
            if let Some((t, n)) = sweep_sphere_vs_aabb(pose.pos, vel, r, aabb, dt) {
                match best {
                    None => best = Some((t, n.into(), ci)),
                    Some((bt, _, bi)) => {
                        if (t < bt - 1e-9) || ((t - bt).abs() <= 1e-9 && ci < bi) {
                            best = Some((t, n.into(), ci));
                        }
                    }
                }
            }
        }
        let (toi, normal, _hit_ci) = match best { Some(x) => x, None => return false };

        // mark CCD & advance
        self.ledger.push(LedgerEvent::CCDHit { id, toi });

        let mid_dt = (toi.clamp(0.0, 1.0)) * dt;
        let mut p = pose.pos + vel * mid_dt;

        let vn = vel.dot(normal);
        let mut new_vel = vel;
        if vn < 0.0 { new_vel -= normal * vn; }

        let rem_dt = dt - mid_dt;
        p += new_vel * rem_dt;

        self.bodies.set_pose(id, Isometry { pos: p, rot: pose.rot });
        self.bodies.set_vel(id, Velocity { lin: new_vel, ang: self.bodies.vel(id).ang });
        true
    }

    /* ---------- Step ---------- */
    pub fn step(&mut self, dt: Scalar) -> StepStats {
        self.schedule.clear();
        self.tick = self.tick.wrapping_add(1);
        self.ledger.clear();

        // boundary swaps (Phase 3/4)
        self.apply_pending_epoch_if_any();

        // Integrate (CCD-aware) — capsule first, then sphere, then fallback
        self.schedule.push(StepStage::Integrate);
        let count = self.bodies.len() as u32;
        for i in 0..count {
            if !self.bodies.is_dynamic(i) || self.bodies.inv_mass_of(i) == 0.0 { continue; }

            // Capsule CCD first (if this body has a capsule collider)
            let mut cap_rhh: Option<(f32,f32)> = None;
            for c in &self.colliders {
                if c.body.0 == i {
                    if let Shape::Capsule { r, hh } = c.shape { cap_rhh = Some((r, hh)); }
                    break;
                }
            }
            if let Some((r, hh)) = cap_rhh {
                if self.ccd_integrate_capsule(i, r, hh, dt) {
                    let a = grav_eval(&self.gravity_proc, self.bodies.pose(i).pos);
                    let dv = a * dt;
                    self.ledger.push(LedgerEvent::Integrate { id: i, a, dv });
                    continue;
                }
            }

            // Then try sphere CCD (if sphere collider is present)
            if self.ccd_integrate_sphere(i, dt) {
                let a = grav_eval(&self.gravity_proc, self.bodies.pose(i).pos);
                let dv = a * dt;
                self.ledger.push(LedgerEvent::Integrate { id: i, a, dv });
                continue;
            }

            // Fallback: uniform acceleration integrate
            let pose = self.bodies.pose(i);
            let mut vel = self.bodies.vel(i);
            let a = grav_eval(&self.gravity_proc, pose.pos);
            vel.lin += a * dt;
            let new_pos = pose.pos + vel.lin * dt;
            self.bodies.set_vel(i, vel);
            self.ledger.push(LedgerEvent::Integrate { id: i, a, dv: a * dt });
            self.bodies.set_pose(i, Isometry { pos: new_pos, rot: pose.rot });
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
            if let Some(c) = self.contact_box_box(i, j)      { contacts.push(c); continue; }
            if let Some(c) = self.contact_sphere_sphere(i, j){ contacts.push(c); continue; }
            if let Some(c) = self.contact_sphere_box(i, j)   { contacts.push(c); continue; }
            if let Some(c) = self.contact_capsule_box(i, j)  { contacts.push(c); continue; }
        }

        // --- Guard/Brace controllers: update based on contacts; set joint params ---
        for g in &mut self.guards {
            let mut hit = false;
            for c in &contacts {
                let a = self.colliders[c.a_collider].body;
                let b = self.colliders[c.b_collider].body;
                if a == g.eff || b == g.eff { hit = true; break; }
            }
            if hit { g.ctrl.on_contact(); }
            let (rest, comp) = g.ctrl.step(dt);
            // SAFE setter; no private field access
            self.joints.set_distance_params(g.joint, rest, comp);
            // Telemetry (don't peek into private joints internals)
            self.ledger.push(LedgerEvent::JointDistance {
                a: g.eff.0, b: 0, lambda: comp, compliance: comp
            });
        }

        // Joints
        self.joints.solve(&mut self.bodies, dt, 4);

        // Solve + AABB update (post)
        self.schedule.push(StepStage::Solve);
        let contacts_len = contacts.len() as u32;
        if contacts_len > 0 {
            self.solve_contacts(&contacts);
            self.schedule.push(StepStage::UpdateAabbsPost);
            for idx in 0..self.colliders.len() {
                let b = self.colliders[idx].body;
                let shape = self.colliders[idx].shape;
                let pose = self.bodies.pose(b.0);
                self.colliders[idx].aabb = aabb_of(&shape, &pose);
            }
        }

        // Debug print + JSONL dump every N ticks
        if self.debug.print_every != 0 && (self.tick as u32) % self.debug.print_every == 0 {
            self.print_debug_block(&contacts);
            let _ = self.ledger.write_jsonl("out", self.tick);
        }

        StepStats { pairs_tested: pairs.len() as u32, contacts: contacts_len, islands: 1 }
    }

    pub fn step_hash(&self) -> [u8; 32] {
        let mut h = StepHasher::new();
        h.update_bytes(&self.epoch_id.to_le_bytes());
        h.update_bytes(&self.rng.state().to_le_bytes());
        h.update_bytes(&self.schedule.digest());
        for i in self.bodies.indices() {
            let pose = self.bodies.pose(i);
            let vel  = self.bodies.vel(i);
            h.update_bytes(&i.to_le_bytes());
            hash_vec3(&mut h, &pose.pos);
            hash_quat(&mut h, &pose.rot);
            hash_vec3(&mut h, &vel.lin);
            hash_vec3(&mut h, &vel.ang);
        }
        h.finalize()
    }

    /* ---------- Contacts ---------- */
    fn contact_box_box(&self, ci: usize, cj: usize) -> Option<Contact> {
        let a = &self.colliders[ci];
        let b = &self.colliders[cj];
        match (a.shape, b.shape) {
            (Shape::Box { .. }, Shape::Box { .. }) => {}
            _ => return None,
        }
        let aa = a.aabb; let bb = b.aabb;
        if !aa.overlaps(&bb) { return None; }
        let ca = (aa.min + aa.max) * 0.5;
        let cb = (bb.min + bb.max) * 0.5;
        let px = (aa.max.x - bb.min.x).min(bb.max.x - aa.min.x);
        let py = (aa.max.y - bb.min.y).min(bb.max.y - aa.min.y);
        let pz = (aa.max.z - bb.min.z).min(bb.max.z - aa.min.z);
        let (mut normal, depth) = if px <= py && px <= pz {
            let dir = if cb.x > ca.x { 1.0 } else { -1.0 }; (Vec3::new(dir, 0.0, 0.0), px)
        } else if py <= pz {
            let dir = if cb.y > ca.y { 1.0 } else { -1.0 }; (Vec3::new(0.0, dir, 0.0), py)
        } else {
            let dir = if cb.z > ca.z { 1.0 } else { -1.0 }; (Vec3::new(0.0, 0.0, dir), pz)
        };
        if depth <= 0.0 { return None; }
        let n_len = normal.length(); if n_len == 0.0 { return None; }
        normal /= n_len;
        Some(Contact { a_collider: ci, b_collider: cj, normal, depth })
    }

    fn contact_sphere_sphere(&self, ci: usize, cj: usize) -> Option<Contact> {
        let a = &self.colliders[ci];
        let b = &self.colliders[cj];
        let (ra, rb) = match (a.shape, b.shape) {
            (Shape::Sphere { r: r1 }, Shape::Sphere { r: r2 }) => (r1, r2),
            _ => return None,
        };
        let pa = self.bodies.pose(a.body.0).pos;
        let pb = self.bodies.pose(b.body.0).pos;
        let d = pb - pa;
        let dist2 = d.length_squared();
        let rsum = ra + rb;
        if dist2 >= rsum * rsum { return None; }
        let dist = dist2.sqrt();
        let normal = if dist > 1.0e-6 { d / dist } else { Vec3::new(1.0, 0.0, 0.0) };
        let depth  = rsum - dist;
        Some(Contact { a_collider: ci, b_collider: cj, normal, depth })
    }

    fn contact_sphere_box(&self, ci: usize, cj: usize) -> Option<Contact> {
        let (si, bi, flip) = match (self.colliders[ci].shape, self.colliders[cj].shape) {
            (Shape::Sphere { .. }, Shape::Box { .. }) => (ci, cj, false),
            (Shape::Box { .. }, Shape::Sphere { .. }) => (cj, ci, true),
            _ => return None,
        };
        let s = &self.colliders[si]; let b = &self.colliders[bi];
        let r = match s.shape { Shape::Sphere { r } => r, _ => unreachable!() };
        let ps = self.bodies.pose(s.body.0).pos;
        let bb = b.aabb;
        let q = clamp_vec3(ps, bb.min, bb.max);
        let mut n = ps - q; // box -> sphere
        let dist = n.length(); if dist >= r { return None; }
        if dist > 1.0e-6 { n /= dist; } else { n = Vec3::new(0.0, 1.0, 0.0); }
        let depth = r - dist;
        let normal = if flip {  n } else { -n };
        Some(Contact { a_collider: ci, b_collider: cj, normal, depth })
    }

    fn contact_capsule_box(&self, ci: usize, cj: usize) -> Option<Contact> {
        let (cap_i, box_i, flip) = match (self.colliders[ci].shape, self.colliders[cj].shape) {
            (Shape::Capsule { .. }, Shape::Box { .. }) => (ci, cj, false),
            (Shape::Box { .. }, Shape::Capsule { .. }) => (cj, ci, true),
            _ => return None,
        };
        let cap = &self.colliders[cap_i]; let bx = &self.colliders[box_i];
        let (r, hh) = match cap.shape { Shape::Capsule { r, hh } => (r, hh), _ => unreachable!() };
        let pose = self.bodies.pose(cap.body.0);
        let pa = pose.pos + (pose.rot * Vec3::new(0.0,  hh, 0.0));
        let pb = pose.pos + (pose.rot * Vec3::new(0.0, -hh, 0.0));
        let bb = bx.aabb;
        let (p_seg, p_box) = closest_points_segment_aabb(pa, pb, bb.min, bb.max);
        let mut n = p_seg - p_box; // box -> capsule axis
        let dist = n.length(); if dist >= r { return None; }
        if dist > 1.0e-6 { n /= dist; } else { n = Vec3::new(0.0, 1.0, 0.0); }
        let depth = r - dist;
        let normal = if flip {  n } else { -n };
        Some(Contact { a_collider: ci, b_collider: cj, normal, depth })
    }

    /* ---------- Solver (normal + friction) ---------- */
    fn solve_contacts(&mut self, contacts: &[Contact]) {
        let iterations = 8;
        let slop = 0.001;
        let beta = 0.8;

        for _ in 0..iterations {
            for c in contacts {
                let ai = self.colliders[c.a_collider].body.0;
                let bi = self.colliders[c.b_collider].body.0;
                if ai == bi { continue; }

                let inv_a = self.bodies.inv_mass_of(ai);
                let inv_b = self.bodies.inv_mass_of(bi);
                if inv_a + inv_b == 0.0 { continue; }

                let ma = self.colliders[c.a_collider].material;
                let mb = self.colliders[c.b_collider].material;
                // Combine however you prefer; default uses collider fields:
                let restitution = ma.restitution.max(mb.restitution);
                let mu_s = (ma.mu_s * mb.mu_s).abs().sqrt();
                let mu_k = (ma.mu_k * mb.mu_k).abs().sqrt();

                let va = self.bodies.vel(ai);
                let vb = self.bodies.vel(bi);
                let n  = c.normal;
                let rel_v_n = (vb.lin - va.lin).dot(n);

                // Normal impulse
                let mut jn = 0.0;
                if rel_v_n < 0.0 {
                    jn = -(1.0 + restitution) * rel_v_n / (inv_a + inv_b);
                    let imp_n = n * jn;
                    self.bodies.apply_impulse(ai, -imp_n);
                    self.bodies.apply_impulse(bi,  imp_n);
                    self.ledger.push(LedgerEvent::ImpulseN { a: ai, b: bi, jn });
                }

                // Positional correction (split impulse style)
                let corr = (c.depth - slop).max(0.0) * beta;
                if corr > 0.0 {
                    let denom = inv_a + inv_b;
                    let corr_vec = n * (corr / denom);
                    self.bodies.apply_position_delta(ai, -corr_vec * inv_a);
                    self.bodies.apply_position_delta(bi,  corr_vec * inv_b);
                    self.ledger.push(LedgerEvent::PosCorr { a: ai, b: bi, corr });
                }

                // Friction (2 tangents)
                if jn > 0.0 || c.depth > slop {
                    let va2 = self.bodies.vel(ai);
                    let vb2 = self.bodies.vel(bi);
                    let vrel = vb2.lin - va2.lin;
                    let v_n = n * vrel.dot(n);
                    let v_t = vrel - v_n;

                    let (t1, t2) = orthonormal_basis(n);
                    let vt1 = v_t.dot(t1);
                    let vt2 = v_t.dot(t2);

                    let denom = inv_a + inv_b;
                    if denom > 0.0 {
                        let jt1_des = -vt1 / denom;
                        let jt2_des = -vt2 / denom;
                        let jt_des_len = (jt1_des * jt1_des + jt2_des * jt2_des).sqrt();
                        let jt_max_static = mu_s * jn;

                        let (jt1, jt2) = if jt_des_len <= jt_max_static || jn == 0.0 {
                            (jt1_des, jt2_des)
                        } else {
                            let jt_max_kin = mu_k * jn;
                            let scale = if jt_des_len > 1.0e-9 { jt_max_kin / jt_des_len } else { 0.0 };
                            (jt1_des * scale, jt2_des * scale)
                        };

                        let jt_vec = t1 * jt1 + t2 * jt2;
                        self.bodies.apply_impulse(ai, -jt_vec);
                        self.bodies.apply_impulse(bi,  jt_vec);
                        self.ledger.push(LedgerEvent::ImpulseT { a: ai, b: bi, jt1, jt2 });
                    }
                }
            }
        }
    }

    /* ---------- Debug printer ---------- */
    fn print_debug_block(&self, contacts: &[Contact]) {
        println!("--- debug @ tick {}  epoch=0x{:016x} ---", self.tick, self.epoch_id);

        if self.debug.show_energy {
            let mut ke = 0.0f32;
            for i in 0..(self.bodies.len() as u32) {
                let im = self.bodies.inv_mass_of(i);
                if im > 0.0 {
                    let m = 1.0 / im;
                    let v = self.bodies.vel(i).lin;
                    ke += 0.5 * m * v.length_squared();
                }
            }
            println!("energy: KE_total = {:.6}", ke);
        }

        if self.debug.show_bodies {
            let mut lines = 0usize;
            for i in 0..(self.bodies.len() as u32) {
                let p = self.bodies.pose(i).pos;
                let v = self.bodies.vel(i).lin;
                println!("body {:3}  pos=({:+.3},{:+.3},{:+.3})  vel=({:+.3},{:+.3},{:+.3})",
                         i, p.x,p.y,p.z, v.x,v.y,v.z);
                lines += 1; if lines >= self.debug.max_lines { break; }
            }
        }

        if self.debug.show_contacts {
            if contacts.is_empty() {
                println!("contacts: (none)");
            } else {
                let mut shown = 0usize;
                for c in contacts.iter() {
                    println!("contact  cA={} cB={}  n=({:+.3},{:+.3},{:+.3})  depth={:.5}",
                             c.a_collider, c.b_collider, c.normal.x, c.normal.y, c.normal.z, c.depth);
                    shown += 1; if shown >= self.debug.max_lines { break; }
                }
            }
        }
    }
}

/* ---------- helpers ---------- */
#[inline] fn clampf(x: f32, lo: f32, hi: f32) -> f32 { x.max(lo).min(hi) }
#[inline] fn clamp_vec3(p: Vec3, mn: Vec3, mx: Vec3) -> Vec3 {
    Vec3::new(clampf(p.x, mn.x, mx.x), clampf(p.y, mn.y, mx.y), clampf(p.z, mn.z, mx.z))
}
#[inline]
fn closest_point_on_segment(a: Vec3, b: Vec3, p: Vec3) -> (Vec3, f32) {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    (a + ab * t, t)
}
fn closest_points_segment_aabb(a: Vec3, b: Vec3, mn: Vec3, mx: Vec3) -> (Vec3, Vec3) {
    let mut ps = (a + b) * 0.5;
    let mut qs = clamp_vec3(ps, mn, mx);
    for _ in 0..3 {
        let (p2, _t) = closest_point_on_segment(a, b, qs);
        ps = p2;
        qs = clamp_vec3(ps, mn, mx);
    }
    (ps, qs)
}
fn orthonormal_basis(n: Vec3) -> (Vec3, Vec3) {
    let ax = n.x.abs(); let ay = n.y.abs(); let az = n.z.abs();
    let base = if ax <= ay && ax <= az { Vec3::new(1.0, 0.0, 0.0) }
    else if ay <= az        { Vec3::new(0.0, 1.0, 0.0) }
    else                    { Vec3::new(0.0, 0.0, 1.0) };
    let t1 = (base.cross(n)).normalize_or_zero();
    let t2 = n.cross(t1);
    (t1, t2)
}
