mod det_harness;
use riftphys_controllers::{GuardCtrl, BalanceCtrl, BalanceParams};

use riftphys_core::{
    Scalar, Vec3, Isometry, Velocity, BodyId, ColliderId, StepStats, StepHasher, hash_vec3, hash_quat,
    StepStage, XorShift64, EpochDescriptor, epoch_id, JointId,
};
use riftphys_geom::{Aabb, Shape, MassProps, Material, aabb_of};
use riftphys_melee::sweep_capsule_vs_aabb_two_spheres;
use riftphys_materials as mats;
use riftphys_collision::pairs_sap;
use riftphys_dynamics::{Bodies, BodyDesc};
use riftphys_viz::{ScheduleRecorder, DebugSettings, Ledger, LedgerEvent};
use riftphys_gravity::{GravitySpec, eval as grav_eval, spec_id as gravity_epoch_id};
use riftphys_articulation::{Joints, D6Joint};
use crate::det_harness::{SimWorld, StepReport, Inputs, InputEvent};
use riftphys_core::vec3;
use glam::Quat; // for from_rotation_y

use std::collections::BTreeMap;
use riftphys_terrain::HeightField;
use riftphys_melee::sweep_sphere_vs_aabb;
use riftphys_core::models::{
    ModelRegistry, AccelPackHandle, AeroHandle, PropHandle, AeroQuery, PropQuery,
};
use riftphys_core::{StepCtx, EpochId};
use riftphys_vehicles::{VehicleInstance, VehicleParams, WheelParams, AxleInput};

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
struct BalanceInstance {
    pelvis: BodyId,
    left: BodyId,
    right: BodyId,
    ctrl: BalanceCtrl }

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
        self.bodies = bodies;
        self.colliders = colliders;
        self
    }

    pub fn build(self) -> World {
        World::with_capacity(self.bodies, self.colliders)
    }
}
#[derive(Copy, Clone, Debug)]
struct AccelComp {
    aero: Option<AeroHandle>,
    prop: Option<PropHandle>,
    ref_area_m2: f32,
    throttle01: f32,
    // If you want per-body forward override; otherwise we infer from pose:
    forward_dir_world: Option<Vec3>, // default: +X body rotated to world
}
#[derive(Copy, Clone, Default)]
struct WarmImp { jn: f32, jt1: f32, jt2: f32 }

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
    models: ModelRegistry,
    accel_comps: Vec<Option<AccelComp>>,
    balances: Vec<BalanceInstance>,

    warm_cache: BTreeMap<(u32, u32), WarmImp>,
    last_normal_impulse: Vec<f32>,
}

impl World {
    // Read-only helpers for the viewer/debuggers.
    pub fn num_bodies(&self) -> u32 { self.bodies.len() as u32 }
    pub fn primary_shape(&self, body: BodyId) -> Option<riftphys_geom::Shape> {
        for c in &self.colliders {
            if c.body == body {
                return Some(c.shape);
            }
        }
        None
    }
    #[inline] pub fn tick_index(&self) -> u64 { self.tick }
    pub fn for_each_collider<F: FnMut(u32, BodyId, &Shape, &Aabb)>(&self, mut f: F) {
        for (i, c) in self.colliders.iter().enumerate() {
            f(i as u32, c.body, &c.shape, &c.aabb);
        }
    }

    // Handy single-pose reader for generic viewers.
    pub fn body_pose(&self, id: BodyId) -> Isometry { self.bodies.pose(id.0) }

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
            models: ModelRegistry::new(),
            accel_comps: vec![None; bodies],
            balances: Vec::new(),
            warm_cache: BTreeMap::new(),
            last_normal_impulse: vec![0.0; bodies],
        }
    }
    pub fn add_balance_controller(
        &mut self,
        pelvis: BodyId,
        left: BodyId,
        right: BodyId,
        params: BalanceParams,
    ) {
        self.balances.push(BalanceInstance {
            pelvis, left, right, ctrl: BalanceCtrl::new(params)
        });
    }
    /// Access the registry to register models from benches or game init.
    pub fn models_mut(&mut self) -> &mut ModelRegistry { &mut self.models }
    pub fn models(&self) -> &ModelRegistry { &self.models }

    /// Attach/replace an AccelComp on a body.
    pub fn set_body_accel(
        &mut self,
        body: BodyId,
        aero: Option<AeroHandle>,
        prop: Option<PropHandle>,
        ref_area_m2: f32,
        throttle01: f32,
        forward_dir_world: Option<Vec3>,
    ) {
        let i = body.0 as usize;
        if self.accel_comps.len() <= i { self.accel_comps.resize(i + 1, None); }
        self.accel_comps[i] = Some(AccelComp {
            aero, prop, ref_area_m2, throttle01, forward_dir_world,
        });
    }
    /// Deterministically set a body's pose at a tick boundary.
    /// Call only outside `World::step()` (e.g., before the step) to keep hashes stable.
    pub fn set_body_pose(&mut self, id: BodyId, pose: Isometry) {
        self.bodies.set_pose(id.0, pose);

        // Optional immediate AABB refresh so debug/viewers see it right away.
        // (It will be recomputed again in UpdateAabbsPre during the step.)
        for c in &mut self.colliders {
            if c.body == id {
                c.aabb = aabb_of(&c.shape, &pose);
            }
        }
    }
    /// Convenience: rotate about +Y by `dy` radians (pose write).
    /// Use this at double-stance frames so both ACTIVE/SHADOW apply the same delta.
    pub fn yaw_body(&mut self, id: BodyId, dy: f32) {
        if dy == 0.0 { return; }
        let mut p = self.bodies.pose(id.0);
        let dq = Quat::from_rotation_y(dy);
        p.rot = (dq * p.rot).normalize();
        self.set_body_pose(id, p);
    }
    /// Optional convenience to update throttle per tick from inputs.
    pub fn set_body_throttle(&mut self, body: BodyId, t: f32) {
        if let Some(Some(ac)) = self.accel_comps.get_mut(body.0 as usize) {
            ac.throttle01 = t.clamp(0.0, 1.0);
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

        // advance remainder of the step with gravity + aero/prop for this tick
        let (a_grav, a_extra) = Self::eval_extra_accel(&self.models, &self.bodies, &self.accel_comps,
                self.epoch_id, self.tick, id, pose, self.bodies.vel(id), &self.gravity_proc, dt);
        let a_total = a_grav + a_extra;
        v += a_total * dt; // simple: apply full-tick accel; we already resolved TOI once
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
            self.warm_cache.clear();
        }
        if let Some(desc) = self.pending_epoch.take() {
            self.epoch_id = epoch_id(&desc);
            let g = Vec3::new(desc.gravity_g[0], desc.gravity_g[1], desc.gravity_g[2]);
            self.set_gravity(g);
            self.warm_cache.clear();
        }

    }

    /* ---------- World composition ---------- */
    pub fn add_body(&mut self, pose: Isometry, vel: Velocity, mass: MassProps, dynamic: bool) -> BodyId {
        let inv_mass = if dynamic { mass.inv_mass } else { 0.0 };
        let id = self.bodies.add(BodyDesc { pose, vel, inv_mass, dynamic });
        if self.accel_comps.len() <= id as usize {
            self.accel_comps.resize(id as usize + 1, None);
        }

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
    #[inline]
    fn eval_extra_accel(models: &ModelRegistry,
                        bodies: &Bodies,
                        accel_comps: &Vec<Option<AccelComp>>,
                        epoch_id: u64,
                        tick: u64,
                        i: u32,
                        pose: Isometry,
                        vel: Velocity,
                        gravity_proc: &GravitySpec,
                        dt: f32) -> (Vec3, Vec3) {
        // returns (a_grav, a_extra) where a_extra = aero + prop like the fallback path
        let mut a_grav = grav_eval(gravity_proc, pose.pos);
        let mut a_extra = Vec3::ZERO;

        if let Some(Some(ac)) = accel_comps.get(i as usize) {
            let inv_m = bodies.inv_mass_of(i);
            if inv_m > 0.0 {
                let mass = 1.0 / inv_m;
                let fwd_w = ac.forward_dir_world.unwrap_or_else(|| pose.rot * Vec3::new(1.0, 0.0, 0.0));
                let v_w = vel.lin;
                let speed = v_w.length();
                let vhat = if speed > 1e-6 { v_w / speed } else { fwd_w };
                let cos_a = fwd_w.dot(vhat).clamp(-1.0, 1.0);
                let alpha_rad = cos_a.acos();
                let aq = AeroQuery {
                    vel_world: [v_w.x, v_w.y, v_w.z],
                    ang_vel_world: [vel.ang.x, vel.ang.y, vel.ang.z],
                    orientation: pose.rot,
                    area: ac.ref_area_m2,
                    mass,
                    altitude: pose.pos.y,
                    alpha_rad,
                };
                let pq = PropQuery {
                    throttle01: ac.throttle01,
                    forward_dir_world: [fwd_w.x, fwd_w.y, fwd_w.z],
                    mass,
                };
                if let Some(h) = ac.aero {
                    let a = models.aero(h).accel_contrib(&StepCtx { dt, tick, epoch: EpochId(epoch_id) }, aq);
                    a_extra += Vec3::new(a[0], a[1], a[2]);
                }
                if let Some(h) = ac.prop {
                    let a = models.prop(h).accel_contrib(&StepCtx { dt, tick, epoch: EpochId(epoch_id) }, pq);
                    a_extra += Vec3::new(a[0], a[1], a[2]);
                }
            }
        }
        (a_grav, a_extra)
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

       let (a_grav, a_extra) = Self::eval_extra_accel(&self.models, &self.bodies, &self.accel_comps,
                                                self.epoch_id, self.tick, id, pose, self.bodies.vel(id), &self.gravity_proc, dt);
        let a_total = a_grav + a_extra;
        let rem_dt = dt - mid_dt;
         new_vel += a_total * dt;
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
                    // self.ledger.push(LedgerEvent::Integrate { id: i, a, dv });
                    continue;
                }
            }

            // Then try sphere CCD (if sphere collider is present)
            if self.ccd_integrate_sphere(i, dt) {
                let a = grav_eval(&self.gravity_proc, self.bodies.pose(i).pos);
                let dv = a * dt;
                // self.ledger.push(LedgerEvent::Integrate { id: i, a, dv });
                continue;
            }

            // Fallback: uniform acceleration integrate + Phase 11 accel pass
            let pose = self.bodies.pose(i);
            let mut vel = self.bodies.vel(i);

            // Gravity (existing)
            // 1) Gravity magnitude from the active spec
            let mut a_grav = grav_eval(&self.gravity_proc, pose.pos);
            let gmag = a_grav.length();

            if gmag > 1.0e-6 {
                // 2) If we are near a STATIC sphere collider (planet “ground”), steer gravity to radial “down”
                let mut best_gap: f32 = f32::INFINITY;
                let mut radial_down: Option<Vec3> = None;

                for col in &self.colliders {
                    if let Shape::Sphere { r } = col.shape {
                        // treat only static spheres as planets
                        if self.bodies.inv_mass_of(col.body.0) == 0.0 {
                            let center = self.bodies.pose(col.body.0).pos;
                            let rv = pose.pos - center;
                            let dist = rv.length();
                            if dist > 1.0e-6 {
                                let gap = dist - r;
                                if gap < best_gap && gap < 5.0 {
                                    best_gap = gap;
                                    radial_down = Some((-rv / dist).normalize()); // toward center
                                }
                            }
                        }
                    }
                }

                if let Some(rd) = radial_down {
                    // lock direction to planet radial; keep the magnitude from spec
                    a_grav = rd * gmag;
                } else if let Some((h_world, n_hf)) = self.sample_terrain_height_normal(pose.pos.x, pose.pos.z) {
                    // 3) Not near a planet sphere: blend toward HF normal in a ~50 m band above surface
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


            // Phase 11 Accel (post-gravity): aero + propulsion if present
            let mut a_extra = Vec3::ZERO;
            if let Some(Some(ac)) = self.accel_comps.get(i as usize) {
                // Mass from inv_mass; skip statics
                let inv_m = self.bodies.inv_mass_of(i);
                if inv_m > 0.0 {
                    let mass = 1.0 / inv_m;

                    // Forward dir (world): default is +X in body rotated to world
                    let fwd_w = ac.forward_dir_world.unwrap_or_else(|| pose.rot * Vec3::new(1.0, 0.0, 0.0));

                    // Velocity info
                    let v_w = vel.lin;
                    let speed = v_w.length();
                    let vhat = if speed > 1e-6 { v_w / speed } else { fwd_w };

                    // Very simple α: angle between forward and velocity
                    let cos_a = fwd_w.dot(vhat).clamp(-1.0, 1.0);
                    let alpha_rad = cos_a.acos();

                    // Altitude proxy = world Y
                    let altitude = pose.pos.y;

                    // Build queries
                    let aq = AeroQuery {
                        vel_world: [v_w.x, v_w.y, v_w.z],
                        ang_vel_world: [vel.ang.x, vel.ang.y, vel.ang.z],
                        orientation: pose.rot,
                        area: ac.ref_area_m2,
                        mass,
                        altitude,
                        alpha_rad,
                    };
                    let pq = PropQuery {
                        throttle01: ac.throttle01,
                        forward_dir_world: [fwd_w.x, fwd_w.y, fwd_w.z],
                        mass,
                    };

                    // Evaluate models (split for telemetry)
                    let mut a_aero = Vec3::ZERO;
                    let mut a_prop = Vec3::ZERO;

                    if let Some(h) = ac.aero {
                        let a = self.models.aero(h).accel_contrib(
                            &StepCtx { dt, tick: self.tick, epoch: EpochId(self.epoch_id) }, aq);
                        a_aero = Vec3::new(a[0], a[1], a[2]);
                    }
                    if let Some(h) = ac.prop {
                        let a = self.models.prop(h).accel_contrib(
                            &StepCtx { dt, tick: self.tick, epoch: EpochId(self.epoch_id) }, pq);
                        a_prop = Vec3::new(a[0], a[1], a[2]);
                    }

                    // Telemetry: estimate thrust/drag forces from the actual accelerations
                    let v = vel.lin;
                    let speed = v.length();
                    if speed > 1e-6 {
                        let vhat = v / speed;
                        let a_drag_along = -a_aero.dot(vhat);   // >0 when opposing velocity
                        let t_est_n = a_prop.length() * mass;   // thrust ≈ |a_prop| * m
                        let d_est_n = (a_drag_along.max(0.0)) * mass; // drag ≈ opposing aero * m
                        self.ledger.push(LedgerEvent::AeroProp { id: i, t_n: t_est_n, d_n: d_est_n, speed });
                    }

                    // Accumulate into the integrator
                    a_extra += a_aero + a_prop;

                }
            }

            let a_total = a_grav + a_extra;
            vel.lin += a_total * dt;
            let new_pos = pose.pos + vel.lin * dt;

            self.bodies.set_vel(i, vel);
            self.ledger.push(LedgerEvent::Integrate { id: i, a: a_total, dv: a_total * dt });
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
        // ---- Deterministic cull: keep at most 4 contacts per collider pair
        let axis_code = |n: riftphys_core::Vec3| -> u8 {
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
                let ac1 = axis_code(c1.normal); let ac2 = axis_code(c2.normal);
                ac1.cmp(&ac2)
                    .then_with(|| c2.depth.partial_cmp(&c1.depth).unwrap()) // deeper first
                    .then_with(|| c1.a_collider.cmp(&c2.a_collider))
                    .then_with(|| c1.b_collider.cmp(&c2.b_collider))
            });
            v.truncate(4);
            contacts_culled.extend(v.into_iter());
        }
        let mut contacts = contacts_culled;
        // Ensure final orientation is A -> B (robust against future edits)
        for c in &mut contacts {
            let a = self.colliders[c.a_collider].body;
            let b = self.colliders[c.b_collider].body;
            let pa = self.bodies.pose(a.0).pos;
            let pb = self.bodies.pose(b.0).pos;
            if c.normal.dot(pb - pa) < 0.0 {
                c.normal = -c.normal;
            }
        }

        // ---- Quantize normals and depths (kill ulp jitter)
        let q = 1.0e-6f32;
        for c in &mut contacts {
            let x = (c.normal.x / q).round() * q;
            let y = (c.normal.y / q).round() * q;
            let z = (c.normal.z / q).round() * q;
            let len = (x*x + y*y + z*z).sqrt();
            c.normal = if len > 1.0e-20 { riftphys_core::vec3(x/len, y/len, z/len) } else { riftphys_core::vec3(0.0, 1.0, 0.0) };
            c.depth  = (c.depth / q).round() * q;
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
        // --- Balance controllers (Phase 13) ---
        // We compute a deterministic support point: mean(XZ) of feet that are in contact this tick.
        // If no foot contacts, we fall back to previous pose (implicitly: zero accel).
        for b in &mut self.balances {
            // 1) detect if either foot is in this tick's contacts (XZ support center)
            let mut acc = 0usize;
            let mut sx = 0.0f32;
            let mut sz = 0.0f32;
            for c in &contacts {
                let a = self.colliders[c.a_collider].body;
                let bdy = self.colliders[c.b_collider].body;
                if a == b.left || a == b.right {
                    let p = self.bodies.pose(a.0).pos;
                    sx += p.x; sz += p.z; acc += 1;
                }
                if bdy == b.left || bdy == b.right {
                    let p = self.bodies.pose(bdy.0).pos;
                    sx += p.x; sz += p.z; acc += 1;
                }
            }
            if acc == 0 { continue; } // no support this tick → do nothing (deterministic)

            let support_xz = glam::Vec2::new(sx / acc as f32, sz / acc as f32);
            let pelvis_p   = self.bodies.pose(b.pelvis.0).pos;
            let a_xz       = b.ctrl.step(glam::Vec3::new(pelvis_p.x, pelvis_p.y, pelvis_p.z), support_xz);

            // 2) apply as horizontal acceleration to pelvis (deterministically)
            let mut v = self.bodies.vel(b.pelvis.0);
            v.lin.x += a_xz.x * dt;
            v.lin.z += a_xz.y * dt;

            // 3) telemetry (deterministic, controller-specific)
            self.bodies.set_vel(b.pelvis.0, v);
            self.ledger.push(LedgerEvent::BalanceAccel {
                id:  b.pelvis.0,
                ax:  a_xz.x,
                az:  a_xz.y,
            });

            // 3) extra telemetry (optional)
            self.ledger.push(LedgerEvent::Integrate {
                id: b.pelvis.0,
                a: Vec3::new(a_xz.x, 0.0, a_xz.y),
                dv: Vec3::new(a_xz.x * dt, 0.0, a_xz.y * dt),
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
        {
            let mut touching = vec![false; self.bodies.len() as usize];
            for c in &contacts {
                touching[self.colliders[c.a_collider].body.0 as usize] = true;
                touching[self.colliders[c.b_collider].body.0 as usize] = true;
            }
            // for i in 0..(self.bodies.len() as u32) {
            //     if self.bodies.is_dynamic(i) && touching[i as usize] {
            //         let mut v = self.bodies.vel(i);
            //         if v.lin.length_squared() < 5.0e-10 { v.lin = Vec3::ZERO; }
            //         if v.ang.length_squared() < 5.0e-7 { v.ang = Vec3::ZERO; }
            //         self.bodies.set_vel(i, v);
            //     }
            // }
        }
        // Debug print + JSONL dump every N ticks
        if self.debug.print_every != 0 && (self.tick as u32) % self.debug.print_every == 0 {
            self.print_debug_block(&contacts);
            let _ = self.ledger.write_jsonl("out", self.tick);
        }
        if self.debug.json_every != 0 && (self.tick as u32) % self.debug.json_every == 0 {
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
        // n is BOX -> SPHERE; A is the SPHERE, B is the BOX
        let normal = -n;  // always A( sphere ) -> B( box )
        Some(Contact { a_collider: si, b_collider: bi, normal, depth })

    }
    pub fn set_body_vel(&mut self, id: BodyId, vel: Velocity) {
        self.bodies.set_vel(id.0, vel);
    }
    #[inline]
    pub fn normal_force(&self, body: BodyId, dt: f32) -> f32 {
        // impulse (N·s) / dt ≈ average normal force over the step
        self.last_normal_impulse.get(body.0 as usize).copied().unwrap_or(0.0) / dt
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
        // n is BOX -> CAPSULE; A is the CAPSULE, B is the BOX
        let normal = -n;  // always A( capsule ) -> B( box )
        Some(Contact { a_collider: cap_i, b_collider: box_i, normal, depth })

    }

    pub fn add_ball_joint(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry) -> JointId {
        self.joints.add_ball(a, b, fa, fb)
    }

    pub fn add_hinge_joint(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, hinge_axis: usize) -> JointId {
        self.joints.add_hinge(a, b, fa, fb, hinge_axis)
    }

    pub fn add_d6_joint(&mut self, j: D6Joint) -> JointId {
        self.joints.add_d6(j)
    }
    /* ---------- Solver (normal + friction) ---------- */
    fn solve_contacts(&mut self, contacts: &[Contact]) {
        let iterations = 12;
        let slop = 0.010;
        let beta = 0.10;

        // Build warmstart vector aligned with `contacts` order
        let mut warms: Vec<WarmImp> = Vec::with_capacity(contacts.len());
        for c in contacts {
            let key = if c.a_collider <= c.b_collider {
                (c.a_collider as u32, c.b_collider as u32)
            } else {
                (c.b_collider as u32, c.a_collider as u32)
            };
            warms.push(*self.warm_cache.get(&key).unwrap_or(&WarmImp::default()));
        }
        for v in &mut self.last_normal_impulse {
            *v = 0.0;
        }
        // Track impulses to store for next tick (final iteration values)
        let mut next_warms: Vec<WarmImp> = vec![WarmImp::default(); contacts.len()];

        for it in 0..iterations {
            for (idx, c) in contacts.iter().enumerate() {
                let ai = self.colliders[c.a_collider].body.0;
                let bi = self.colliders[c.b_collider].body.0;
                if ai == bi { continue; }

                let inv_a = self.bodies.inv_mass_of(ai);
                let inv_b = self.bodies.inv_mass_of(bi);
                if inv_a + inv_b == 0.0 { continue; }

                if it == 0 {
                    let w = warms[idx];
                    if w.jn != 0.0 || w.jt1 != 0.0 || w.jt2 != 0.0 {
                        let n = c.normal;
                        let (t1, t2) = orthonormal_basis(n);
                        let imp = n * w.jn + t1 * w.jt1 + t2 * w.jt2;
                        self.bodies.apply_impulse(ai, -imp);
                        self.bodies.apply_impulse(bi,  imp);
                    }
                }

                // Effective pair properties (order-independent, deterministic)
                let ma = self.colliders[c.a_collider].material;
                let mb = self.colliders[c.b_collider].material;
                let pair = mats::pair_props(ma.id, mb.id);

                // Keep restitution from the pair
                let restitution = pair.restitution;

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
                    // Accumulate magnitude for a quick “normal force” proxy (per body, this tick)
                    self.last_normal_impulse[ai as usize] += jn.max(0.0);
                    self.last_normal_impulse[bi as usize] += jn.max(0.0);

                    self.ledger.push(LedgerEvent::ImpulseN { a: ai, b: bi, jn });
                }
                next_warms[idx].jn = jn;

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
                    // relative velocity split
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
                        // desired impulses that would zero tangential velocity
                        let jt1_des = -vt1 / denom;
                        let jt2_des = -vt2 / denom;
                        let jt_des_len = (jt1_des * jt1_des + jt2_des * jt2_des).sqrt();

                        // --- NEW: speed-dependent kinetic coefficient (Stribeck), static cone from pair.mu_s
                        let vt_mag = v_t.length();
                        let mu_k_eff = mats::mu_dynamic(&pair, vt_mag); // internally quantized to 1e-6
                        let jt_max_static = pair.mu_s * jn;

                        let (jt1, jt2) = if jt_des_len <= jt_max_static || jn == 0.0 {
                            // stick region
                            (jt1_des, jt2_des)
                        } else {
                            // slip region capped by kinetic cone
                            let jt_max_kin = mu_k_eff * jn;
                            let scale = if jt_des_len > 1.0e-9 { jt_max_kin / jt_des_len } else { 0.0 };
                            (jt1_des * scale, jt2_des * scale)
                        };

                        // warmstart record
                        next_warms[idx].jt1 = jt1;
                        next_warms[idx].jt2 = jt2;

                        // apply
                        let jt_vec = t1 * jt1 + t2 * jt2;
                        self.bodies.apply_impulse(ai, -jt_vec);
                        self.bodies.apply_impulse(bi,  jt_vec);
                        self.ledger.push(LedgerEvent::ImpulseT { a: ai, b: bi, jt1, jt2 });
                    }
                }

            }
        }

        // ----- write warmstart cache for next tick (once, after last iteration) -----
        let mut new_cache: BTreeMap<(u32,u32), WarmImp> = BTreeMap::new();
        for (idx, c) in contacts.iter().enumerate() {
            let key = if c.a_collider <= c.b_collider {
                (c.a_collider as u32, c.b_collider as u32)
            } else {
                (c.b_collider as u32, c.a_collider as u32)
            };
            new_cache.insert(key, next_warms[idx]);
        }
        self.warm_cache = new_cache;
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
    fn provenance_sums(&self) -> (f32, u32, f32, f32) {
        let mut imp = 0.0f32;
        let mut ccd = 0u32;
        let mut aero = 0.0f32;
        let mut prop = 0.0f32;

        // If Ledger has no iterator yet, expose one (e.g., `pub fn iter(&self)->impl Iterator<Item=&LedgerEvent>`).
        for e in self.ledger.iter() {
            use riftphys_viz::LedgerEvent::*;
            match *e {
                CCDHit { .. } => ccd += 1,
                ImpulseN { jn, .. } => imp += jn.abs(),
                ImpulseT { jt1, jt2, .. } => imp += (jt1*jt1 + jt2*jt2).sqrt(),
                AeroProp { t_n, d_n, .. } => { prop += t_n; aero += d_n; }
                BalanceAccel { ax, az, .. } => {
                    // fold balance accel magnitude into “prop” so we can watch it easily
                    // or keep a separate field if you prefer
                    prop += (ax*ax + az*az).sqrt();
                }
                _ => {}
            }
        }

        (imp, ccd, aero, prop)
    }
}
// ---- glue: adapt World to the harness surface ----
impl crate::det_harness::types::SimWorld for World {
    fn step_dt(&mut self, dt: f32) -> crate::det_harness::types::StepReport {
        let stats = self.step(dt);                      // ← don’t forget this call
        let (imp, ccd, aero, prop) = self.provenance_sums();
        crate::det_harness::types::StepReport {
            dt, epoch: self.epoch_id, hash: self.step_hash(),
            pairs_tested: stats.pairs_tested,
            contacts:     stats.contacts,
            impulses_sum: imp,
            ccd_hits:     ccd,
            aero_sum:     aero,
            prop_sum:     prop,
        }
    }


    fn epoch_id(&self) -> u64      { self.epoch_id }
    fn step_hash(&self) -> [u8;32] { self.step_hash() }

    fn apply_inputs(&mut self, inputs: &crate::det_harness::types::Inputs) {
        use crate::det_harness::types::InputEvent::*;
        for ev in &inputs.events {
            match *ev {
                SetThrottle { body, throttle01 } => {
                    self.set_body_throttle(body, throttle01);
                }
                SetVelocity { body, lin, ang } => {
                    self.set_body_vel(body, riftphys_core::Velocity {
                        lin: riftphys_core::vec3(lin[0], lin[1], lin[2]),
                        ang: riftphys_core::vec3(ang[0], ang[1], ang[2]),
                    });
                }
                SetBodyAccel { body, aero, prop, ref_area, throttle01 } => {
                    self.set_body_accel(body, aero, prop, ref_area, throttle01, None);
                }
                GravityLayeredPlanet { surface_g, radius, center, min_r } => {
                    self.queue_gravity_swap(riftphys_gravity::GravitySpec::LayeredPlanet {
                        surface_g, radius, center, min_r
                    });
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
