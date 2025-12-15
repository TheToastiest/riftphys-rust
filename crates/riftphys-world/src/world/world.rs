// crates/riftphys-world/src/world/world.rs

use glam::UVec2;

use riftphys_controllers::{BalanceCtrl, BalanceParams, GuardCtrl};
use riftphys_core::{
    epoch_id,
    BodyId,
    ColliderId,
    EpochDescriptor,
    EpochId,
    Isometry,
    JointId,
    Quat,
    Scalar,
    StepCtx,
    Velocity,
    Vec3,
    XorShift64,
};
use riftphys_core::models::{AeroHandle, AeroQuery, ModelRegistry, PropHandle, PropQuery};

use riftphys_articulation::{D6Joint, Joints};

use riftphys_dynamics::{Bodies, BodyDesc};
use riftphys_geom::{aabb_of, Aabb, MassProps, Shape};
use riftphys_gravity::{eval as grav_eval, spec_id as gravity_epoch_id, GravitySpec};

use riftphys_terrain::HeightField;
use riftphys_viz::{DebugSettings, Ledger, LedgerEvent, ScheduleRecorder};

use std::collections::BTreeMap;

// NOTE: This was used by "expand by min(|v|*dt, CCD_MAX_MARGIN)" style logic.
// If/when you move fully to swept-AABB union logic in step.rs, you can delete this.
pub(super) const CCD_MAX_MARGIN: f32 = 1.0e-0;

/* ---------------- Controllers ---------------- */

pub(super) struct GuardInstance {
    pub(super) joint: JointId,
    pub(super) eff: BodyId, // eff = shield body
    pub(super) ctrl: GuardCtrl,
}

#[derive(Copy, Clone, Debug)]
pub(super) struct TerrainTileCfg {
    pub(super) tile_cells: UVec2,
    pub(super) y_offset: f32,
    pub(super) origin: [f32; 2], // world-space (x,z) of this tile's (0,0) sample
}

#[derive(Clone, Debug, Default)]
pub(super) struct TileEntry {
    pub(super) uses: u32,
}

pub(super) struct BalanceInstance {
    pub(super) pelvis: BodyId,
    pub(super) left: BodyId,
    pub(super) right: BodyId,
    pub(super) ctrl: BalanceCtrl,
}

/* ---------------- Collider & Contact ---------------- */

#[derive(Copy, Clone, Debug)]
pub struct Collider {
    pub body: BodyId,
    pub shape: Shape,
    pub aabb: Aabb,
    pub material: riftphys_materials::materials::Material,
}

#[derive(Copy, Clone, Debug)]
pub(super) struct Contact {
    pub(super) a_collider: usize,
    pub(super) b_collider: usize,
    pub(super) normal: Vec3, // from A -> B
    pub(super) depth: Scalar,
}

#[derive(Copy, Clone, Debug)]
pub struct RayHit {
    pub body: BodyId,
    pub toi: f32, // fraction along max_distance in [0,1]
    pub point: Vec3,
    pub normal: Vec3,
}

#[derive(Copy, Clone, Debug)]
pub struct SweepHit {
    pub body: BodyId,
    pub toi: f32, // fraction along [from -> to] in [0,1]
    pub started_overlapping: bool,
    pub point: Vec3,
    pub normal: Vec3,
}

/* ---------------- Builder ---------------- */

pub struct WorldBuilder {
    pub bodies: usize,
    pub colliders: usize,
}

impl WorldBuilder {
    pub fn new() -> Self {
        Self { bodies: 128, colliders: 128 }
    }

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
pub(super) struct AccelComp {
    pub(super) aero: Option<AeroHandle>,
    pub(super) prop: Option<PropHandle>,
    pub(super) ref_area_m2: f32,
    pub(super) throttle01: f32,
    pub(super) forward_dir_world: Option<Vec3>, // default: +X body rotated to world
}

#[derive(Copy, Clone, Default)]
pub(super) struct WarmImp {
    pub(super) jn: f32,
    pub(super) jt1: f32,
    pub(super) jt2: f32,
}

/* ---------------- World ---------------- */

pub struct World {
    // legacy convenience gravity vector (kept in sync for Uniform spec)
    pub gravity: Vec3,

    pub epoch_id: u64,
    pub rng: XorShift64,

    pub(super) schedule: ScheduleRecorder,

    pub(super) bodies: Bodies,            // SoA
    pub(super) colliders: Vec<Collider>,

    // Phase 3/4: queued swaps
    pub(super) pending_epoch: Option<EpochDescriptor>,
    pub(super) pending_gravity: Option<GravitySpec>,

    // Active gravity procedure (Phase 4)
    pub(super) gravity_proc: GravitySpec,

    // Systems
    pub(super) joints: Joints,
    pub(super) tick: u64,
    pub(super) debug: DebugSettings,
    pub(super) ledger: Ledger,

    // Terrain (Phase 5)
    pub(super) terrain: Option<HeightField>,
    pub(super) tile_cfg: TerrainTileCfg,
    pub(super) tile_cache: BTreeMap<(i32, i32), TileEntry>,

    pub(super) guards: Vec<GuardInstance>,
    pub(super) models: ModelRegistry,
    pub(super) accel_comps: Vec<Option<AccelComp>>,
    pub(super) balances: Vec<BalanceInstance>,

    pub(super) warm_cache: BTreeMap<(u32, u32), WarmImp>,
    pub(super) last_normal_impulse: Vec<f32>,

    // Provenance counters (avoid depending on Ledger iteration API)
    pub(super) prov_impulses_sum: f32,
    pub(super) prov_ccd_hits: u32,
    pub(super) prov_aero_sum: f32,
    pub(super) prov_prop_sum: f32,

    pub(super) alive: Vec<u8>,
}

impl World {
    #[inline]
    pub(super) fn record(&mut self, e: LedgerEvent) {
        // Update provenance counters deterministically without needing Ledger::iter()
        use LedgerEvent::*;
        match e {
            CCDHit { .. } => self.prov_ccd_hits = self.prov_ccd_hits.wrapping_add(1),
            ImpulseN { jn, .. } => self.prov_impulses_sum += jn.abs(),
            ImpulseT { jt1, jt2, .. } => self.prov_impulses_sum += (jt1 * jt1 + jt2 * jt2).sqrt(),
            AeroProp { t_n, d_n, .. } => { self.prov_prop_sum += t_n; self.prov_aero_sum += d_n; }
            BalanceAccel { ax, az, .. } => self.prov_prop_sum += (ax * ax + az * az).sqrt(),
            _ => {}
        }
        self.ledger.push(e);
    }

    #[inline]
    pub(super) fn reset_provenance(&mut self) {
        self.prov_impulses_sum = 0.0;
        self.prov_ccd_hits = 0;
        self.prov_aero_sum = 0.0;
        self.prov_prop_sum = 0.0;
    }

    #[inline]
    pub(super) fn gather_box_aabbs_with_map(&self) -> (Vec<Aabb>, Vec<usize>) {
        // aabbs[k] corresponds to collider index map[k]
        let mut aabbs: Vec<Aabb> = Vec::new();
        let mut map: Vec<usize> = Vec::new();

        for (ci, c) in self.colliders.iter().enumerate() {
            if let Shape::Box { .. } = c.shape {
                aabbs.push(c.aabb);
                map.push(ci);
            }
        }
        (aabbs, map)
    }

    // Read-only helpers for the viewer/debuggers.
    pub fn num_bodies(&self) -> u32 { self.bodies.len() as u32 }

    pub fn primary_shape(&self, body: BodyId) -> Option<Shape> {
        for c in &self.colliders {
            if c.body == body {
                return Some(c.shape);
            }
        }
        None
    }

    #[inline]
    pub fn tick_index(&self) -> u64 { self.tick }

    pub fn for_each_collider<F: FnMut(u32, BodyId, &Shape, &Aabb)>(&self, mut f: F) {
        for (i, c) in self.colliders.iter().enumerate() {
            f(i as u32, c.body, &c.shape, &c.aabb);
        }
    }

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
            tile_cfg: TerrainTileCfg {
                tile_cells: UVec2::new(32, 32),
                y_offset: 0.0,
                origin: [0.0, 0.0],
            },
            tile_cache: BTreeMap::new(),
            guards: Vec::new(),
            models: ModelRegistry::new(),
            accel_comps: vec![None; bodies],
            balances: Vec::new(),
            warm_cache: BTreeMap::new(),
            last_normal_impulse: vec![0.0; bodies],
            prov_impulses_sum: 0.0,
            prov_ccd_hits: 0,
            prov_aero_sum: 0.0,
            prov_prop_sum: 0.0,
            alive: vec![0; bodies],
        }
    }

    pub fn add_balance_controller(&mut self, pelvis: BodyId, left: BodyId, right: BodyId, params: BalanceParams) {
        self.balances.push(BalanceInstance { pelvis, left, right, ctrl: BalanceCtrl::new(params) });
    }

    pub fn models_mut(&mut self) -> &mut ModelRegistry { &mut self.models }
    pub fn models(&self) -> &ModelRegistry { &self.models }

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
        self.accel_comps[i] = Some(AccelComp { aero, prop, ref_area_m2, throttle01, forward_dir_world });
    }

    pub fn set_body_pose(&mut self, id: BodyId, pose: Isometry) {
        self.bodies.set_pose(id.0, pose);
        // immediate AABB refresh for viewers (step recomputes too)
        for c in &mut self.colliders {
            if c.body == id {
                c.aabb = aabb_of(&c.shape, &pose);
            }
        }
    }

    pub fn yaw_body(&mut self, id: BodyId, dy: f32) {
        if dy == 0.0 { return; }
        let mut p = self.bodies.pose(id.0);
        let dq = Quat::from_rotation_y(dy);
        p.rot = (dq * p.rot).normalize();
        self.set_body_pose(id, p);
    }

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
        self.guards.push(GuardInstance { joint: j, eff, ctrl: GuardCtrl::new(params) });
        j
    }

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

    pub fn queue_epoch_swap(&mut self, desc: EpochDescriptor) { self.pending_epoch = Some(desc); }
    pub fn queue_gravity_swap(&mut self, spec: GravitySpec) { self.pending_gravity = Some(spec); }

    pub(super) fn apply_pending_epoch_if_any(&mut self) {
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

    pub fn add_body(&mut self, pose: Isometry, vel: Velocity, mass: MassProps, dynamic: bool) -> BodyId {
        let inv_mass = if dynamic { mass.inv_mass } else { 0.0 };
        let id = self.bodies.add(BodyDesc { pose, vel, inv_mass, dynamic });
        let idx = id as usize;
        if self.alive.len() <= idx { self.alive.resize(idx + 1, 0); }
        self.alive[idx] = 1;

        if self.accel_comps.len() <= id as usize {
            self.accel_comps.resize(id as usize + 1, None);
        }

        BodyId(id)

    }

    pub fn add_collider(
        &mut self,
        body: BodyId,
        shape: Shape,
        material: riftphys_materials::materials::Material,
    ) -> ColliderId {
        let pose = self.bodies.pose(body.0);
        let aabb = aabb_of(&shape, &pose);
        let id = self.colliders.len() as u32;
        self.colliders.push(Collider { body, shape, aabb, material });
        ColliderId(id)
    }

    pub fn set_heightfield(&mut self, hf: HeightField, y_offset: f32) {
        self.set_heightfield_at(hf, 0.0, 0.0, y_offset);
    }

    pub fn set_heightfield_at(&mut self, hf: HeightField, origin_x: f32, origin_z: f32, y_offset: f32) {
        self.terrain = Some(hf);
        self.tile_cfg.y_offset = y_offset;
        self.tile_cfg.origin = [origin_x, origin_z];
        self.tile_cache.clear();
    }

    pub fn clear_heightfield(&mut self) {
        self.terrain = None;
        self.tile_cache.clear();
    }

    pub fn heightfield(&self) -> Option<&HeightField> { self.terrain.as_ref() }

    pub fn sample_terrain_height_normal(&mut self, wx: f32, wz: f32) -> Option<(f32, Vec3)> {
        let hf = self.terrain.as_ref()?;

        // translate world-space to local HF coords
        let lx = wx - self.tile_cfg.origin[0];
        let lz = wz - self.tile_cfg.origin[1];

        // Tile key: each tile = tile_cells * cell_size in world units
        let tile_world_x = hf.cell.x * self.tile_cfg.tile_cells.x as f32;
        let tile_world_z = hf.cell.y * self.tile_cfg.tile_cells.y as f32;
        let tx = (lx / tile_world_x).floor() as i32;
        let tz = (lz / tile_world_z).floor() as i32;

        let key = (tx, tz);
        let entry = self.tile_cache.entry(key).or_default();
        entry.uses = entry.uses.wrapping_add(1);

        let h_local = hf.sample_height(lx, lz);
        let n_local = hf.sample_normal(lx, lz);
        let h_world = h_local + self.tile_cfg.y_offset;

        Some((h_world, n_local.into()))
    }

    #[inline]
    pub(super) fn eval_extra_accel(
        models: &ModelRegistry,
        bodies: &Bodies,
        accel_comps: &Vec<Option<AccelComp>>,
        epoch_id: u64,
        tick: u64,
        i: u32,
        pose: Isometry,
        vel: Velocity,
        gravity_proc: &GravitySpec,
        dt: f32,
    ) -> (Vec3, Vec3) {
        let a_grav = grav_eval(gravity_proc, pose.pos);
        let mut a_extra = Vec3::ZERO;

        if let Some(Some(ac)) = accel_comps.get(i as usize) {
            let inv_m = bodies.inv_mass_of(i);
            if inv_m > 0.0 {
                let mass = 1.0 / inv_m;

                let fwd_w = ac.forward_dir_world
                    .unwrap_or_else(|| pose.rot * Vec3::new(1.0, 0.0, 0.0));

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

    pub fn set_body_vel(&mut self, id: BodyId, vel: Velocity) { self.bodies.set_vel(id.0, vel); }
    pub fn apply_impulse(&mut self, id: BodyId, impulse: Vec3) { self.bodies.apply_impulse(id.0, impulse); }

    pub fn body_alive(&self, id: BodyId) -> bool {
        self.alive.get(id.0 as usize).copied().unwrap_or(0) != 0
    }

    pub fn remove_body(&mut self, id: BodyId) -> bool {
        let i = id.0 as usize;
        if i >= self.bodies.len() { return false; }
        if !self.body_alive(id) { return false; }

        self.alive[i] = 0;

        // stop sim
        let _ = self.bodies.deactivate(id.0);

        // remove colliders immediately so queries stop hitting it
        self.colliders.retain(|c| c.body != id);

        // clean aux state
        if i < self.accel_comps.len() { self.accel_comps[i] = None; }
        if i < self.last_normal_impulse.len() { self.last_normal_impulse[i] = 0.0; }

        self.guards.retain(|g| g.eff != id);
        self.balances.retain(|b| b.pelvis != id && b.left != id && b.right != id);

        self.warm_cache.clear();
        true
    }

    #[inline]
    pub fn normal_force(&self, body: BodyId, dt: f32) -> f32 {
        self.last_normal_impulse.get(body.0 as usize).copied().unwrap_or(0.0) / dt
    }

    pub fn add_ball_joint(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry) -> JointId {
        self.joints.add_ball(a, b, fa, fb)
    }

    pub fn add_hinge_joint(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, hinge_axis: usize) -> JointId {
        self.joints.add_hinge(a, b, fa, fb, hinge_axis)
    }

    pub fn add_d6_joint(&mut self, j: D6Joint) -> JointId { self.joints.add_d6(j) }
}
