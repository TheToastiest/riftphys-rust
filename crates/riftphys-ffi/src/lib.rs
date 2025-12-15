// SPDX-License-Identifier: MIT or your internal license
#![allow(clippy::missing_safety_doc)]

mod tests;

use std::cell::RefCell;
use std::ffi::{c_char, CString};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;
use std::sync::Mutex;

use riftphys_core::{iso, vec3, BodyId, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::*;
use riftphys_world::world;

/* ===================== ABI CONTRACT =====================
   - Body IDs exposed via FFI are 1-based (never 0).
   - 0 is reserved as "invalid / none".
   - Terrain uses a dedicated sentinel: RPHYS_TERRAIN_BODY_ID (0xFFFF_FFFF).
   - Any API accepting body_id treats:
        - 0 as invalid
        - RPHYS_TERRAIN_BODY_ID as invalid (terrain is not a real body)
        - out-of-range as invalid
        - removed (not alive) as invalid
   - ignore_body fields are also 1-based; 0 means "no ignore".
   ======================================================= */

pub type RPhysBool = u32;
pub type RPhysResult = u32;

// Keep this as u32 to match riftphys_c.h and avoid UB from invalid discriminants.
pub type RPhysBodyType = u32;
pub const RPHYS_BODY_STATIC: RPhysBodyType = 0;
pub const RPHYS_BODY_DYNAMIC: RPhysBodyType = 1;

// Treat zero-length (or effectively-zero) ray directions as "OK + miss".
const RPHYS_RAY_DIR_EPS_SQ: f32 = 1.0e-20;

// Stationary sweep epsilon (used only to force a query path when we need overlap semantics).
const RPHYS_STATIONARY_SWEEP_EPS: f32 = 1.0e-6;

pub const RPHYS_FALSE: RPhysBool = 0;
pub const RPHYS_TRUE: RPhysBool = 1;

pub const RPHYS_OK: RPhysResult = 0;
pub const RPHYS_ERR_NULL: RPhysResult = 1;
pub const RPHYS_ERR_INVALID_ID: RPhysResult = 2;
pub const RPHYS_ERR_CAPACITY: RPhysResult = 3;
pub const RPHYS_ERR_BAD_ARG: RPhysResult = 4;
pub const RPHYS_ERR_INTERNAL: RPhysResult = 0x8000_0000;

pub const RPHYS_ABI_VERSION: u32 = 1;

pub const RPHYS_INVALID_BODY_ID: u32 = 0;
pub const RPHYS_TERRAIN_BODY_ID: u32 = 0xFFFF_FFFF;

pub const RPHYS_STEP_HASH_BYTES: usize = 32;

thread_local! {
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

#[inline]
fn clear_last_error() {
    LAST_ERROR.with(|cell| *cell.borrow_mut() = None);
}

#[inline]
fn set_last_error(msg: &str) {
    let s = msg.to_string();
    LAST_ERROR.with(|cell| {
        *cell.borrow_mut() = CString::new(s).ok();
    });
}

#[no_mangle]
pub extern "C" fn rphys_last_error_message() -> *const c_char {
    LAST_ERROR.with(|cell| {
        if let Some(ref cstr) = *cell.borrow() {
            cstr.as_ptr()
        } else {
            ptr::null()
        }
    })
}

#[no_mangle]
pub extern "C" fn rphys_get_abi_version() -> u32 {
    RPHYS_ABI_VERSION
}

// Null-terminated, stable for a build (crate version).
static BUILD_ID: &str = concat!("riftphys_ffi-", env!("CARGO_PKG_VERSION"), "\0");

#[no_mangle]
pub extern "C" fn rphys_get_build_id() -> *const c_char {
    BUILD_ID.as_ptr() as *const c_char
}

#[inline]
fn ffi_guard<F: FnOnce() -> RPhysResult>(f: F) -> RPhysResult {
    clear_last_error();
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(r) => r,
        Err(_) => {
            set_last_error("panic crossed FFI boundary");
            RPHYS_ERR_INTERNAL
        }
    }
}

/* ===================== FFI TYPES ===================== */

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysHeightfieldDesc {
    pub size_bytes: u32,
    pub flags: u32,
    pub width: u32,
    pub height: u32,
    pub row_stride: u32, // 0 = width
    pub cell_size_x: f32,
    pub cell_size_z: f32,
    pub height_scale: f32,
    pub height_offset: f32,
    pub heights: *const i16,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysVec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysQuat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysIsometry {
    pub pos: RPhysVec3,
    pub rot: RPhysQuat,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysVelocity {
    pub lin: RPhysVec3,
    pub ang: RPhysVec3,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysMaterial {
    pub mu_static: f32,
    pub mu_dynamic: f32,
    pub restitution: f32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysCapsuleBodyDesc {
    pub pose: RPhysIsometry,
    pub vel: RPhysVelocity,
    pub radius: f32,
    pub half_height: f32,
    pub mass: f32,
    pub body_type: RPhysBodyType,
    pub material: RPhysMaterial,
    pub user_tag: u32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysBoxBodyDesc {
    pub pose: RPhysIsometry,
    pub vel: RPhysVelocity,
    pub hx: f32,
    pub hy: f32,
    pub hz: f32,
    pub mass: f32,
    pub body_type: RPhysBodyType,
    pub material: RPhysMaterial,
    pub user_tag: u32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysSphereBodyDesc {
    pub pose: RPhysIsometry,
    pub vel: RPhysVelocity,
    pub radius: f32,
    pub mass: f32,
    pub body_type: RPhysBodyType,
    pub material: RPhysMaterial,
    pub user_tag: u32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysRayQuery {
    pub origin: RPhysVec3,
    pub dir: RPhysVec3,
    pub max_distance: f32,
    pub ignore_body: u32, // 1-based; 0 = none
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysRayHit {
    pub hit: RPhysBool,
    pub body_id: u32, // 1-based; 0 = none; terrain = RPHYS_TERRAIN_BODY_ID
    pub fraction: f32,
    pub point: RPhysVec3,
    pub normal: RPhysVec3,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysCapsuleSweepQuery {
    pub from: RPhysVec3,
    pub to: RPhysVec3,
    pub radius: f32,
    pub half_height: f32,
    pub ignore_body: u32, // 1-based; 0 = none
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysCapsuleSweepHit {
    pub hit: RPhysBool,
    pub body_id: u32, // 1-based; 0 = none; terrain = RPHYS_TERRAIN_BODY_ID
    pub fraction: f32,
    pub started_overlapping: RPhysBool,
    pub point: RPhysVec3,
    pub normal: RPhysVec3,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysHash32 {
    pub bytes: [u8; RPHYS_STEP_HASH_BYTES],
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysStepParams {
    pub size_bytes: u32,
    pub flags: u32,
    pub substeps: u32,
    pub solver_iters: u32,
    pub dt: f32,
}

/* ===================== HANDLE ===================== */

struct WorldState {
    world: world::World,
    bodies_created: u32,    // monotonic max assigned body index + 1 (no-reuse contract)
    colliders_created: u32, // monotonic (no-reuse contract)
}

pub struct RPhysWorld {
    state: Mutex<WorldState>,
    max_bodies: u32,    // lifetime-created cap
    max_colliders: u32, // lifetime-created cap
}

impl RPhysWorld {
    #[inline]
    unsafe fn from_raw_ref<'a>(ptr: *const RPhysWorld) -> &'a RPhysWorld {
        &*ptr
    }
    #[inline]
    unsafe fn from_raw_mut<'a>(ptr: *mut RPhysWorld) -> &'a RPhysWorld {
        // NOTE: we only ever take &RPhysWorld and mutate through the mutex.
        &*ptr
    }
}

/* ===================== VALIDATION HELPERS ===================== */

#[inline]
fn validate_body_type(ctx: &'static str, t: RPhysBodyType) -> Result<(), RPhysResult> {
    if t == RPHYS_BODY_STATIC || t == RPHYS_BODY_DYNAMIC {
        Ok(())
    } else {
        set_last_error(&format!("{ctx}: invalid body_type={t}"));
        Err(RPHYS_ERR_BAD_ARG)
    }
}

#[inline]
fn validate_dynamic_mass(ctx: &'static str, t: RPhysBodyType, mass: f32) -> Result<(), RPhysResult> {
    validate_body_type(ctx, t)?;
    if t == RPHYS_BODY_DYNAMIC && !(mass.is_finite() && mass > 0.0) {
        set_last_error(&format!("{ctx}: dynamic body mass must be finite and > 0"));
        return Err(RPHYS_ERR_BAD_ARG);
    }
    Ok(())
}

#[inline]
fn enforce_v1_monotonic_body_alloc(
    st: &mut WorldState,
    body: BodyId,
    ctx: &'static str,
) -> Result<(), RPhysResult> {
    if body.0 == u32::MAX {
        set_last_error(&format!("{ctx}: internal returned terrain sentinel as a body id"));
        return Err(RPHYS_ERR_INTERNAL);
    }

    // ABI v1: no reuse, no gaps. next id must be exactly bodies_created.
    if body.0 != st.bodies_created {
        let _ = st.world.remove_body(body);
        set_last_error(&format!(
            "{ctx}: internal body id reuse/skip detected (got={}, expected={}); ABI v1 forbids reuse",
            body.0, st.bodies_created
        ));
        return Err(RPHYS_ERR_INTERNAL);
    }

    st.bodies_created = st.bodies_created.saturating_add(1);
    Ok(())
}

#[inline]
fn is_finite_vec3(v: &RPhysVec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}
#[inline]
fn is_finite_quat(q: &RPhysQuat) -> bool {
    q.x.is_finite() && q.y.is_finite() && q.z.is_finite() && q.w.is_finite()
}
#[inline]
fn is_finite_iso(p: &RPhysIsometry) -> bool {
    is_finite_vec3(&p.pos) && is_finite_quat(&p.rot)
}
#[inline]
fn is_finite_vel(v: &RPhysVelocity) -> bool {
    is_finite_vec3(&v.lin) && is_finite_vec3(&v.ang)
}
#[inline]
fn is_finite_material(m: &RPhysMaterial) -> bool {
    m.mu_static.is_finite() && m.mu_dynamic.is_finite() && m.restitution.is_finite()
}

#[inline]
fn bad_arg(ctx: &'static str, msg: &str) -> RPhysResult {
    set_last_error(&format!("{ctx}: {msg}"));
    RPHYS_ERR_BAD_ARG
}

/* ===================== ID CONVERSIONS ===================== */

#[inline]
fn body_to_ffi_checked(id: BodyId, ctx: &'static str) -> Result<u32, RPhysResult> {
    if id.0 == u32::MAX {
        return Ok(RPHYS_TERRAIN_BODY_ID);
    }
    match id.0.checked_add(1) {
        Some(v) if v != RPHYS_TERRAIN_BODY_ID => Ok(v),
        _ => {
            set_last_error(&format!("{ctx}: internal id overflow / sentinel collision"));
            Err(RPHYS_ERR_INTERNAL)
        }
    }
}

#[inline]
fn checked_live_body_id(st: &WorldState, id: u32, ctx: &'static str) -> Result<BodyId, RPhysResult> {
    if id == RPHYS_INVALID_BODY_ID {
        set_last_error(&format!("{ctx}: body_id == 0 (invalid)"));
        return Err(RPHYS_ERR_INVALID_ID);
    }
    if id == RPHYS_TERRAIN_BODY_ID {
        set_last_error(&format!("{ctx}: body_id is terrain sentinel (not a real body)"));
        return Err(RPHYS_ERR_INVALID_ID);
    }

    let raw = id - 1;

    // Range is against monotonic created max (not alive count).
    if raw >= st.bodies_created {
        set_last_error(&format!(
            "{ctx}: body_id out of range (id={id}, created_max={})",
            st.bodies_created
        ));
        return Err(RPHYS_ERR_INVALID_ID);
    }

    let bid = BodyId(raw);
    if !st.world.body_alive(bid) {
        set_last_error(&format!("{ctx}: body_id not found (removed) (id={id})"));
        return Err(RPHYS_ERR_INVALID_ID);
    }

    Ok(bid)
}

#[inline]
fn checked_optional_live_body_id(
    st: &WorldState,
    id: u32,
    ctx: &'static str,
) -> Result<Option<BodyId>, RPhysResult> {
    if id == 0 {
        return Ok(None);
    }
    Ok(Some(checked_live_body_id(st, id, ctx)?))
}

#[inline]
fn ensure_capacity(
    st: &WorldState,
    max_bodies: u32,
    max_colliders: u32,
    add_bodies: u32,
    add_colliders: u32,
    ctx: &'static str,
) -> Result<(), RPhysResult> {
    if st.bodies_created.saturating_add(add_bodies) > max_bodies {
        set_last_error(&format!(
            "{ctx}: body capacity exceeded (created={}, add={}, max={})",
            st.bodies_created, add_bodies, max_bodies
        ));
        return Err(RPHYS_ERR_CAPACITY);
    }
    if st.colliders_created.saturating_add(add_colliders) > max_colliders {
        set_last_error(&format!(
            "{ctx}: collider capacity exceeded (created={}, add={}, max={})",
            st.colliders_created, add_colliders, max_colliders
        ));
        return Err(RPHYS_ERR_CAPACITY);
    }
    Ok(())
}

/* ===================== DETERMINISTIC DEFAULT WRITES ===================== */

#[inline]
unsafe fn write_identity_pose(out_pose: *mut RPhysIsometry) {
    if out_pose.is_null() {
        return;
    }
    (*out_pose).pos = RPhysVec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    (*out_pose).rot = RPhysQuat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
}

#[inline]
unsafe fn write_zero_vel(out_vel: *mut RPhysVelocity) {
    if out_vel.is_null() {
        return;
    }
    (*out_vel).lin = RPhysVec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    (*out_vel).ang = RPhysVec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
}

#[inline]
unsafe fn write_raycast_miss(out_hit: *mut RPhysRayHit) {
    if out_hit.is_null() {
        return;
    }
    (*out_hit).hit = RPHYS_FALSE;
    (*out_hit).body_id = RPHYS_INVALID_BODY_ID;
    (*out_hit).fraction = 1.0;
    (*out_hit).point = RPhysVec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    (*out_hit).normal = RPhysVec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
}

#[inline]
unsafe fn write_sweep_miss(out_hit: *mut RPhysCapsuleSweepHit) {
    if out_hit.is_null() {
        return;
    }
    (*out_hit).hit = RPHYS_FALSE;
    (*out_hit).body_id = RPHYS_INVALID_BODY_ID;
    (*out_hit).fraction = 1.0;
    (*out_hit).started_overlapping = RPHYS_FALSE;
    (*out_hit).point = RPhysVec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    (*out_hit).normal = RPhysVec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
}

/* ===================== CONVERSIONS ===================== */

#[inline]
fn v3_from_ffi(v: RPhysVec3) -> riftphys_core::Vec3 {
    vec3(v.x, v.y, v.z)
}
#[inline]
fn v3_to_ffi(v: riftphys_core::Vec3) -> RPhysVec3 {
    let a = v.to_array();
    RPhysVec3 {
        x: a[0],
        y: a[1],
        z: a[2],
    }
}

#[inline]
fn q_from_ffi(q: RPhysQuat) -> riftphys_core::Quat {
    if !(q.x.is_finite() && q.y.is_finite() && q.z.is_finite() && q.w.is_finite()) {
        return riftphys_core::Quat::IDENTITY;
    }
    let qq = riftphys_core::Quat::from_xyzw(q.x, q.y, q.z, q.w);
    let lsq = qq.length_squared();
    if lsq.is_finite() && lsq > 0.0 {
        qq / lsq.sqrt()
    } else {
        riftphys_core::Quat::IDENTITY
    }
}

#[inline]
fn q_to_ffi(q: riftphys_core::Quat) -> RPhysQuat {
    RPhysQuat {
        x: q.x,
        y: q.y,
        z: q.z,
        w: q.w,
    }
}

#[inline]
fn iso_from_ffi(p: &RPhysIsometry) -> riftphys_core::Isometry {
    iso(v3_from_ffi(p.pos), q_from_ffi(p.rot))
}
#[inline]
fn iso_to_ffi(p: &riftphys_core::Isometry) -> RPhysIsometry {
    RPhysIsometry {
        pos: v3_to_ffi(p.pos),
        rot: q_to_ffi(p.rot),
    }
}

#[inline]
fn vel_from_ffi(v: &RPhysVelocity) -> Velocity {
    Velocity {
        lin: v3_from_ffi(v.lin),
        ang: v3_from_ffi(v.ang),
    }
}
#[inline]
fn vel_to_ffi(v: &Velocity) -> RPhysVelocity {
    RPhysVelocity {
        lin: v3_to_ffi(v.lin),
        ang: v3_to_ffi(v.ang),
    }
}

#[inline]
fn mat_from_ffi(m: &RPhysMaterial) -> Material {
    let mut out = material(MaterialId::Default);

    let mu_s = quantize(m.mu_static.max(0.0));
    let mu_k = quantize(m.mu_dynamic.max(0.0));
    let rest = quantize(m.restitution.clamp(0.0, 1.0));

    out.contact.mu_s = mu_s;
    out.contact.mu_k = mu_k;
    out.contact.restitution = rest;

    out.mu_s = mu_s;
    out.mu_k = mu_k;
    out.restitution = rest;

    out
}

/* ===================== LAST ERROR COPY ===================== */

#[no_mangle]
pub unsafe extern "C" fn rphys_last_error_copy(
    dst: *mut c_char,
    cap: u32,
    out_len: *mut u32,
) -> RPhysResult {
    // Determine current message length (bytes, without NUL) without allocating.
    let mut len: u32 = 0;
    LAST_ERROR.with(|cell| {
        if let Some(ref s) = *cell.borrow() {
            len = s.as_bytes().len() as u32;
        }
    });

    if !out_len.is_null() {
        *out_len = len;
    }

    // Allow “query length only”.
    if cap == 0 {
        return RPHYS_OK;
    }
    if dst.is_null() {
        return RPHYS_ERR_NULL;
    }

    // No message => empty string.
    if len == 0 {
        *(dst as *mut u8) = 0;
        return RPHYS_OK;
    }

    let max_copy = (cap - 1) as usize;

    LAST_ERROR.with(|cell| {
        if let Some(ref s) = *cell.borrow() {
            let b = s.as_bytes();
            let n = core::cmp::min(b.len(), max_copy);
            core::ptr::copy_nonoverlapping(b.as_ptr(), dst as *mut u8, n);
            *((dst as *mut u8).add(n)) = 0;
        } else {
            *(dst as *mut u8) = 0;
        }
    });

    RPHYS_OK
}

/* ===================== WORLD LIFECYCLE ===================== */

#[no_mangle]
pub unsafe extern "C" fn rphys_world_create(max_bodies: u32, max_colliders: u32) -> *mut RPhysWorld {
    clear_last_error();
    match catch_unwind(AssertUnwindSafe(|| {
        let w = world::WorldBuilder::new()
            .with_capacity(max_bodies as usize, max_colliders as usize)
            .build();

        Box::into_raw(Box::new(RPhysWorld {
            state: Mutex::new(WorldState {
                world: w,
                bodies_created: 0,
                colliders_created: 0,
            }),
            max_bodies,
            max_colliders,
        }))
    })) {
        Ok(ptr) => ptr,
        Err(_) => {
            set_last_error("panic in rphys_world_create");
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_destroy(world: *mut RPhysWorld) {
    if !world.is_null() {
        let _ = Box::from_raw(world);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_set_epoch(world: *mut RPhysWorld, epoch: u64) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() {
            set_last_error("rphys_world_set_epoch: world is null");
            return RPHYS_ERR_NULL;
        }
        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };
        st.world.set_epoch(epoch);
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_set_rng_seed(world: *mut RPhysWorld, seed: u64) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() {
            set_last_error("rphys_world_set_rng_seed: world is null");
            return RPHYS_ERR_NULL;
        }
        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };
        st.world.set_rng_seed(seed);
        RPHYS_OK
    })
}

/* ===================== BODY CREATION ===================== */

#[no_mangle]
pub unsafe extern "C" fn rphys_add_capsule_body(
    world: *mut RPhysWorld,
    desc: *const RPhysCapsuleBodyDesc,
    out_body_id: *mut u32,
) -> RPhysResult {
    ffi_guard(|| {
        if !out_body_id.is_null() {
            unsafe { *out_body_id = RPHYS_INVALID_BODY_ID };
        }
        if world.is_null() || desc.is_null() || out_body_id.is_null() {
            set_last_error("rphys_add_capsule_body: null arg");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let d = unsafe { &*desc };

        if !is_finite_iso(&d.pose) {
            return bad_arg("rphys_add_capsule_body", "pose contains NaN/Inf");
        }
        if !is_finite_vel(&d.vel) {
            return bad_arg("rphys_add_capsule_body", "vel contains NaN/Inf");
        }
        if !is_finite_material(&d.material) {
            return bad_arg("rphys_add_capsule_body", "material contains NaN/Inf");
        }
        if !(d.radius.is_finite() && d.radius > 0.0) {
            return bad_arg("rphys_add_capsule_body", "radius must be finite and > 0");
        }
        if !(d.half_height.is_finite() && d.half_height >= 0.0) {
            return bad_arg("rphys_add_capsule_body", "half_height must be finite and >= 0");
        }
        if let Err(e) = validate_dynamic_mass("rphys_add_capsule_body", d.body_type, d.mass) {
            return e;
        }

        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        if let Err(e) = ensure_capacity(&st, w.max_bodies, w.max_colliders, 1, 1, "rphys_add_capsule_body") {
            return e;
        }

        let dyn_flag = d.body_type == RPHYS_BODY_DYNAMIC;
        let mass_props = if dyn_flag {
            MassProps::from_capsule(d.radius, d.half_height, MaterialId::Default)
        } else {
            MassProps::infinite()
        };

        let body = st
            .world
            .add_body(iso_from_ffi(&d.pose), vel_from_ffi(&d.vel), mass_props, dyn_flag);
        if let Err(e) = enforce_v1_monotonic_body_alloc(&mut st, body, "rphys_add_capsule_body") {
            return e;
        }

        let mat = mat_from_ffi(&d.material);
        st.world
            .add_collider(body, Shape::Capsule { r: d.radius, hh: d.half_height }, mat);

        st.colliders_created = st.colliders_created.saturating_add(1);

        match body_to_ffi_checked(body, "rphys_add_capsule_body") {
            Ok(fid) => {
                unsafe { *out_body_id = fid };
                RPHYS_OK
            }
            Err(e) => e,
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_add_box_body(
    world: *mut RPhysWorld,
    desc: *const RPhysBoxBodyDesc,
    out_body_id: *mut u32,
) -> RPhysResult {
    ffi_guard(|| {
        if !out_body_id.is_null() {
            unsafe { *out_body_id = RPHYS_INVALID_BODY_ID };
        }
        if world.is_null() || desc.is_null() || out_body_id.is_null() {
            set_last_error("rphys_add_box_body: null arg");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let d = unsafe { &*desc };

        if !is_finite_iso(&d.pose) {
            return bad_arg("rphys_add_box_body", "pose contains NaN/Inf");
        }
        if !is_finite_vel(&d.vel) {
            return bad_arg("rphys_add_box_body", "vel contains NaN/Inf");
        }
        if !is_finite_material(&d.material) {
            return bad_arg("rphys_add_box_body", "material contains NaN/Inf");
        }
        if !(d.hx.is_finite() && d.hx > 0.0) {
            return bad_arg("rphys_add_box_body", "hx must be finite and > 0");
        }
        if !(d.hy.is_finite() && d.hy > 0.0) {
            return bad_arg("rphys_add_box_body", "hy must be finite and > 0");
        }
        if !(d.hz.is_finite() && d.hz > 0.0) {
            return bad_arg("rphys_add_box_body", "hz must be finite and > 0");
        }
        if let Err(e) = validate_dynamic_mass("rphys_add_box_body", d.body_type, d.mass) {
            return e;
        }

        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        if let Err(e) = ensure_capacity(&st, w.max_bodies, w.max_colliders, 1, 1, "rphys_add_box_body") {
            return e;
        }

        let dyn_flag = d.body_type == RPHYS_BODY_DYNAMIC;
        let mass_props = if dyn_flag {
            MassProps::from_box(vec3(d.hx, d.hy, d.hz), MaterialId::Default)
        } else {
            MassProps::infinite()
        };

        let body = st
            .world
            .add_body(iso_from_ffi(&d.pose), vel_from_ffi(&d.vel), mass_props, dyn_flag);
        if let Err(e) = enforce_v1_monotonic_body_alloc(&mut st, body, "rphys_add_box_body") {
            return e;
        }

        let mat = mat_from_ffi(&d.material);
        st.world
            .add_collider(body, Shape::Box { hx: d.hx, hy: d.hy, hz: d.hz }, mat);

        st.colliders_created = st.colliders_created.saturating_add(1);

        match body_to_ffi_checked(body, "rphys_add_box_body") {
            Ok(fid) => {
                unsafe { *out_body_id = fid };
                RPHYS_OK
            }
            Err(e) => e,
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_add_sphere_body(
    world: *mut RPhysWorld,
    desc: *const RPhysSphereBodyDesc,
    out_body_id: *mut u32,
) -> RPhysResult {
    ffi_guard(|| {
        if !out_body_id.is_null() {
            unsafe { *out_body_id = RPHYS_INVALID_BODY_ID };
        }
        if world.is_null() || desc.is_null() || out_body_id.is_null() {
            set_last_error("rphys_add_sphere_body: null arg");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let d = unsafe { &*desc };

        if !is_finite_iso(&d.pose) {
            return bad_arg("rphys_add_sphere_body", "pose contains NaN/Inf");
        }
        if !is_finite_vel(&d.vel) {
            return bad_arg("rphys_add_sphere_body", "vel contains NaN/Inf");
        }
        if !is_finite_material(&d.material) {
            return bad_arg("rphys_add_sphere_body", "material contains NaN/Inf");
        }
        if !(d.radius.is_finite() && d.radius > 0.0) {
            return bad_arg("rphys_add_sphere_body", "radius must be finite and > 0");
        }
        if let Err(e) = validate_dynamic_mass("rphys_add_sphere_body", d.body_type, d.mass) {
            return e;
        }

        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        if let Err(e) = ensure_capacity(&st, w.max_bodies, w.max_colliders, 1, 1, "rphys_add_sphere_body") {
            return e;
        }

        let dyn_flag = d.body_type == RPHYS_BODY_DYNAMIC;
        let mass_props = if dyn_flag {
            MassProps::from_sphere(d.radius, MaterialId::Default)
        } else {
            MassProps::infinite()
        };

        let body = st
            .world
            .add_body(iso_from_ffi(&d.pose), vel_from_ffi(&d.vel), mass_props, dyn_flag);
        if let Err(e) = enforce_v1_monotonic_body_alloc(&mut st, body, "rphys_add_sphere_body") {
            return e;
        }

        let mat = mat_from_ffi(&d.material);
        st.world.add_collider(body, Shape::Sphere { r: d.radius }, mat);

        st.colliders_created = st.colliders_created.saturating_add(1);

        match body_to_ffi_checked(body, "rphys_add_sphere_body") {
            Ok(fid) => {
                unsafe { *out_body_id = fid };
                RPHYS_OK
            }
            Err(e) => e,
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_remove(world: *mut RPhysWorld, body_id: u32) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() {
            set_last_error("rphys_body_remove: world is null");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let bid = match checked_live_body_id(&st, body_id, "rphys_body_remove") {
            Ok(v) => v,
            Err(e) => return e,
        };

        if st.world.remove_body(bid) {
            RPHYS_OK
        } else {
            set_last_error("rphys_body_remove: internal remove failed");
            RPHYS_ERR_INTERNAL
        }
    })
}

/* ===================== POSE / VELOCITY ===================== */

#[no_mangle]
pub unsafe extern "C" fn rphys_body_get_pose(
    world: *const RPhysWorld,
    body_id: u32,
    out_pose: *mut RPhysIsometry,
) -> RPhysResult {
    ffi_guard(|| {
        if out_pose.is_null() {
            set_last_error("rphys_body_get_pose: out_pose is null");
            return RPHYS_ERR_NULL;
        }
        unsafe { write_identity_pose(out_pose) };

        if world.is_null() {
            set_last_error("rphys_body_get_pose: world is null");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let bid = match checked_live_body_id(&st, body_id, "rphys_body_get_pose") {
            Ok(v) => v,
            Err(e) => return e,
        };

        let pose = st.world.get_body_pose(bid);
        unsafe { ptr::write(out_pose, iso_to_ffi(&pose)) };
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_set_pose(
    world: *mut RPhysWorld,
    body_id: u32,
    pose: *const RPhysIsometry,
) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() || pose.is_null() {
            set_last_error("rphys_body_set_pose: null arg");
            return RPHYS_ERR_NULL;
        }

        let p = unsafe { &*pose };
        if !is_finite_iso(p) {
            return bad_arg("rphys_body_set_pose", "pose contains NaN/Inf");
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let bid = match checked_live_body_id(&st, body_id, "rphys_body_set_pose") {
            Ok(v) => v,
            Err(e) => return e,
        };

        st.world.set_body_pose(bid, iso_from_ffi(p));
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_get_velocity(
    world: *const RPhysWorld,
    body_id: u32,
    out_vel: *mut RPhysVelocity,
) -> RPhysResult {
    ffi_guard(|| {
        if out_vel.is_null() {
            set_last_error("rphys_body_get_velocity: out_vel is null");
            return RPHYS_ERR_NULL;
        }
        unsafe { write_zero_vel(out_vel) };

        if world.is_null() {
            set_last_error("rphys_body_get_velocity: world is null");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let bid = match checked_live_body_id(&st, body_id, "rphys_body_get_velocity") {
            Ok(v) => v,
            Err(e) => return e,
        };

        let vel = st.world.get_body_vel(bid);
        unsafe { ptr::write(out_vel, vel_to_ffi(&vel)) };
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_set_velocity(
    world: *mut RPhysWorld,
    body_id: u32,
    vel: *const RPhysVelocity,
) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() || vel.is_null() {
            set_last_error("rphys_body_set_velocity: null arg");
            return RPHYS_ERR_NULL;
        }

        let v = unsafe { &*vel };
        if !is_finite_vel(v) {
            return bad_arg("rphys_body_set_velocity", "velocity contains NaN/Inf");
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let bid = match checked_live_body_id(&st, body_id, "rphys_body_set_velocity") {
            Ok(v) => v,
            Err(e) => return e,
        };

        st.world.set_body_vel(bid, vel_from_ffi(v));
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_apply_impulse(
    world: *mut RPhysWorld,
    body_id: u32,
    impulse: RPhysVec3,
) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() {
            set_last_error("rphys_body_apply_impulse: world is null");
            return RPHYS_ERR_NULL;
        }
        if !is_finite_vec3(&impulse) {
            return bad_arg("rphys_body_apply_impulse", "impulse contains NaN/Inf");
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let bid = match checked_live_body_id(&st, body_id, "rphys_body_apply_impulse") {
            Ok(v) => v,
            Err(e) => return e,
        };

        st.world.apply_impulse(bid, v3_from_ffi(impulse));
        RPHYS_OK
    })
}

/* ===================== STEPPING ===================== */

#[no_mangle]
pub unsafe extern "C" fn rphys_world_step_ex(world: *mut RPhysWorld, p: *const RPhysStepParams) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() || p.is_null() {
            set_last_error("rphys_world_step_ex: null arg");
            return RPHYS_ERR_NULL;
        }

        let params = unsafe { &*p };
        let need = core::mem::size_of::<RPhysStepParams>() as u32;
        if params.size_bytes < need {
            set_last_error("rphys_world_step_ex: size_bytes too small");
            return RPHYS_ERR_BAD_ARG;
        }

        if !params.dt.is_finite() {
            set_last_error("rphys_world_step_ex: dt is not finite");
            return RPHYS_ERR_BAD_ARG;
        }
        if params.dt < 0.0 {
            set_last_error("rphys_world_step_ex: dt is negative");
            return RPHYS_ERR_BAD_ARG;
        }
        if params.substeps == 0 {
            set_last_error("rphys_world_step_ex: substeps == 0");
            return RPHYS_ERR_BAD_ARG;
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let sub_dt = params.dt / (params.substeps as f32);
        if !sub_dt.is_finite() {
            set_last_error("rphys_world_step_ex: sub_dt is not finite");
            return RPHYS_ERR_BAD_ARG;
        }

        for _ in 0..params.substeps {
            st.world.step(sub_dt);
        }

        let _ = params.solver_iters;
        let _ = params.flags;

        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_step(world: *mut RPhysWorld, dt: f32) -> RPhysResult {
    let params = RPhysStepParams {
        size_bytes: core::mem::size_of::<RPhysStepParams>() as u32,
        flags: 0,
        substeps: 1,
        solver_iters: 0,
        dt,
    };
    rphys_world_step_ex(world, &params)
}

/* ===================== DETERMINISM / DIAGNOSTICS ===================== */

#[no_mangle]
pub unsafe extern "C" fn rphys_world_tick_index(world: *const RPhysWorld, out_tick: *mut u64) -> RPhysResult {
    ffi_guard(|| {
        if out_tick.is_null() {
            set_last_error("rphys_world_tick_index: out_tick is null");
            return RPHYS_ERR_NULL;
        }
        unsafe { *out_tick = 0 };

        if world.is_null() {
            set_last_error("rphys_world_tick_index: world is null");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        unsafe { *out_tick = st.world.tick_index() as u64 };
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_num_bodies(world: *const RPhysWorld, out_num: *mut u32) -> RPhysResult {
    ffi_guard(|| {
        if out_num.is_null() {
            set_last_error("rphys_world_num_bodies: out_num is null");
            return RPHYS_ERR_NULL;
        }
        unsafe { *out_num = 0 };

        if world.is_null() {
            set_last_error("rphys_world_num_bodies: world is null");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        unsafe { *out_num = st.world.num_bodies() as u32 };
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_get_step_hash(
    world: *const RPhysWorld,
    out_hash: *mut RPhysHash32,
) -> RPhysResult {
    ffi_guard(|| {
        if out_hash.is_null() {
            set_last_error("rphys_world_get_step_hash: out_hash is null");
            return RPHYS_ERR_NULL;
        }
        unsafe { ptr::write(out_hash, RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] }) };

        if world.is_null() {
            set_last_error("rphys_world_get_step_hash: world is null");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let h = st.world.step_hash();
        unsafe { ptr::write(out_hash, RPhysHash32 { bytes: h }) };
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_step_and_hash(
    world: *mut RPhysWorld,
    dt: f32,
    out_hash: *mut RPhysHash32,
) -> RPhysResult {
    ffi_guard(|| {
        if out_hash.is_null() {
            set_last_error("rphys_world_step_and_hash: out_hash is null");
            return RPHYS_ERR_NULL;
        }
        unsafe { ptr::write(out_hash, RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] }) };

        let r = rphys_world_step(world, dt);
        if r != RPHYS_OK {
            return r;
        }
        rphys_world_get_step_hash(world as *const RPhysWorld, out_hash)
    })
}

/* ===================== QUERIES ===================== */

#[no_mangle]
pub unsafe extern "C" fn rphys_world_raycast(
    world: *const RPhysWorld,
    query: *const RPhysRayQuery,
    out_hit: *mut RPhysRayHit,
) -> RPhysResult {
    ffi_guard(|| {
        unsafe { write_raycast_miss(out_hit) };

        if world.is_null() || query.is_null() || out_hit.is_null() {
            set_last_error("rphys_world_raycast: null arg");
            return RPHYS_ERR_NULL;
        }

        let q = unsafe { &*query };

        if !is_finite_vec3(&q.origin) {
            return bad_arg("rphys_world_raycast", "origin contains NaN/Inf");
        }
        if !is_finite_vec3(&q.dir) {
            return bad_arg("rphys_world_raycast", "dir contains NaN/Inf");
        }
        if !q.max_distance.is_finite() {
            return bad_arg("rphys_world_raycast", "max_distance is not finite");
        }

        let origin = v3_from_ffi(q.origin);
        let dir_raw = v3_from_ffi(q.dir);

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let ignore = match checked_optional_live_body_id(&st, q.ignore_body, "rphys_world_raycast.ignore_body") {
            Ok(v) => v,
            Err(e) => return e,
        };

        if q.max_distance <= 0.0 {
            return RPHYS_OK;
        }

        let len2 = dir_raw.length_squared();
        if !len2.is_finite() {
            return bad_arg("rphys_world_raycast", "dir length overflow/invalid");
        }
        if len2 <= RPHYS_RAY_DIR_EPS_SQ {
            return RPHYS_OK;
        }

        let dir = dir_raw / len2.sqrt();
        let max_d = q.max_distance;

        if let Some(hit) = st.world.raycast(origin, dir, max_d, ignore) {
            let body_id = if hit.body.0 == u32::MAX {
                RPHYS_TERRAIN_BODY_ID
            } else {
                match body_to_ffi_checked(hit.body, "rphys_world_raycast") {
                    Ok(v) => v,
                    Err(e) => return e,
                }
            };

            unsafe {
                (*out_hit).hit = RPHYS_TRUE;
                (*out_hit).body_id = body_id;
                (*out_hit).fraction = hit.toi.clamp(0.0, 1.0);
                (*out_hit).point = v3_to_ffi(hit.point);
                (*out_hit).normal = v3_to_ffi(hit.normal);
            }
        }

        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_capsule_sweep(
    world: *const RPhysWorld,
    query: *const RPhysCapsuleSweepQuery,
    out_hit: *mut RPhysCapsuleSweepHit,
) -> RPhysResult {
    ffi_guard(|| {
        unsafe { write_sweep_miss(out_hit) };

        if world.is_null() || query.is_null() || out_hit.is_null() {
            set_last_error("rphys_world_capsule_sweep: null arg");
            return RPHYS_ERR_NULL;
        }

        let q = unsafe { &*query };
        if !is_finite_vec3(&q.from) {
            return bad_arg("rphys_world_capsule_sweep", "from contains NaN/Inf");
        }
        if !is_finite_vec3(&q.to) {
            return bad_arg("rphys_world_capsule_sweep", "to contains NaN/Inf");
        }
        if !(q.radius.is_finite() && q.radius > 0.0) {
            return bad_arg("rphys_world_capsule_sweep", "radius must be finite and > 0");
        }
        if !(q.half_height.is_finite() && q.half_height >= 0.0) {
            return bad_arg("rphys_world_capsule_sweep", "half_height must be finite and >= 0");
        }

        let from = v3_from_ffi(q.from);
        let to = v3_from_ffi(q.to);

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        let ignore = match checked_optional_live_body_id(&st, q.ignore_body, "rphys_world_capsule_sweep.ignore_body") {
            Ok(v) => v,
            Err(e) => return e,
        };

        // Stationary: overlap semantics only (must *start* overlapping to count as a hit).
        let delta = to - from;
        let stationary = delta.length_squared() == 0.0;

        let sweep_to = if stationary {
            from + vec3(RPHYS_STATIONARY_SWEEP_EPS, 0.0, 0.0)
        } else {
            to
        };

        if let Some(hit) = st.world.sweep_capsule(from, sweep_to, q.radius, q.half_height, ignore) {
            if stationary && !hit.started_overlapping {
                return RPHYS_OK; // overlap query semantics: not overlapping => miss
            }

            let body_id = if hit.body.0 == u32::MAX {
                RPHYS_TERRAIN_BODY_ID
            } else {
                match body_to_ffi_checked(hit.body, "rphys_world_capsule_sweep") {
                    Ok(v) => v,
                    Err(e) => return e,
                }
            };

            unsafe {
                (*out_hit).hit = RPHYS_TRUE;
                (*out_hit).body_id = body_id;
                (*out_hit).fraction = if stationary { 0.0 } else { hit.toi.clamp(0.0, 1.0) };
                (*out_hit).started_overlapping = if hit.started_overlapping { RPHYS_TRUE } else { RPHYS_FALSE };
                (*out_hit).point = v3_to_ffi(hit.point);
                (*out_hit).normal = v3_to_ffi(hit.normal);
            }
        }

        RPHYS_OK
    })
}

/* ===================== TERRAIN ===================== */

#[no_mangle]
pub unsafe extern "C" fn rphys_world_set_heightfield_i16(
    world: *mut RPhysWorld,
    hf: *const RPhysHeightfieldDesc,
    origin_x: f32,
    origin_z: f32,
    y_offset: f32,
) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() || hf.is_null() {
            set_last_error("rphys_world_set_heightfield_i16: null arg");
            return RPHYS_ERR_NULL;
        }

        let need = core::mem::size_of::<RPhysHeightfieldDesc>() as u32;
        let d = unsafe { &*hf };

        if d.size_bytes < need {
            set_last_error("rphys_world_set_heightfield_i16: size_bytes too small");
            return RPHYS_ERR_BAD_ARG;
        }
        if d.width == 0 || d.height == 0 {
            set_last_error("rphys_world_set_heightfield_i16: width/height == 0");
            return RPHYS_ERR_BAD_ARG;
        }
        if d.heights.is_null() {
            set_last_error("rphys_world_set_heightfield_i16: heights is null");
            return RPHYS_ERR_NULL;
        }
        if !(d.cell_size_x.is_finite()
            && d.cell_size_z.is_finite()
            && d.cell_size_x > 0.0
            && d.cell_size_z > 0.0)
        {
            set_last_error("rphys_world_set_heightfield_i16: bad cell sizes");
            return RPHYS_ERR_BAD_ARG;
        }
        if !(d.height_scale.is_finite() && d.height_offset.is_finite()) {
            set_last_error("rphys_world_set_heightfield_i16: height_scale/height_offset not finite");
            return RPHYS_ERR_BAD_ARG;
        }
        if !(origin_x.is_finite() && origin_z.is_finite() && y_offset.is_finite()) {
            set_last_error("rphys_world_set_heightfield_i16: origin/y_offset not finite");
            return RPHYS_ERR_BAD_ARG;
        }

        let stride = if d.row_stride == 0 { d.width } else { d.row_stride };
        if stride < d.width {
            set_last_error("rphys_world_set_heightfield_i16: row_stride < width");
            return RPHYS_ERR_BAD_ARG;
        }

        let count = match (stride as usize).checked_mul(d.height as usize) {
            Some(v) => v,
            None => {
                set_last_error("rphys_world_set_heightfield_i16: stride*height overflow");
                return RPHYS_ERR_BAD_ARG;
            }
        };

        let src = unsafe { core::slice::from_raw_parts(d.heights, count) };
        let cell = glam::Vec2::new(d.cell_size_x, d.cell_size_z);

        let hf = match riftphys_terrain::HeightField::from_i16_grid_strided(
            d.width,
            d.height,
            stride,
            cell,
            src,
            d.height_scale,
            d.height_offset,
        ) {
            Ok(hf) => hf,
            Err(e) => {
                set_last_error(&format!("rphys_world_set_heightfield_i16: {e}"));
                return RPHYS_ERR_BAD_ARG;
            }
        };

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        st.world.set_heightfield_at(hf, origin_x, origin_z, y_offset);

        let _ = d.flags;
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_set_heightfield_raw32_square(
    world: *mut RPhysWorld,
    bytes: *const u8,
    bytes_len: u32,
    world_size_x: f32,
    world_size_z: f32,
    origin_x: f32,
    origin_z: f32,
    y_offset: f32,
) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() || bytes.is_null() {
            set_last_error("rphys_world_set_heightfield_raw32_square: null arg");
            return RPHYS_ERR_NULL;
        }
        if bytes_len == 0 {
            set_last_error("rphys_world_set_heightfield_raw32_square: bytes_len == 0");
            return RPHYS_ERR_BAD_ARG;
        }
        if !(origin_x.is_finite() && origin_z.is_finite() && y_offset.is_finite()) {
            set_last_error("rphys_world_set_heightfield_raw32_square: origin/y_offset not finite");
            return RPHYS_ERR_BAD_ARG;
        }
        if !(world_size_x.is_finite() && world_size_z.is_finite()) {
            set_last_error("rphys_world_set_heightfield_raw32_square: world_size_x/z not finite");
            return RPHYS_ERR_BAD_ARG;
        }
        if !(world_size_x > 0.0 && world_size_z > 0.0) {
            set_last_error("rphys_world_set_heightfield_raw32_square: world_size_x/z must be > 0");
            return RPHYS_ERR_BAD_ARG;
        }

        let data = unsafe { core::slice::from_raw_parts(bytes, bytes_len as usize) };

        let hf = match riftphys_terrain::HeightField::from_raw32_square(
            data,
            glam::Vec2::new(world_size_x, world_size_z),
        ) {
            Ok(hf) => hf,
            Err(e) => {
                set_last_error(&format!("heightfield raw32 parse failed: {e}"));
                return RPHYS_ERR_BAD_ARG;
            }
        };

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };

        st.world.set_heightfield_at(hf, origin_x, origin_z, y_offset);
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_clear_heightfield(world: *mut RPhysWorld) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() {
            set_last_error("rphys_world_clear_heightfield: world is null");
            return RPHYS_ERR_NULL;
        }
        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let mut st = match w.state.lock() {
            Ok(g) => g,
            Err(poison) => poison.into_inner(),
        };
        st.world.clear_heightfield();
        RPHYS_OK
    })
}
