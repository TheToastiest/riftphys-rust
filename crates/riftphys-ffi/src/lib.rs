// SPDX-License-Identifier: MIT or your internal license
#![allow(clippy::missing_safety_doc)]

mod tests;

use std::cell::RefCell;
use std::ffi::c_char;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;

use riftphys_core::{iso, vec3, BodyId, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::*;
use riftphys_world::world;

/* ===================== ABI CONTRACT =====================
   - Body IDs exposed via FFI are 1-based (never 0).
   - 0 is reserved as "invalid / none".
   - Any API accepting body_id treats 0 as invalid and returns RPHYS_ERR_INVALID_ID.
   - ignore_body fields are also 1-based; 0 means "no ignore".

   ABI v1: RPhysBool/RPhysResult are u32.
   ======================================================= */

pub type RPhysBool = u32;
pub type RPhysResult = u32;

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

pub const RPHYS_STEP_HASH_BYTES: usize = 32;

thread_local! {
    static LAST_ERROR: RefCell<Option<std::ffi::CString>> = const { RefCell::new(None) };
}

#[inline]
fn set_last_error(msg: &str) {
    // TLS error message, valid until next error set on same thread.
    let s = msg.to_string();
    LAST_ERROR.with(|cell| {
        *cell.borrow_mut() = std::ffi::CString::new(s).ok();
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
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(r) => r,
        Err(_) => {
            set_last_error("panic crossed FFI boundary");
            RPHYS_ERR_INTERNAL
        }
    }
}

#[inline]
fn body_to_ffi(id: BodyId) -> u32 {
    id.0.wrapping_add(1) // never 0
}
#[inline]
fn body_from_ffi(id: u32) -> Option<BodyId> {
    if id == 0 { None } else { Some(BodyId(id - 1)) }
}

/* ===================== FFI TYPES ===================== */

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

// ABI: body_type values (mirrors C header)
#[repr(u32)]
#[derive(Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum RPhysBodyType {
    RPHYS_BODY_STATIC = 0,
    RPHYS_BODY_DYNAMIC = 1,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysCapsuleBodyDesc {
    pub pose: RPhysIsometry,
    pub vel: RPhysVelocity,
    pub radius: f32,
    pub half_height: f32,
    pub mass: f32, // ABI kept; currently ignored (mass derives from density table + shape)
    pub body_type: RPhysBodyType,
    pub material: RPhysMaterial,
    pub user_tag: u32, // ABI kept; currently unused
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysBoxBodyDesc {
    pub pose: RPhysIsometry,
    pub vel: RPhysVelocity,
    pub hx: f32,
    pub hy: f32,
    pub hz: f32,
    pub mass: f32, // ABI kept; currently ignored
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
    pub mass: f32, // ABI kept; currently ignored
    pub body_type: RPhysBodyType,
    pub material: RPhysMaterial,
    pub user_tag: u32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysRayQuery {
    pub origin: RPhysVec3,
    pub dir: RPhysVec3, // normalized by engine/FFI
    pub max_distance: f32,
    pub ignore_body: u32, // 1-based; 0 = none
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysRayHit {
    pub hit: RPhysBool,
    pub body_id: u32, // 1-based; 0 = none
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
    pub body_id: u32, // 1-based; 0 = none
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
    pub size_bytes: u32,    // must be set by caller = sizeof(RPhysStepParams)
    pub flags: u32,         // reserved for future
    pub substeps: u32,      // 1 = normal
    pub solver_iters: u32,  // 0 = engine default (currently ignored by FFI)
    pub dt: f32,
}

/* ===================== HANDLE ===================== */

pub struct RPhysWorld {
    inner: world::World,
}

impl RPhysWorld {
    #[inline]
    unsafe fn from_raw_mut<'a>(ptr: *mut RPhysWorld) -> &'a mut RPhysWorld {
        &mut *ptr
    }
    #[inline]
    unsafe fn from_raw_ref<'a>(ptr: *const RPhysWorld) -> &'a RPhysWorld {
        &*ptr
    }
}

/* ===================== CONVERSIONS ===================== */

#[inline]
fn v3_from_ffi(v: RPhysVec3) -> riftphys_core::Vec3 {
    vec3(v.x, v.y, v.z)
}
#[inline]
fn v3_to_ffi(v: riftphys_core::Vec3) -> RPhysVec3 {
    RPhysVec3 { x: v.x, y: v.y, z: v.z }
}

#[inline]
fn q_from_ffi(q: RPhysQuat) -> riftphys_core::Quat {
    riftphys_core::Quat::from_xyzw(q.x, q.y, q.z, q.w).normalize()
}
#[inline]
fn q_to_ffi(q: riftphys_core::Quat) -> RPhysQuat {
    RPhysQuat { x: q.x, y: q.y, z: q.z, w: q.w }
}

#[inline]
fn iso_from_ffi(p: &RPhysIsometry) -> riftphys_core::Isometry {
    iso(v3_from_ffi(p.pos), q_from_ffi(p.rot))
}
#[inline]
fn iso_to_ffi(p: &riftphys_core::Isometry) -> RPhysIsometry {
    RPhysIsometry { pos: v3_to_ffi(p.pos), rot: q_to_ffi(p.rot) }
}

#[inline]
fn vel_from_ffi(v: &RPhysVelocity) -> Velocity {
    Velocity { lin: v3_from_ffi(v.lin), ang: v3_from_ffi(v.ang) }
}
#[inline]
fn vel_to_ffi(v: &Velocity) -> RPhysVelocity {
    RPhysVelocity { lin: v3_to_ffi(v.lin), ang: v3_to_ffi(v.ang) }
}

#[inline]
fn mat_from_ffi(m: &RPhysMaterial) -> Material {
    // ABI behavior: clamp + quantize. Changing quantize() granularity impacts determinism hashes.
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

#[inline]
fn normalize_dir(v: riftphys_core::Vec3) -> riftphys_core::Vec3 {
    // Deterministic-ish normalization; engine should ideally own this rule.
    let lsq = v.x * v.x + v.y * v.y + v.z * v.z;
    if lsq > 0.0 {
        let inv_len = 1.0 / lsq.sqrt();
        vec3(v.x * inv_len, v.y * inv_len, v.z * inv_len)
    } else {
        v
    }
}

#[inline]
unsafe fn write_raycast_miss(out_hit: *mut RPhysRayHit) {
    if out_hit.is_null() { return; }
    (*out_hit).hit = RPHYS_FALSE;
    (*out_hit).body_id = RPHYS_INVALID_BODY_ID;
    (*out_hit).fraction = 0.0;
    (*out_hit).point = RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 };
    (*out_hit).normal = RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 };
}

#[inline]
unsafe fn write_sweep_miss(out_hit: *mut RPhysCapsuleSweepHit) {
    if out_hit.is_null() { return; }
    (*out_hit).hit = RPHYS_FALSE;
    (*out_hit).body_id = RPHYS_INVALID_BODY_ID;
    (*out_hit).fraction = 0.0;
    (*out_hit).started_overlapping = RPHYS_FALSE;
    (*out_hit).point = RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 };
    (*out_hit).normal = RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 };
}

/* ===================== WORLD LIFECYCLE ===================== */

#[no_mangle]
pub unsafe extern "C" fn rphys_world_create(max_bodies: u32, max_colliders: u32) -> *mut RPhysWorld {
    // Return null on failure; never panic across FFI.
    match catch_unwind(AssertUnwindSafe(|| {
        let w = world::WorldBuilder::new()
            .with_capacity(max_bodies as usize, max_colliders as usize)
            .build();
        Box::into_raw(Box::new(RPhysWorld { inner: w }))
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
        w.inner.set_epoch(epoch);
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
        w.inner.set_rng_seed(seed);
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
            *out_body_id = RPHYS_INVALID_BODY_ID;
        }
        if world.is_null() || desc.is_null() || out_body_id.is_null() {
            set_last_error("rphys_add_capsule_body: null arg");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let d = unsafe { &*desc };

        let dyn_flag = matches!(d.body_type, RPhysBodyType::RPHYS_BODY_DYNAMIC);
        let mass_props = if dyn_flag {
            MassProps::from_capsule(d.radius, d.half_height, MaterialId::Default)
        } else {
            MassProps::infinite()
        };

        let body = w.inner.add_body(iso_from_ffi(&d.pose), vel_from_ffi(&d.vel), mass_props, dyn_flag);
        let mat = mat_from_ffi(&d.material);
        w.inner.add_collider(body, Shape::Capsule { r: d.radius, hh: d.half_height }, mat);

        unsafe { *out_body_id = body_to_ffi(body) };
        RPHYS_OK
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
            *out_body_id = RPHYS_INVALID_BODY_ID;
        }
        if world.is_null() || desc.is_null() || out_body_id.is_null() {
            set_last_error("rphys_add_box_body: null arg");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let d = unsafe { &*desc };

        let dyn_flag = matches!(d.body_type, RPhysBodyType::RPHYS_BODY_DYNAMIC);
        let mass_props = if dyn_flag {
            MassProps::from_box(vec3(d.hx, d.hy, d.hz), MaterialId::Default)
        } else {
            MassProps::infinite()
        };

        let body = w.inner.add_body(iso_from_ffi(&d.pose), vel_from_ffi(&d.vel), mass_props, dyn_flag);
        let mat = mat_from_ffi(&d.material);
        w.inner.add_collider(body, Shape::Box { hx: d.hx, hy: d.hy, hz: d.hz }, mat);

        unsafe { *out_body_id = body_to_ffi(body) };
        RPHYS_OK
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
            *out_body_id = RPHYS_INVALID_BODY_ID;
        }
        if world.is_null() || desc.is_null() || out_body_id.is_null() {
            set_last_error("rphys_add_sphere_body: null arg");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        let d = unsafe { &*desc };

        let dyn_flag = matches!(d.body_type, RPhysBodyType::RPHYS_BODY_DYNAMIC);
        let mass_props = if dyn_flag {
            MassProps::from_sphere(d.radius, MaterialId::Default)
        } else {
            MassProps::infinite()
        };

        let body = w.inner.add_body(iso_from_ffi(&d.pose), vel_from_ffi(&d.vel), mass_props, dyn_flag);
        let mat = mat_from_ffi(&d.material);
        w.inner.add_collider(body, Shape::Sphere { r: d.radius }, mat);

        unsafe { *out_body_id = body_to_ffi(body) };
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_remove(world: *mut RPhysWorld, body_id: u32) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() {
            set_last_error("rphys_body_remove: world is null");
            return RPHYS_ERR_NULL;
        }
        let Some(bid) = body_from_ffi(body_id) else {
            set_last_error("rphys_body_remove: invalid body_id");
            return RPHYS_ERR_INVALID_ID;
        };

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        if w.inner.remove_body(bid) {
            RPHYS_OK
        } else {
            set_last_error("rphys_body_remove: id not found");
            RPHYS_ERR_INVALID_ID
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
        if world.is_null() {
            set_last_error("rphys_body_get_pose: world is null");
            return RPHYS_ERR_NULL;
        }
        let Some(bid) = body_from_ffi(body_id) else {
            set_last_error("rphys_body_get_pose: invalid body_id");
            return RPHYS_ERR_INVALID_ID;
        };

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let pose = w.inner.get_body_pose(bid);
        unsafe { ptr::write(out_pose, iso_to_ffi(&pose)); }
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
        let Some(bid) = body_from_ffi(body_id) else {
            set_last_error("rphys_body_set_pose: invalid body_id");
            return RPHYS_ERR_INVALID_ID;
        };

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        w.inner.set_body_pose(bid, iso_from_ffi(unsafe { &*pose }));
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
        if world.is_null() {
            set_last_error("rphys_body_get_velocity: world is null");
            return RPHYS_ERR_NULL;
        }
        let Some(bid) = body_from_ffi(body_id) else {
            set_last_error("rphys_body_get_velocity: invalid body_id");
            return RPHYS_ERR_INVALID_ID;
        };

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let vel = w.inner.get_body_vel(bid);
        unsafe { ptr::write(out_vel, vel_to_ffi(&vel)); }
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
        let Some(bid) = body_from_ffi(body_id) else {
            set_last_error("rphys_body_set_velocity: invalid body_id");
            return RPHYS_ERR_INVALID_ID;
        };

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        w.inner.set_body_vel(bid, vel_from_ffi(unsafe { &*vel }));
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
        let Some(bid) = body_from_ffi(body_id) else {
            set_last_error("rphys_body_apply_impulse: invalid body_id");
            return RPHYS_ERR_INVALID_ID;
        };

        let w = unsafe { RPhysWorld::from_raw_mut(world) };
        w.inner.apply_impulse(bid, v3_from_ffi(impulse));
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
        let need = std::mem::size_of::<RPhysStepParams>() as u32;
        if params.size_bytes < need {
            set_last_error("rphys_world_step_ex: size_bytes too small");
            return RPHYS_ERR_BAD_ARG;
        }

        if !params.dt.is_finite() {
            set_last_error("rphys_world_step_ex: dt is not finite");
            return RPHYS_ERR_BAD_ARG;
        }
        if params.substeps == 0 {
            set_last_error("rphys_world_step_ex: substeps == 0");
            return RPHYS_ERR_BAD_ARG;
        }

        let w = unsafe { RPhysWorld::from_raw_mut(world) };

        let sub_dt = params.dt / (params.substeps as f32);
        for _ in 0..params.substeps {
            w.inner.step(sub_dt);
        }

        // solver_iters/flags reserved: frozen in ABI, currently ignored by FFI wrapper.
        let _ = params.solver_iters;
        let _ = params.flags;

        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_step(world: *mut RPhysWorld, dt: f32) -> RPhysResult {
    let params = RPhysStepParams {
        size_bytes: std::mem::size_of::<RPhysStepParams>() as u32,
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
        if world.is_null() || out_tick.is_null() {
            set_last_error("rphys_world_tick_index: null arg");
            return RPHYS_ERR_NULL;
        }
        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        unsafe { *out_tick = w.inner.tick_index() as u64; }
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_num_bodies(world: *const RPhysWorld, out_num: *mut u32) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() || out_num.is_null() {
            set_last_error("rphys_world_num_bodies: null arg");
            return RPHYS_ERR_NULL;
        }
        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        unsafe { *out_num = w.inner.num_bodies() as u32; }
        RPHYS_OK
    })
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_get_step_hash(
    world: *const RPhysWorld,
    out_hash: *mut RPhysHash32,
) -> RPhysResult {
    ffi_guard(|| {
        if world.is_null() || out_hash.is_null() {
            set_last_error("rphys_world_get_step_hash: null arg");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let h = w.inner.step_hash(); // [u8; 32]

        // No slices. No autoref. Just write the POD struct.
        unsafe {
            ptr::write(out_hash, RPhysHash32 { bytes: h });
        }

        RPHYS_OK
    })
}


#[no_mangle]
pub unsafe extern "C" fn rphys_world_step_and_hash(world: *mut RPhysWorld, dt: f32, out_hash: *mut RPhysHash32) -> RPhysResult {
    ffi_guard(|| {
        if out_hash.is_null() {
            set_last_error("rphys_world_step_and_hash: out_hash is null");
            return RPHYS_ERR_NULL;
        }
        // Always write something deterministic on failure paths when possible.
        unsafe {
            ptr::write(out_hash, RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] });
        }
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
        // Always initialize output if possible.
        unsafe { write_raycast_miss(out_hit); }

        if world.is_null() || query.is_null() || out_hit.is_null() {
            set_last_error("rphys_world_raycast: null arg");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let q = unsafe { &*query };

        let origin = v3_from_ffi(q.origin);
        let dir = normalize_dir(v3_from_ffi(q.dir));
        let max_d = q.max_distance.max(0.0);

        let ignore = body_from_ffi(q.ignore_body);

        if let Some(hit) = w.inner.raycast(origin, dir, max_d, ignore) {
            unsafe {
                (*out_hit).hit = RPHYS_TRUE;
                (*out_hit).body_id = body_to_ffi(hit.body);
                (*out_hit).fraction = hit.toi;
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
        unsafe { write_sweep_miss(out_hit); }

        if world.is_null() || query.is_null() || out_hit.is_null() {
            set_last_error("rphys_world_capsule_sweep: null arg");
            return RPHYS_ERR_NULL;
        }

        let w = unsafe { RPhysWorld::from_raw_ref(world) };
        let q = unsafe { &*query };

        let from = v3_from_ffi(q.from);
        let to = v3_from_ffi(q.to);

        let ignore = body_from_ffi(q.ignore_body);

        if let Some(hit) = w.inner.sweep_capsule(from, to, q.radius, q.half_height, ignore) {
            unsafe {
                (*out_hit).hit = RPHYS_TRUE;
                (*out_hit).body_id = body_to_ffi(hit.body);
                (*out_hit).fraction = hit.toi;
                (*out_hit).started_overlapping = if hit.started_overlapping { RPHYS_TRUE } else { RPHYS_FALSE };
                (*out_hit).point = v3_to_ffi(hit.point);
                (*out_hit).normal = v3_to_ffi(hit.normal);
            }
        }

        RPHYS_OK
    })
}
