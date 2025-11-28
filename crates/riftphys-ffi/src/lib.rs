// SPDX-License-Identifier: MIT or your internal license
#![allow(clippy::missing_safety_doc)]

pub mod tests;

use std::ffi::c_int;
use std::ptr;

use riftphys_core::{vec3, iso, Velocity, BodyId};
use riftphys_world::{world};
use riftphys_geom::{Shape, MassProps, Material};

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
pub enum RPhysBodyType {
    RPHYS_BODY_STATIC = 0,
    RPHYS_BODY_DYNAMIC = 1,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysCapsuleBodyDesc {
    pub pose: RPhysIsometry,
    pub vel:  RPhysVelocity,
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
    pub vel:  RPhysVelocity,
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
    pub vel:  RPhysVelocity,
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
    pub dir:    RPhysVec3,
    pub max_distance: f32,
    pub ignore_body: u32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysRayHit {
    pub hit: c_int,
    pub body_id: u32,
    pub fraction: f32,
    pub point: RPhysVec3,
    pub normal: RPhysVec3,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysCapsuleSweepQuery {
    pub from: RPhysVec3,
    pub to:   RPhysVec3,
    pub radius: f32,
    pub half_height: f32,
    pub ignore_body: u32,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RPhysCapsuleSweepHit {
    pub hit: c_int,
    pub body_id: u32,
    pub fraction: f32,
    pub started_overlapping: c_int,
    pub point: RPhysVec3,
    pub normal: RPhysVec3,
}


// -------- handles --------

pub struct RPhysWorld {
    inner: world::World,
}

impl RPhysWorld {
    fn from_raw<'a>(ptr: *mut RPhysWorld) -> &'a mut RPhysWorld {
        assert!(!ptr.is_null());
        unsafe { &mut *ptr }
    }
}

// -------- conversions --------

fn v3_from_ffi(v: RPhysVec3) -> riftphys_core::Vec3 {
    vec3(v.x, v.y, v.z)
}
fn v3_to_ffi(v: riftphys_core::Vec3) -> RPhysVec3 {
    RPhysVec3 { x: v.x, y: v.y, z: v.z }
}

fn q_from_ffi(q: RPhysQuat) -> riftphys_core::Quat {
    riftphys_core::Quat::from_xyzw(q.x, q.y, q.z, q.w).normalize()
}
fn q_to_ffi(q: riftphys_core::Quat) -> RPhysQuat {
    RPhysQuat { x: q.x, y: q.y, z: q.z, w: q.w }
}

fn iso_from_ffi(p: &RPhysIsometry) -> riftphys_core::Isometry {
    iso(v3_from_ffi(p.pos), q_from_ffi(p.rot))
}
fn iso_to_ffi(p: &riftphys_core::Isometry) -> RPhysIsometry {
    RPhysIsometry { pos: v3_to_ffi(p.pos), rot: q_to_ffi(p.rot) }
}

fn vel_from_ffi(v: &RPhysVelocity) -> Velocity {
    Velocity {
        lin: v3_from_ffi(v.lin),
        ang: v3_from_ffi(v.ang),
    }
}
fn vel_to_ffi(v: &Velocity) -> RPhysVelocity {
    RPhysVelocity {
        lin: v3_to_ffi(v.lin),
        ang: v3_to_ffi(v.ang),
    }
}

fn mat_from_ffi(m: &RPhysMaterial) -> Material {
    let mut out = Material::default();
    out.mu_s = m.mu_static;
    out.mu_k = m.mu_dynamic;
    out.restitution = m.restitution;
    out
}

// -------- world lifecycle --------

#[no_mangle]
pub unsafe extern "C" fn rphys_world_create(
    max_bodies: u32,
    max_colliders: u32,
) -> *mut RPhysWorld {
    let world = world::WorldBuilder::new()
        .with_capacity(max_bodies as usize, max_colliders as usize)
        .build();
    let wrapper = RPhysWorld { inner: world };
    Box::into_raw(Box::new(wrapper))
}


#[no_mangle]
pub unsafe extern "C" fn rphys_world_destroy(world: *mut RPhysWorld) {
    if !world.is_null() {
        let _ = Box::from_raw(world);
    }
}

// Optional: seed + epoch setters if you want parity with harness.

#[no_mangle]
pub unsafe extern "C" fn rphys_world_set_epoch(world: *mut RPhysWorld, epoch: u64) {
    let w = RPhysWorld::from_raw(world);
    w.inner.set_epoch(epoch);
}


#[no_mangle]
pub unsafe extern "C" fn rphys_world_set_rng_seed(world: *mut RPhysWorld, seed: u64) {
    let w = RPhysWorld::from_raw(world);
    w.inner.set_rng_seed(seed);
}

// -------- body creation --------

#[no_mangle]
pub unsafe extern "C" fn rphys_add_capsule_body(
    world: *mut RPhysWorld,
    desc: *const RPhysCapsuleBodyDesc,
) -> u32 {
    let w = RPhysWorld::from_raw(world);
    let desc = &*desc;

    let dyn_flag = matches!(desc.body_type, RPhysBodyType::RPHYS_BODY_DYNAMIC);
    let mass_props = if dyn_flag && desc.mass > 0.0 {
        MassProps::from_capsule(desc.radius, desc.half_height, desc.mass)
    } else {
        MassProps::infinite()
    };

    let body = w.inner.add_body(
        iso_from_ffi(&desc.pose),
        vel_from_ffi(&desc.vel),
        mass_props,
        dyn_flag,
    );
    let mat = mat_from_ffi(&desc.material);
    w.inner.add_collider(body, Shape::Capsule { r: desc.radius, hh: desc.half_height }, mat);

    // If you want user_tag ↔ BodyId mapping, do it on the C++ side (map bid→eid).

    body.0
}

#[no_mangle]
pub unsafe extern "C" fn rphys_add_box_body(
    world: *mut RPhysWorld,
    desc: *const RPhysBoxBodyDesc,
) -> u32 {
    let w = RPhysWorld::from_raw(world);
    let desc = &*desc;

    let dyn_flag = matches!(desc.body_type, RPhysBodyType::RPHYS_BODY_DYNAMIC);
    let mass_props = if dyn_flag && desc.mass > 0.0 {
        MassProps::from_box(vec3(desc.hx, desc.hy, desc.hz), desc.mass)
    } else {
        MassProps::infinite()
    };

    let body = w.inner.add_body(
        iso_from_ffi(&desc.pose),
        vel_from_ffi(&desc.vel),
        mass_props,
        dyn_flag,
    );
    let mat = mat_from_ffi(&desc.material);
    w.inner.add_collider(body, Shape::Box { hx: desc.hx, hy: desc.hy, hz: desc.hz }, mat);

    body.0
}

#[no_mangle]
pub unsafe extern "C" fn rphys_add_sphere_body(
    world: *mut RPhysWorld,
    desc: *const RPhysSphereBodyDesc,
) -> u32 {
    let w = RPhysWorld::from_raw(world);
    let desc = &*desc;

    let dyn_flag = matches!(desc.body_type, RPhysBodyType::RPHYS_BODY_DYNAMIC);
    let mass_props = if dyn_flag && desc.mass > 0.0 {
        MassProps::from_sphere(desc.radius, desc.mass)
    } else {
        MassProps::infinite()
    };

    let body = w.inner.add_body(
        iso_from_ffi(&desc.pose),
        vel_from_ffi(&desc.vel),
        mass_props,
        dyn_flag,
    );
    let mat = mat_from_ffi(&desc.material);
    w.inner.add_collider(body, Shape::Sphere { r: desc.radius }, mat);

    body.0
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_remove(world: *mut RPhysWorld, body_id: u32) -> c_int {
    let w = RPhysWorld::from_raw(world);
    let bid = BodyId(body_id);
    if w.inner.remove_body(bid) {
        1
    } else {
        0
    }
}

// -------- pose / velocity --------

#[no_mangle]
pub unsafe extern "C" fn rphys_body_get_pose(
    world: *mut RPhysWorld,
    body_id: u32,
    out_pose: *mut RPhysIsometry,
) -> c_int {
    if out_pose.is_null() {
        return 0;
    }
    let w = RPhysWorld::from_raw(world);
    let bid = BodyId(body_id);
    let pose = w.inner.get_body_pose(bid);
    ptr::write(out_pose, iso_to_ffi(&pose));
    1
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_set_pose(
    world: *mut RPhysWorld,
    body_id: u32,
    pose: *const RPhysIsometry,
) -> c_int {
    let w = RPhysWorld::from_raw(world);
    let bid = BodyId(body_id);
    let pose = iso_from_ffi(&*pose);
    w.inner.set_body_pose(bid, pose);
    1
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_get_velocity(
    world: *mut RPhysWorld,
    body_id: u32,
    out_vel: *mut RPhysVelocity,
) -> c_int {
    if out_vel.is_null() {
        return 0;
    }
    let w = RPhysWorld::from_raw(world);
    let bid = BodyId(body_id);
    let vel = w.inner.get_body_vel(bid);
    ptr::write(out_vel, vel_to_ffi(&vel));
    1
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_set_velocity(
    world: *mut RPhysWorld,
    body_id: u32,
    vel: *const RPhysVelocity,
) -> c_int {
    let w = RPhysWorld::from_raw(world);
    let bid = BodyId(body_id);
    let vel = vel_from_ffi(&*vel);
    w.inner.set_body_vel(bid, vel);
    1
}

#[no_mangle]
pub unsafe extern "C" fn rphys_body_apply_impulse(
    world: *mut RPhysWorld,
    body_id: u32,
    impulse: RPhysVec3,
) -> c_int {
    let w = RPhysWorld::from_raw(world);
    let bid = BodyId(body_id);
    w.inner.apply_impulse(bid, v3_from_ffi(impulse));
    1
}

// -------- step --------

#[no_mangle]
pub unsafe extern "C" fn rphys_world_step(world: *mut RPhysWorld, dt: f32) {
    let w = RPhysWorld::from_raw(world);
    let _ = w.inner.step(dt);
}
#[no_mangle]
pub unsafe extern "C" fn rphys_world_raycast(
    world: *mut RPhysWorld,
    query: *const RPhysRayQuery,
    out_hit: *mut RPhysRayHit,
) {
    let w = RPhysWorld::from_raw(world);
    let q = &*query;
    let origin = v3_from_ffi(q.origin);
    let dir    = v3_from_ffi(q.dir);
    let max_d  = q.max_distance.max(0.0);

    let ignore = if q.ignore_body != 0 { Some(BodyId(q.ignore_body)) } else { None };

    // You need to implement this on World:
    // fn raycast(&self, origin: Vec3, dir: Vec3, max_dist: f32, ignore: Option<BodyId>) -> Option<RayHit>
    if let Some(hit) = w.inner.raycast(origin, dir, max_d, ignore) {
        if !out_hit.is_null() {
            (*out_hit).hit = 1;
            (*out_hit).body_id = hit.body.0;
            (*out_hit).fraction = hit.toi;
            (*out_hit).point = v3_to_ffi(hit.point);
            (*out_hit).normal = v3_to_ffi(hit.normal);
        }
    } else if !out_hit.is_null() {
        (*out_hit).hit = 0;
        (*out_hit).body_id = 0;
        (*out_hit).fraction = 0.0;
        (*out_hit).point = RPhysVec3{ x:0.0, y:0.0, z:0.0 };
        (*out_hit).normal = RPhysVec3{ x:0.0, y:1.0, z:0.0 };
    }
}

#[no_mangle]
pub unsafe extern "C" fn rphys_world_capsule_sweep(
    world: *mut RPhysWorld,
    query: *const RPhysCapsuleSweepQuery,
    out_hit: *mut RPhysCapsuleSweepHit,
) {
    let w = RPhysWorld::from_raw(world);
    let q = &*query;
    let from = v3_from_ffi(q.from);
    let to   = v3_from_ffi(q.to);
    let ignore = if q.ignore_body != 0 { Some(BodyId(q.ignore_body)) } else { None };

    // You implement this on World:
    // fn sweep_capsule(&self, from: Vec3, to: Vec3, r: f32, hh: f32, ignore: Option<BodyId>) -> Option<SweepHit>
    if let Some(hit) = w.inner.sweep_capsule(from, to, q.radius, q.half_height, ignore) {
        if !out_hit.is_null() {
            (*out_hit).hit = 1;
            (*out_hit).body_id = hit.body.0;
            (*out_hit).fraction = hit.toi;
            (*out_hit).started_overlapping = if hit.started_overlapping { 1 } else { 0 };
            (*out_hit).point = v3_to_ffi(hit.point);
            (*out_hit).normal = v3_to_ffi(hit.normal);
        }
    } else if !out_hit.is_null() {
        (*out_hit).hit = 0;
        (*out_hit).body_id = 0;
        (*out_hit).fraction = 0.0;
        (*out_hit).started_overlapping = 0;
        (*out_hit).point = RPhysVec3{ x:0.0, y:0.0, z:0.0 };
        (*out_hit).normal = RPhysVec3{ x:0.0, y:1.0, z:0.0 };
    }
}
