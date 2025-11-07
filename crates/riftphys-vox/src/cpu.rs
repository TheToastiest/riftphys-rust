use glam::{UVec3, Vec3};
use crate::{Ray, RayHit, VoxelChunk};

fn ray_aabb(o: Vec3, d: Vec3, mn: Vec3, mx: Vec3) -> Option<(f32, f32)> {
    let inv = Vec3::new(1.0 / d.x, 1.0 / d.y, 1.0 / d.z);
    let t0s = (mn - o) * inv;
    let t1s = (mx - o) * inv;

    // glam has no component-wise min/max methods taking another Vec3 in stable form,
    // so compute per-component explicitly.
    let tmin = Vec3::new(
        t0s.x.min(t1s.x),
        t0s.y.min(t1s.y),
        t0s.z.min(t1s.z),
    );
    let tmax = Vec3::new(
        t0s.x.max(t1s.x),
        t0s.y.max(t1s.y),
        t0s.z.max(t1s.z),
    );

    let t0 = tmin.x.max(tmin.y).max(tmin.z);
    let t1 = tmax.x.min(tmax.y).min(tmax.z);
    (t1 >= t0).then_some((t0, t1))
}

/// Integer grid DDA (Amanatides & Woo)
pub fn raycast_dda(chunk: &VoxelChunk, ray: Ray) -> RayHit {
    let mut d = ray.d;
    let len = d.length();
    if len < 1.0e-12 {
        return RayHit::default();
    }
    d /= len;

    let s = chunk.voxel_size;
    let mn = chunk.aabb_min();
    let mx = chunk.aabb_max();

    let (mut t, t_exit) = match ray_aabb(ray.o, d, mn, mx) {
        Some(p) => p,
        None => return RayHit::default(),
    };

    if t < 0.0 { t = 0.0; }
    if t > ray.tmax { return RayHit::default(); }

    // Entry point and starting cell
    let p_entry = ray.o + d * t;
    let mut cell = UVec3::new(
        (p_entry.x / s).floor() as u32,
        (p_entry.y / s).floor() as u32,
        (p_entry.z / s).floor() as u32,
    );

    // Clamp to grid if we hit exactly on the max face
    cell.x = cell.x.min(chunk.dims.x.saturating_sub(1));
    cell.y = cell.y.min(chunk.dims.y.saturating_sub(1));
    cell.z = cell.z.min(chunk.dims.z.saturating_sub(1));

    // Step direction per axis: -1 or +1
    let (sx, sy, sz) = (
        if d.x >= 0.0 { 1i32 } else { -1i32 },
        if d.y >= 0.0 { 1i32 } else { -1i32 },
        if d.z >= 0.0 { 1i32 } else { -1i32 },
    );

    // First crossing times
    let mut tmaxx = if d.x.abs() < 1.0e-12 {
        f32::INFINITY
    } else {
        let edge = ((cell.x as i32 + if sx > 0 { 1 } else { 0 }) as f32) * s;
        t + (edge - p_entry.x) / d.x
    };
    let mut tmaxy = if d.y.abs() < 1.0e-12 {
        f32::INFINITY
    } else {
        let edge = ((cell.y as i32 + if sy > 0 { 1 } else { 0 }) as f32) * s;
        t + (edge - p_entry.y) / d.y
    };
    let mut tmaxz = if d.z.abs() < 1.0e-12 {
        f32::INFINITY
    } else {
        let edge = ((cell.z as i32 + if sz > 0 { 1 } else { 0 }) as f32) * s;
        t + (edge - p_entry.z) / d.z
    };

    // Distance to advance each time we cross a voxel boundary on an axis
    let tdx = if d.x.abs() < 1.0e-12 { f32::INFINITY } else { s / d.x.abs() };
    let tdy = if d.y.abs() < 1.0e-12 { f32::INFINITY } else { s / d.y.abs() };
    let tdz = if d.z.abs() < 1.0e-12 { f32::INFINITY } else { s / d.z.abs() };

    let mut last_n = Vec3::ZERO;
    if chunk.get(cell.x, cell.y, cell.z) {
        if tmaxx <= tmaxy && tmaxx <= tmaxz {
            last_n = if sx > 0 { Vec3::NEG_X } else { Vec3::X };
        } else if tmaxy <= tmaxz {
            last_n = if sy > 0 { Vec3::NEG_Y } else { Vec3::Y };
        } else {
            last_n = if sz > 0 { Vec3::NEG_Z } else { Vec3::Z };
        }
        return RayHit { hit: true, t, normal: last_n, cell };
    }
    loop {
        if cell.x >= chunk.dims.x || cell.y >= chunk.dims.y || cell.z >= chunk.dims.z {
            return RayHit::default();
        }
        if chunk.get(cell.x, cell.y, cell.z) {
            return RayHit { hit: true, t, normal: last_n, cell };
        }

        // Advance to the next boundary
        if tmaxx <= tmaxy && tmaxx <= tmaxz {
            if tmaxx > t_exit || tmaxx > ray.tmax { return RayHit::default(); }
            t = tmaxx; tmaxx += tdx;
            last_n = if sx > 0 { Vec3::new(-1.0, 0.0, 0.0) } else { Vec3::new(1.0, 0.0, 0.0) };
            cell.x = (cell.x as i32 + sx) as u32;
        } else if tmaxy <= tmaxz {
            if tmaxy > t_exit || tmaxy > ray.tmax { return RayHit::default(); }
            t = tmaxy; tmaxy += tdy;
            last_n = if sy > 0 { Vec3::new(0.0, -1.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
            cell.y = (cell.y as i32 + sy) as u32;
        } else {
            if tmaxz > t_exit || tmaxz > ray.tmax { return RayHit::default(); }
            t = tmaxz; tmaxz += tdz;
            last_n = if sz > 0 { Vec3::new(0.0, 0.0, -1.0) } else { Vec3::new(0.0, 0.0, 1.0) };
            cell.z = (cell.z as i32 + sz) as u32;
        }
    }
}

pub fn raycast_many(chunk: &VoxelChunk, rays: &[Ray], out: &mut [RayHit]) {
    assert_eq!(rays.len(), out.len());
    for (i, r) in rays.iter().enumerate() {
        out[i] = raycast_dda(chunk, *r);
    }
}
