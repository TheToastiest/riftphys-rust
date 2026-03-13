use riftphys_core::types::Vec3;
use riftphys_geom::Aabb;

const TOI_EPS: f32 = 1e-9;

/// Query hit with deterministic tie-break ordering.
#[derive(Copy, Clone, Debug)]
pub struct Hit {
    pub toi: f32,
    pub normal: Vec3,
    pub target_index: usize,
    /// Deterministic secondary key (e.g. 0=tip, 1=base).
    pub sample_kind: u8,
}

#[inline]
fn pick_better(cur: Option<Hit>, cand: Hit) -> Option<Hit> {
    match cur {
        None => Some(cand),
        Some(b) => {
            if cand.toi < b.toi - TOI_EPS {
                return Some(cand);
            }
            if (cand.toi - b.toi).abs() <= TOI_EPS {
                if cand.target_index < b.target_index {
                    return Some(cand);
                }
                if cand.target_index == b.target_index && cand.sample_kind < b.sample_kind {
                    return Some(cand);
                }
            }
            Some(b)
        }
    }
}

#[inline]
fn expand_aabb(aabb: &Aabb, r: f32) -> Aabb {
    let e = Vec3::splat(r);
    Aabb { min: aabb.min - e, max: aabb.max + e }
}

#[inline]
fn ray_aabb_slab_unit(origin: Vec3, dir: Vec3, aabb: &Aabb) -> Option<(f32, Vec3)> {
    // NaN guard stays (origin.x == origin.x) and “huge inv” fallback keeps behavior stable.
    let inv = Vec3::new(
        if origin.x == origin.x && dir.x.abs() > 1e-9 { 1.0 / dir.x } else { 1.0e9 },
        if origin.y == origin.y && dir.y.abs() > 1e-9 { 1.0 / dir.y } else { 1.0e9 },
        if origin.z == origin.z && dir.z.abs() > 1e-9 { 1.0 / dir.z } else { 1.0e9 },
    );

    let t1 = (aabb.min - origin) * inv;
    let t2 = (aabb.max - origin) * inv;

    let tmin = t1.min(t2);
    let tmax = t1.max(t2);

    let mut t_enter = tmin.x;
    let mut n = Vec3::new(if t1.x > t2.x { 1.0 } else { -1.0 }, 0.0, 0.0);

    if tmin.y > t_enter {
        t_enter = tmin.y;
        n = Vec3::new(0.0, if t1.y > t2.y { 1.0 } else { -1.0 }, 0.0);
    }
    if tmin.z > t_enter {
        t_enter = tmin.z;
        n = Vec3::new(0.0, 0.0, if t1.z > t2.z { 1.0 } else { -1.0 });
    }

    let t_exit = tmax.x.min(tmax.y).min(tmax.z);

    if t_enter <= t_exit && t_exit >= 0.0 && (0.0..=1.0).contains(&t_enter) {
        Some((t_enter, n))
    } else {
        None
    }
}

/// Sweep a sphere against a single AABB over `dt`.
/// Returns `(toi, normal)` where `toi` is normalized to `[0..1]` along `v*dt`.
pub fn sweep_sphere_aabb(p0: Vec3, v: Vec3, r: f32, aabb: &Aabb, dt: f32) -> Option<(f32, Vec3)> {
    if dt <= 0.0 {
        return None;
    }

    let dir = v * dt;

    if dir.length_squared() < 1e-12 {
        let e = expand_aabb(aabb, r);
        let inside =
            p0.x >= e.min.x && p0.x <= e.max.x &&
                p0.y >= e.min.y && p0.y <= e.max.y &&
                p0.z >= e.min.z && p0.z <= e.max.z;

        return if inside {
            Some((0.0, Vec3::new(0.0, 1.0, 0.0)))
        } else {
            None
        };
    }

    ray_aabb_slab_unit(p0, dir, &expand_aabb(aabb, r))
}

/// Sweep one sphere against many AABBs; deterministic earliest hit.
pub fn sweep_sphere_aabbs(p0: Vec3, v: Vec3, r: f32, aabbs: &[Aabb], dt: f32) -> Option<Hit> {
    let mut best: Option<Hit> = None;
    for (i, a) in aabbs.iter().enumerate() {
        if let Some((t, n)) = sweep_sphere_aabb(p0, v, r, a, dt) {
            best = pick_better(best, Hit { toi: t, normal: n, target_index: i, sample_kind: 0 });
        }
    }
    best
}

/// Sweep two spheres against many AABBs; deterministic earliest hit.
/// `sample_kind` is 0 for sphere A, 1 for sphere B.
pub fn sweep_two_spheres_aabbs(
    a_p0: Vec3, a_v: Vec3, a_r: f32,
    b_p0: Vec3, b_v: Vec3, b_r: f32,
    aabbs: &[Aabb],
    dt: f32,
) -> Option<Hit> {
    let mut best: Option<Hit> = None;
    for (i, aabb) in aabbs.iter().enumerate() {
        if let Some((t, n)) = sweep_sphere_aabb(a_p0, a_v, a_r, aabb, dt) {
            best = pick_better(best, Hit { toi: t, normal: n, target_index: i, sample_kind: 0 });
        }
        if let Some((t, n)) = sweep_sphere_aabb(b_p0, b_v, b_r, aabb, dt) {
            best = pick_better(best, Hit { toi: t, normal: n, target_index: i, sample_kind: 1 });
        }
    }
    best
}

/// Capsule sweep approximation against many AABBs using tip+base spheres.
/// `sample_kind` 0 = top, 1 = bottom.
pub fn sweep_capsule_aabbs(
    p_top0: Vec3,
    p_bot0: Vec3,
    v: Vec3,
    radius: f32,
    aabbs: &[Aabb],
    dt: f32,
) -> Option<Hit> {
    sweep_two_spheres_aabbs(p_top0, v, radius, p_bot0, v, radius, aabbs, dt)
}

pub fn sweep_aabb_aabbs(
    source: Aabb,
    v: Vec3,
    targets: &[Aabb],
    dt: f32,
) -> Option<Hit> {
    if dt <= 0.0 { return None; }

    let dir = v * dt;
    let mut best: Option<Hit> = None;

    // Calculate source half-extents to expand targets
    let half_extents = (source.max - source.min) * 0.5;
    let center0 = source.min + half_extents;

    for (i, target) in targets.iter().enumerate() {
        // Expand the target AABB by the source's dimensions (Minkowski Sum)
        let expanded_target = Aabb {
            min: target.min - half_extents,
            max: target.max + half_extents,
        };

        // Reuse your existing high-stability slab raycaster
        if let Some((t, n)) = ray_aabb_slab_unit(center0, dir, &expanded_target) {
            best = pick_better(best, Hit {
                toi: t,
                normal: n,
                target_index: i,
                sample_kind: 0
            });
        }
    }
    best
}