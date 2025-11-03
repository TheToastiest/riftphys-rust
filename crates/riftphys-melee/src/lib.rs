use riftphys_core::types::Vec3;
use riftphys_geom::Aabb;

/// Ray vs AABB (slab) returning (t_enter, normal). t in [0,1] across the step.
/// Assumes dir != 0 on all axes; guard small eps for degenerate cases.
fn ray_aabb_slab(origin: Vec3, dir: Vec3, aabb: &Aabb) -> Option<(f32, Vec3)> {
    let inv = Vec3::new(
        if dir.x.abs() < 1e-9 { 1e9 } else { 1.0 / dir.x },
        if dir.y.abs() < 1e-9 { 1e9 } else { 1.0 / dir.y },
        if dir.z.abs() < 1e-9 { 1e9 } else { 1.0 / dir.z },
    );
    let t1 = (aabb.min - origin) * inv;
    let t2 = (aabb.max - origin) * inv;

    let tmin = t1.min(t2);
    let tmax = t1.max(t2);

    let mut t_enter = tmin.x;
    let mut normal = Vec3::new(if t1.x > t2.x { 1.0 } else { -1.0 }, 0.0, 0.0);

    if tmin.y > t_enter {
        t_enter = tmin.y;
        normal = Vec3::new(0.0, if t1.y > t2.y { 1.0 } else { -1.0 }, 0.0);
    }
    if tmin.z > t_enter {
        t_enter = tmin.z;
        normal = Vec3::new(0.0, 0.0, if t1.z > t2.z { 1.0 } else { -1.0 });
    }

    let t_exit = tmax.x.min(tmax.y).min(tmax.z);
    if t_enter <= t_exit && t_exit >= 0.0 && t_enter >= 0.0 && t_enter <= 1.0 {
        Some((t_enter, normal))
    } else {
        None
    }
}

/// Expand AABB by radius r in all directions.
fn expand_aabb(aabb: &Aabb, r: f32) -> Aabb {
    let e = Vec3::splat(r);
    Aabb { min: aabb.min - e, max: aabb.max + e }
}

/// Swept sphere (center p0, velocity v over dt, radius r) vs AABB.
/// Returns earliest TOI in [0,1] and a contact normal.
pub fn sweep_sphere_vs_aabb(p0: Vec3, v: Vec3, r: f32, aabb: &Aabb, dt: f32) -> Option<(f32, Vec3)> {
    if dt <= 0.0 { return None; }
    let dir = v * dt;
    if dir.length_squared() < 1e-12 {
        // treat as overlap check
        let e = expand_aabb(aabb, r);
        return if (p0.x >= e.min.x && p0.x <= e.max.x &&
            p0.y >= e.min.y && p0.y <= e.max.y &&
            p0.z >= e.min.z && p0.z <= e.max.z) {
            Some((0.0, Vec3::new(0.0, 1.0, 0.0))) // arbitrary normal
        } else { None };
    }
    let e = expand_aabb(aabb, r);
    ray_aabb_slab(p0, dir, &e)
}

/// Deterministic blade sweep using two sphere samples (tip & mid).
/// Returns earliest hit across all targets with stable tie-breaks.
pub struct SweepHit {
    pub toi: f32,          // earliest t in [0,1]
    pub normal: Vec3,
    pub target_index: usize,
    pub sample_kind: u8,   // 0=tip, 1=mid
}

pub fn sweep_blade_points(
    tip_p0: Vec3, tip_v: Vec3, tip_r: f32,
    mid_p0: Vec3, mid_v: Vec3, mid_r: f32,
    aabbs: &[Aabb],
    dt: f32,
) -> Option<SweepHit> {
    let mut best: Option<SweepHit> = None;
    for (i, aabb) in aabbs.iter().enumerate() {
        if let Some((t, n)) = sweep_sphere_vs_aabb(tip_p0, tip_v, tip_r, aabb, dt) {
            let cand = SweepHit { toi: t, normal: n, target_index: i, sample_kind: 0 };
            best = pick_better(best, cand);
        }
        if let Some((t, n)) = sweep_sphere_vs_aabb(mid_p0, mid_v, mid_r, aabb, dt) {
            let cand = SweepHit { toi: t, normal: n, target_index: i, sample_kind: 1 };
            best = pick_better(best, cand);
        }
    }
    best
}

/// Deterministic pick: smallest toi → then lowest target_index → then sample_kind.
fn pick_better(current: Option<SweepHit>, cand: SweepHit) -> Option<SweepHit> {
    match current {
        None => Some(cand),
        Some(b) => {
            if cand.toi < b.toi - 1e-9 { return Some(cand); }
            if (cand.toi - b.toi).abs() <= 1e-9 {
                if cand.target_index < b.target_index { return Some(cand); }
                if cand.target_index == b.target_index && cand.sample_kind < b.sample_kind {
                    return Some(cand);
                }
            }
            Some(b)
        }
    }
}
