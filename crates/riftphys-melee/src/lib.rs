use riftphys_core::types::Vec3;
use riftphys_geom::Aabb;

/* ---------- existing sphere sweep helpers ---------- */
fn ray_aabb_slab(origin: Vec3, dir: Vec3, aabb: &Aabb) -> Option<(f32, Vec3)> {
    let inv = Vec3::new(
        if origin.x == origin.x && dir.x.abs() > 1e-9 { 1.0 / dir.x } else { 1.0e9 },
        if origin.y == origin.y && dir.y.abs() > 1e-9 { 1.0 / dir.y } else { 1.0e9 },
        if origin.z == origin.z && dir.z.abs() > 1e-9 { 1.0 / dir.z } else { 1.0e9 },
    );
    let t1 = (aabb.min - origin) * inv;
    let t2 = (aabb.max - origin) * inv;
    let tmin = t1.min(t2);
    let tmax = t1.max(t2);
    let mut t_enter = tmin.x; let mut n = Vec3::new(if t1.x > t2.x { 1.0 } else { -1.0 },0.0,0.0);
    if tmin.y > t_enter { t_enter = tmin.y; n = Vec3::new(0.0, if t1.y > t2.y { 1.0 } else { -1.0 },0.0); }
    if tmin.z > t_enter { t_enter = tmin.z; n = Vec3::new(0.0,0.0, if t1.z > t2.z { 1.0 } else { -1.0 }); }
    let t_exit = tmax.x.min(tmax.y).min(tmax.z);
    if t_enter <= t_exit && t_exit >= 0.0 && (0.0..=1.0).contains(&t_enter) { Some((t_enter, n)) } else { None }
}
fn expand_aabb(aabb: &Aabb, r: f32) -> Aabb { let e = Vec3::splat(r); Aabb{min:aabb.min - e, max:aabb.max + e} }
pub fn sweep_sphere_vs_aabb(p0: Vec3, v: Vec3, r: f32, aabb: &Aabb, dt: f32) -> Option<(f32, Vec3)> {
    if dt <= 0.0 { return None; }
    let dir = v * dt;
    if dir.length_squared() < 1e-12 {
        let e = expand_aabb(aabb, r);
        return if p0.x>=e.min.x&&p0.x<=e.max.x && p0.y>=e.min.y&&p0.y<=e.max.y && p0.z>=e.min.z&&p0.z<=e.max.z {
            Some((0.0, Vec3::new(0.0,1.0,0.0)))
        } else { None };
    }
    ray_aabb_slab(p0, dir, &expand_aabb(aabb, r))
}

/* ---------- deterministic pick ---------- */
#[derive(Copy, Clone)]
pub struct SweepHit { pub toi:f32, pub normal:Vec3, pub target_index:usize, pub sample_kind:u8 }
fn pick_better(cur: Option<SweepHit>, cand: SweepHit) -> Option<SweepHit> {
    match cur {
        None => Some(cand),
        Some(b) => {
            if cand.toi < b.toi - 1e-9 { return Some(cand); }
            if (cand.toi - b.toi).abs() <= 1e-9 {
                if cand.target_index < b.target_index { return Some(cand); }
                if cand.target_index == b.target_index && cand.sample_kind < b.sample_kind { return Some(cand); }
            }
            Some(b)
        }
    }
}

/* ---------- blade (tip/mid spheres) ---------- */
pub fn sweep_blade_points(
    tip_p0: Vec3, tip_v: Vec3, tip_r: f32,
    mid_p0: Vec3, mid_v: Vec3, mid_r: f32,
    aabbs: &[Aabb], dt: f32,
) -> Option<SweepHit> {
    let mut best=None;
    for (i,a) in aabbs.iter().enumerate(){
        if let Some((t,n))=sweep_sphere_vs_aabb(tip_p0,tip_v,tip_r,a,dt){ best=pick_better(best,SweepHit{toi:t,normal:n,target_index:i,sample_kind:0});}
        if let Some((t,n))=sweep_sphere_vs_aabb(mid_p0,mid_v,mid_r,a,dt){ best=pick_better(best,SweepHit{toi:t,normal:n,target_index:i,sample_kind:1});}
    }
    best
}

/* ---------- NEW: capsule tip+base sweep against AABB set ---------- */
pub fn sweep_capsule_vs_aabbs(
    p_top0: Vec3, p_bot0: Vec3, v: Vec3, radius: f32,
    aabbs: &[Aabb], dt: f32
) -> Option<SweepHit> {
    let mut best=None;
    for (i,a) in aabbs.iter().enumerate(){
        if let Some((t,n))=sweep_sphere_vs_aabb(p_top0, v, radius, a, dt){
            best=pick_better(best, SweepHit{toi:t,normal:n,target_index:i,sample_kind:0});
        }
        if let Some((t,n))=sweep_sphere_vs_aabb(p_bot0, v, radius, a, dt){
            best=pick_better(best, SweepHit{toi:t,normal:n,target_index:i,sample_kind:1});
        }
    }
    best
}
// --- NEW: capsule sweep via two-sphere method (tip & base) ------------------

pub struct CapSweepHit {
    pub toi: f32,
    pub normal: Vec3,
    pub target_index: usize,
    /// 0 = tip sphere, 1 = base sphere (deterministic tie-break)
    pub sample_kind: u8,
}

fn pick_cap_better(cur: Option<CapSweepHit>, cand: CapSweepHit) -> Option<CapSweepHit> {
    match cur {
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

/// tip/base sphere sweep against many AABBs; returns earliest hit with stable tiebreak
pub fn sweep_capsule_vs_aabb_two_spheres(
    tip_p0: Vec3, tip_v: Vec3, r_tip: f32,
    base_p0: Vec3, base_v: Vec3, r_base: f32,
    aabbs: &[Aabb],
    dt: f32,
) -> Option<CapSweepHit> {
    let mut best: Option<CapSweepHit> = None;
    for (i, aabb) in aabbs.iter().enumerate() {
        if let Some((t, n)) = sweep_sphere_vs_aabb(tip_p0, tip_v, r_tip, aabb, dt) {
            best = pick_cap_better(best, CapSweepHit { toi: t, normal: n, target_index: i, sample_kind: 0 });
        }
        if let Some((t, n)) = sweep_sphere_vs_aabb(base_p0, base_v, r_base, aabb, dt) {
            best = pick_cap_better(best, CapSweepHit { toi: t, normal: n, target_index: i, sample_kind: 1 });
        }
    }
    best
}
