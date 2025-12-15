// crates/riftphys-world/src/world/queries.rs

use super::geom::{closest_points_segment_aabb, ray_aabb};
use super::world::{RayHit, SweepHit, World};

use riftphys_core::{BodyId, Vec3};
use riftphys_geom::{Aabb, Shape};
use riftphys_query as query;

// Sentinel BodyId for terrain hits (NEVER pass into body APIs).
const TERRAIN_HIT_BODY: BodyId = BodyId(u32::MAX);

#[inline]
fn slab_range_1d(o: f32, d: f32, mn: f32, mx: f32, tmin: &mut f32, tmax: &mut f32) -> bool {
    let eps = 1.0e-12;
    if d.abs() < eps {
        // Parallel: must already be within slab
        return o >= mn && o <= mx;
    }
    let inv = 1.0 / d;
    let mut t0 = (mn - o) * inv;
    let mut t1 = (mx - o) * inv;
    if t0 > t1 { core::mem::swap(&mut t0, &mut t1); }
    *tmin = (*tmin).max(t0);
    *tmax = (*tmax).min(t1);
    *tmax >= *tmin
}

impl World {
    #[inline]
    fn terrain_world_bounds(&self, hf: &riftphys_terrain::HeightField) -> (f32, f32, f32, f32) {
        let ox = self.tile_cfg.origin[0];
        let oz = self.tile_cfg.origin[1];

        let nxm1 = hf.dims.x.saturating_sub(1) as f32;
        let nzm1 = hf.dims.y.saturating_sub(1) as f32;

        let max_lx = nxm1 * hf.cell.x;
        let max_lz = nzm1 * hf.cell.y;

        (ox, ox + max_lx, oz, oz + max_lz)
    }

    #[inline]
    fn terrain_sample_ro(&self, wx: f32, wz: f32) -> Option<(f32, Vec3)> {
        let hf = self.terrain.as_ref()?;

        let ox = self.tile_cfg.origin[0];
        let oz = self.tile_cfg.origin[1];

        // local coords
        let lx = wx - ox;
        let lz = wz - oz;

        let nxm1 = hf.dims.x.saturating_sub(1) as f32;
        let nzm1 = hf.dims.y.saturating_sub(1) as f32;

        let max_lx = nxm1 * hf.cell.x;
        let max_lz = nzm1 * hf.cell.y;

        // IMPORTANT: do NOT rely on HeightField's internal edge clamping for query validity.
        if !(lx.is_finite() && lz.is_finite()) { return None; }
        if lx < 0.0 || lz < 0.0 || lx > max_lx || lz > max_lz { return None; }

        let h_world = hf.sample_height(lx, lz) + self.tile_cfg.y_offset;
        let n_world: Vec3 = hf.sample_normal(lx, lz).into(); // normal already world-aligned
        Some((h_world, n_world))
    }

    fn raycast_terrain(&self, origin: Vec3, dir: Vec3, max_dist: f32) -> Option<(f32, Vec3, Vec3)> {
        let hf = self.terrain.as_ref()?;
        let max_d = max_dist.max(0.0);
        if max_d <= 0.0 { return None; }

        // Compute ray interval where (x,z) are inside terrain bounds.
        let (xmin, xmax, zmin, zmax) = self.terrain_world_bounds(hf);

        let mut tmin = 0.0f32;
        let mut tmax = max_d;

        if !slab_range_1d(origin.x, dir.x, xmin, xmax, &mut tmin, &mut tmax) { return None; }
        if !slab_range_1d(origin.z, dir.z, zmin, zmax, &mut tmin, &mut tmax) { return None; }
        if tmax <= tmin { return None; }

        // Special-case near-vertical rays: xz constant => solve y(t)=h(x,z)
        let xz_mag = dir.x.abs().max(dir.z.abs());
        if xz_mag < 1.0e-9 {
            if dir.y.abs() < 1.0e-12 { return None; }
            let (h, n) = self.terrain_sample_ro(origin.x, origin.z)?;
            let t = (h - origin.y) / dir.y;
            if t >= 0.0 && t <= max_d {
                let mut p = origin + dir * t;
                p.y = h;
                return Some((t, p, n));
            }
            return None;
        }

        // Step size: move at most ~half a cell in XZ per step (deterministic).
        let cell = hf.cell.x.min(hf.cell.y).max(1.0e-6);
        let denom = xz_mag.max(1.0e-6);
        let step = (0.5 * cell / denom).clamp(1.0e-4, (tmax - tmin).max(1.0e-4));

        // Start at tmin (first point where XZ is guaranteed in-bounds)
        let mut t0 = tmin;
        let p0 = origin + dir * t0;
        let (h0, n0) = self.terrain_sample_ro(p0.x, p0.z)?;
        let mut f0 = p0.y - h0;

        // If starting under terrain in-range, count as immediate hit.
        if f0 <= 0.0 {
            let mut p = p0;
            p.y = h0;
            return Some((t0, p, n0));
        }

        let mut t = t0;
        while t < tmax - 1.0e-9 {
            let t1 = (t + step).min(tmax);
            let p1 = origin + dir * t1;

            let Some((h1, n1)) = self.terrain_sample_ro(p1.x, p1.z) else {
                t = t1;
                t0 = t1;
                continue;
            };

            let f1 = p1.y - h1;

            // crossing: above -> at/below
            if f0 > 0.0 && f1 <= 0.0 {
                let mut lo = t0;
                let mut hi = t1;
                let mut h_hi = h1;
                let mut n_hi = n1;

                // Fixed-iteration bisection = deterministic.
                for _ in 0..24 {
                    let mid = 0.5 * (lo + hi);
                    let pm = origin + dir * mid;
                    if let Some((hm, nm)) = self.terrain_sample_ro(pm.x, pm.z) {
                        let fm = pm.y - hm;
                        if fm > 0.0 {
                            lo = mid;
                        } else {
                            hi = mid;
                            h_hi = hm;
                            n_hi = nm;
                        }
                    } else {
                        lo = mid; // shouldn’t happen given slabs; still deterministic
                    }
                }

                let mut ph = origin + dir * hi;
                ph.y = h_hi;
                return Some((hi, ph, n_hi));
            }

            // advance
            t = t1;
            t0 = t1;
            f0 = f1;
        }

        None
    }

    fn sweep_capsule_terrain(
        &self,
        from: Vec3,
        to: Vec3,
        radius: f32,
        half_height: f32,
    ) -> Option<(f32, Vec3, Vec3, bool)> {
        let hf = self.terrain.as_ref()?;
        let v = to - from;
        if v.length_squared() < 1.0e-12 { return None; }

        // Segment param t in [0,1]. Intersect XZ against terrain bounds expanded by radius.
        let (mut xmin, mut xmax, mut zmin, mut zmax) = self.terrain_world_bounds(hf);
        let r = radius.abs();
        xmin -= r; xmax += r; zmin -= r; zmax += r;

        let mut tmin = 0.0f32;
        let mut tmax = 1.0f32;

        if !slab_range_1d(from.x, v.x, xmin, xmax, &mut tmin, &mut tmax) { return None; }
        if !slab_range_1d(from.z, v.z, zmin, zmax, &mut tmin, &mut tmax) { return None; }
        if tmax <= tmin { return None; }

        // Helper: penetration function f(t) = capsule_bottom_y - terrain_height (<=0 => hit)
        #[inline]
        fn capsule_bottom_y(c: Vec3, hh: f32, r: f32) -> f32 { c.y - hh.abs() - r }

        // started-overlap check at t=0 (only if sample exists)
        if let Some((h0, n0)) = self.terrain_sample_ro(from.x, from.z) {
            let f0 = capsule_bottom_y(from, half_height, r) - h0;
            if f0 <= 0.0 {
                let base = from + Vec3::new(0.0, -half_height.abs(), 0.0);
                let point = base - n0 * r;
                return Some((0.0, point, n0, true));
            }
        }

        // If XZ motion is basically zero, solve along Y against constant height sample.
        let xz_mag = v.x.abs().max(v.z.abs());
        if xz_mag < 1.0e-9 {
            if v.y.abs() < 1.0e-12 { return None; }
            let (h, n) = self.terrain_sample_ro(from.x, from.z)?;
            // Solve capsule_bottom_y(from + v*t) == h  (linear in t)
            let hh = half_height.abs();
            let t = (h + hh + r - from.y) / v.y;
            if t >= 0.0 && t <= 1.0 {
                let c = from + v * t;
                let base = c + Vec3::new(0.0, -hh, 0.0);
                let point = base - n * r;
                return Some((t, point, n, false));
            }
            return None;
        }

        // Step size: move at most ~half a cell in XZ per step (deterministic).
        let cell = hf.cell.x.min(hf.cell.y).max(1.0e-6);
        let denom = xz_mag.max(1.0e-6);
        let step = (0.5 * cell / denom).clamp(1.0 / 8192.0, (tmax - tmin).max(1.0 / 8192.0));

        let mut t0 = tmin;
        let c0 = from + v * t0;
        let (h0, _n0) = self.terrain_sample_ro(c0.x, c0.z)?;
        let mut f0 = capsule_bottom_y(c0, half_height, r) - h0;

        let mut t = t0;
        while t < tmax - 1.0e-9 {
            let t1 = (t + step).min(tmax);
            let c1 = from + v * t1;

            let Some((h1, n1)) = self.terrain_sample_ro(c1.x, c1.z) else {
                t = t1;
                t0 = t1;
                continue;
            };

            let f1 = capsule_bottom_y(c1, half_height, r) - h1;

            if f0 > 0.0 && f1 <= 0.0 {
                let mut lo = t0;
                let mut hi = t1;
                let mut h_hi = h1;
                let mut n_hi = n1;

                for _ in 0..24 {
                    let mid = 0.5 * (lo + hi);
                    let cm = from + v * mid;
                    if let Some((hm, nm)) = self.terrain_sample_ro(cm.x, cm.z) {
                        let fm = capsule_bottom_y(cm, half_height, r) - hm;
                        if fm > 0.0 {
                            lo = mid;
                        } else {
                            hi = mid;
                            h_hi = hm;
                            n_hi = nm;
                        }
                    } else {
                        lo = mid;
                    }
                }

                let c = from + v * hi;
                let hh = half_height.abs();
                let base = c + Vec3::new(0.0, -hh, 0.0);
                let point = base - n_hi * r;
                let _ = h_hi; // height already incorporated by the root
                return Some((hi, point, n_hi, false));
            }

            t = t1;
            t0 = t1;
            f0 = f1;
        }

        None
    }

    pub fn raycast(
        &self,
        origin: Vec3,
        dir: Vec3,
        max_dist: f32,
        ignore: Option<riftphys_core::BodyId>,
    ) -> Option<RayHit> {
        let max_d = max_dist.max(0.0);
        if max_d <= 0.0 { return None; }

        let len2 = dir.length_squared();
        if len2 < 1.0e-12 { return None; }

        let mut best: Option<(f32, usize, Vec3, Vec3)> = None; // (t, collider_idx, point, normal)

        for (ci, c) in self.colliders.iter().enumerate() {
            if let Some(ig) = ignore {
                if c.body == ig { continue; }
            }
            if let Some((t, p, n)) = ray_aabb(origin, dir, &c.aabb, max_d) {
                match best {
                    None => best = Some((t, ci, p, n)),
                    Some((bt, bi, _, _)) => {
                        if (t < bt - 1.0e-9) || ((t - bt).abs() <= 1.0e-9 && ci < bi) {
                            best = Some((t, ci, p, n));
                        }
                    }
                }
            }
        }

        // Terrain candidate (treated as collider index = usize::MAX so ties go to real colliders).
        if let Some((tt, tp, tn)) = self.raycast_terrain(origin, dir, max_d) {
            best = match best {
                None => Some((tt, usize::MAX, tp, tn)),
                Some((bt, bi, bp, bn)) => {
                    if tt < bt - 1.0e-9 {
                        Some((tt, usize::MAX, tp, tn))
                    } else {
                        Some((bt, bi, bp, bn))
                    }
                }
            };
        }

        let (t, ci, point, normal) = best?;
        let body = if ci == usize::MAX { TERRAIN_HIT_BODY } else { self.colliders[ci].body };

        Some(RayHit {
            body,
            toi: (t / max_d).clamp(0.0, 1.0),
            point,
            normal,
        })
    }

    pub fn sweep_capsule(
        &self,
        from: Vec3,
        to: Vec3,
        radius: f32,
        half_height: f32,
        ignore: Option<riftphys_core::BodyId>,
    ) -> Option<SweepHit> {
        let v = to - from;
        if v.length_squared() < 1.0e-12 { return None; }

        let dt = 1.0_f32; // toi in [0,1] along [from -> to]
        let tip0 = from + Vec3::new(0.0, half_height, 0.0);
        let base0 = from + Vec3::new(0.0, -half_height, 0.0);

        let eps = 1.0e-9;

        // best: (toi, collider_idx, sample_kind, point, normal, started_overlapping)
        let mut best: Option<(f32, usize, u8, Vec3, Vec3, bool)> = None;

        for (ci, c) in self.colliders.iter().enumerate() {
            if let Some(ig) = ignore {
                if c.body == ig { continue; }
            }

            // overlap at t=0 (capsule vs AABB) -> started_overlapping
            let (p_seg, p_box) = closest_points_segment_aabb(tip0, base0, c.aabb.min, c.aabb.max);
            let d = p_seg - p_box;
            let dist = d.length();

            if dist < radius {
                let n = if dist > 1.0e-6 { d / dist } else { Vec3::new(0.0, 1.0, 0.0) };
                let point = p_seg - n * radius;

                let cand = (0.0, ci, 0u8, point, n, true);
                best = match best {
                    None => Some(cand),
                    Some((bt, bi, bk, _, _, _)) => {
                        if 0.0 < bt - eps
                            || ((0.0 - bt).abs() <= eps && (ci < bi || (ci == bi && 0u8 < bk)))
                        { Some(cand) } else { best }
                    }
                };
                continue;
            }

            // sweep using riftphys-query (tip+base spheres vs this one AABB)
            let Some(hit) = query::sweep_two_spheres_aabbs(
                tip0, v, radius,
                base0, v, radius,
                std::slice::from_ref(&c.aabb),
                dt,
            ) else { continue; };

            let t = hit.toi.clamp(0.0, 1.0);
            let n = hit.normal;

            let center0 = if hit.sample_kind == 0 { tip0 } else { base0 };
            let center_at_hit = center0 + v * t;
            let point = center_at_hit - n * radius;

            let cand = (t, ci, hit.sample_kind, point, n, false);

            best = match best {
                None => Some(cand),
                Some((bt, bi, bk, _, _, _)) => {
                    if (t < bt - eps)
                        || ((t - bt).abs() <= eps && (ci < bi || (ci == bi && hit.sample_kind < bk)))
                    { Some(cand) } else { best }
                }
            };
        }

        // Terrain candidate (collider index = usize::MAX so ties go to real colliders).
        if let Some((tt, tp, tn, started)) = self.sweep_capsule_terrain(from, to, radius, half_height) {
            let cand = (tt, usize::MAX, 0u8, tp, tn, started);
            best = match best {
                None => Some(cand),
                Some((bt, bi, bk, bp, bn, bs)) => {
                    if tt < bt - eps {
                        Some(cand)
                    } else if (tt - bt).abs() <= eps {
                        // tie: prefer real colliders (smaller index than usize::MAX)
                        Some((bt, bi, bk, bp, bn, bs))
                    } else {
                        Some((bt, bi, bk, bp, bn, bs))
                    }
                }
            };
        }

        let (toi, ci, _sk, point, normal, started) = best?;
        let body = if ci == usize::MAX { TERRAIN_HIT_BODY } else { self.colliders[ci].body };

        Some(SweepHit {
            body,
            toi,
            started_overlapping: started,
            point,
            normal,
        })
    }
    pub fn sweep_blade_against_boxes(
        &self,
        tip_p0: Vec3,
        tip_v: Vec3,
        tip_r: f32,
        mid_p0: Vec3,
        mid_v: Vec3,
        mid_r: f32,
        dt: f32,
    ) -> Option<(usize, f32, Vec3)> {
        let mut boxes: Vec<Aabb> = Vec::new();
        for c in &self.colliders {
            if let Shape::Box { .. } = c.shape {
                boxes.push(c.aabb);
            }
        }
        if boxes.is_empty() {
            return None;
        }

        let hit = query::sweep_two_spheres_aabbs(
            tip_p0, tip_v, tip_r,
            mid_p0, mid_v, mid_r,
            &boxes,
            dt,
        )?;

        Some((hit.target_index, hit.toi, hit.normal))
    }
}
