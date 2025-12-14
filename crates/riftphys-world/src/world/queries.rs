// crates/riftphys-world/src/world/queries.rs

use super::geom::{closest_points_segment_aabb, ray_aabb};
use super::world::{RayHit, SweepHit, World};

use riftphys_core::Vec3;
use riftphys_geom::{Aabb, Shape};
use riftphys_query as query;

impl World {
    pub fn raycast(
        &self,
        origin: Vec3,
        dir: Vec3,
        max_dist: f32,
        ignore: Option<riftphys_core::BodyId>,
    ) -> Option<RayHit> {
        let max_d = max_dist.max(0.0);
        if max_d <= 0.0 {
            return None;
        }

        let len2 = dir.length_squared();
        if len2 < 1.0e-12 {
            return None;
        }

        let mut best: Option<(f32, usize, Vec3, Vec3)> = None; // (t, collider_idx, point, normal)

        for (ci, c) in self.colliders.iter().enumerate() {
            if let Some(ig) = ignore {
                if c.body == ig {
                    continue;
                }
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

        let (t, ci, point, normal) = best?;
        Some(RayHit {
            body: self.colliders[ci].body,
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
        if v.length_squared() < 1.0e-12 {
            return None;
        }

        let dt = 1.0_f32; // toi in [0,1] along [from -> to]
        let tip0 = from + Vec3::new(0.0, half_height, 0.0);
        let base0 = from + Vec3::new(0.0, -half_height, 0.0);

        let eps = 1.0e-9;

        // best: (toi, collider_idx, sample_kind, point, normal, started_overlapping)
        let mut best: Option<(f32, usize, u8, Vec3, Vec3, bool)> = None;

        for (ci, c) in self.colliders.iter().enumerate() {
            if let Some(ig) = ignore {
                if c.body == ig {
                    continue;
                }
            }

            // overlap at t=0 (capsule vs AABB) -> started_overlapping
            let (p_seg, p_box) = closest_points_segment_aabb(tip0, base0, c.aabb.min, c.aabb.max);
            let d = p_seg - p_box;
            let dist = d.length();

            if dist < radius {
                let n = if dist > 1.0e-6 {
                    d / dist
                } else {
                    Vec3::new(0.0, 1.0, 0.0)
                };
                let point = p_seg - n * radius;

                let cand = (0.0, ci, 0u8, point, n, true);
                best = match best {
                    None => Some(cand),
                    Some((bt, bi, bk, _, _, _)) => {
                        if 0.0 < bt - eps
                            || ((0.0 - bt).abs() <= eps && (ci < bi || (ci == bi && 0u8 < bk)))
                        {
                            Some(cand)
                        } else {
                            best
                        }
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
            ) else {
                continue; // IMPORTANT: don't early-return None just because this collider didn't hit
            };

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
                        || ((t - bt).abs() <= eps
                        && (ci < bi || (ci == bi && hit.sample_kind < bk)))
                    {
                        Some(cand)
                    } else {
                        best
                    }
                }
            };
        }

        let (toi, ci, _sk, point, normal, started) = best?;
        Some(SweepHit {
            body: self.colliders[ci].body,
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
