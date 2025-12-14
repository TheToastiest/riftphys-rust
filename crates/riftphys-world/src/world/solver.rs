use super::geom::orthonormal_basis;
use super::world::{Contact, WarmImp, World};
use riftphys_materials::materials as mats;
use riftphys_viz::LedgerEvent;

use std::collections::BTreeMap;

impl World {
    pub(super) fn solve_contacts(&mut self, contacts: &[Contact]) {
        let iterations = 12;
        let slop = 0.010;
        let beta = 0.10;

        // Build warmstart vector aligned with `contacts` order
        let mut warms: Vec<WarmImp> = Vec::with_capacity(contacts.len());
        for c in contacts {
            let key = if c.a_collider <= c.b_collider {
                (c.a_collider as u32, c.b_collider as u32)
            } else {
                (c.b_collider as u32, c.a_collider as u32)
            };
            warms.push(*self.warm_cache.get(&key).unwrap_or(&WarmImp::default()));
        }

        for v in &mut self.last_normal_impulse { *v = 0.0; }

        let mut next_warms: Vec<WarmImp> = vec![WarmImp::default(); contacts.len()];

        for it in 0..iterations {
            for (idx, c) in contacts.iter().enumerate() {
                let ai = self.colliders[c.a_collider].body.0;
                let bi = self.colliders[c.b_collider].body.0;
                if ai == bi { continue; }

                let inv_a = self.bodies.inv_mass_of(ai);
                let inv_b = self.bodies.inv_mass_of(bi);
                if inv_a + inv_b == 0.0 { continue; }

                // Warmstart only on first iteration
                if it == 0 {
                    let w = warms[idx];
                    if w.jn != 0.0 || w.jt1 != 0.0 || w.jt2 != 0.0 {
                        let n = c.normal;
                        let (t1, t2) = orthonormal_basis(n);
                        let imp = n * w.jn + t1 * w.jt1 + t2 * w.jt2;
                        self.bodies.apply_impulse(ai, -imp);
                        self.bodies.apply_impulse(bi,  imp);
                    }
                }

                // Effective pair properties (order-independent)
                let ma = self.colliders[c.a_collider].material;
                let mb = self.colliders[c.b_collider].material;
                let pair = mats::pair_props(ma.id, mb.id);

                let restitution = pair.restitution;

                let va = self.bodies.vel(ai);
                let vb = self.bodies.vel(bi);
                let n  = c.normal;
                let rel_v_n = (vb.lin - va.lin).dot(n);

                // Normal impulse
                let mut jn = 0.0;
                if rel_v_n < 0.0 {
                    jn = -(1.0 + restitution) * rel_v_n / (inv_a + inv_b);
                    let imp_n = n * jn;
                    self.bodies.apply_impulse(ai, -imp_n);
                    self.bodies.apply_impulse(bi,  imp_n);

                    self.last_normal_impulse[ai as usize] += jn.max(0.0);
                    self.last_normal_impulse[bi as usize] += jn.max(0.0);

                    self.record(LedgerEvent::ImpulseN { a: ai, b: bi, jn });
                }
                next_warms[idx].jn = jn;

                // Positional correction
                let corr = (c.depth - slop).max(0.0) * beta;
                if corr > 0.0 {
                    let denom = inv_a + inv_b;
                    let corr_vec = n * (corr / denom);
                    self.bodies.apply_position_delta(ai, -corr_vec * inv_a);
                    self.bodies.apply_position_delta(bi,  corr_vec * inv_b);
                    self.record(LedgerEvent::PosCorr { a: ai, b: bi, corr });
                }

                // Friction
                if jn > 0.0 || c.depth > slop {
                    let va2 = self.bodies.vel(ai);
                    let vb2 = self.bodies.vel(bi);
                    let vrel = vb2.lin - va2.lin;
                    let v_n = n * vrel.dot(n);
                    let v_t = vrel - v_n;

                    let (t1, t2) = orthonormal_basis(n);
                    let vt1 = v_t.dot(t1);
                    let vt2 = v_t.dot(t2);

                    let denom = inv_a + inv_b;
                    if denom > 0.0 {
                        let jt1_des = -vt1 / denom;
                        let jt2_des = -vt2 / denom;
                        let jt_des_len = (jt1_des * jt1_des + jt2_des * jt2_des).sqrt();

                        let vt_mag = v_t.length();
                        let mu_k_eff = mats::mu_dynamic(&pair, vt_mag);
                        let jt_max_static = pair.mu_s * jn;

                        let (jt1, jt2) = if jt_des_len <= jt_max_static || jn == 0.0 {
                            (jt1_des, jt2_des)
                        } else {
                            let jt_max_kin = mu_k_eff * jn;
                            let scale = if jt_des_len > 1.0e-9 { jt_max_kin / jt_des_len } else { 0.0 };
                            (jt1_des * scale, jt2_des * scale)
                        };

                        next_warms[idx].jt1 = jt1;
                        next_warms[idx].jt2 = jt2;

                        let jt_vec = t1 * jt1 + t2 * jt2;
                        self.bodies.apply_impulse(ai, -jt_vec);
                        self.bodies.apply_impulse(bi,  jt_vec);
                        self.record(LedgerEvent::ImpulseT { a: ai, b: bi, jt1, jt2 });
                    }
                }
            }
        }

        // write warmstart cache for next tick
        let mut new_cache: BTreeMap<(u32,u32), WarmImp> = BTreeMap::new();
        for (idx, c) in contacts.iter().enumerate() {
            let key = if c.a_collider <= c.b_collider {
                (c.a_collider as u32, c.b_collider as u32)
            } else {
                (c.b_collider as u32, c.a_collider as u32)
            };
            new_cache.insert(key, next_warms[idx]);
        }
        self.warm_cache = new_cache;
    }
}
