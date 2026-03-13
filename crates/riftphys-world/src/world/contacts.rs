// use super::geom::{clamp_vec3, closest_points_segment_aabb};
// use super::world::{Contact, World};
//
// use riftphys_core::Vec3;
// use riftphys_geom::Shape;
//
// impl World {
//     pub(super) fn contact_box_box(&self, ci: usize, cj: usize) -> Option<Contact> {
//         let a = &self.colliders[ci];
//         let b = &self.colliders[cj];
//         match (a.shape, b.shape) {
//             (Shape::Box { .. }, Shape::Box { .. }) => {}
//             _ => return None,
//         }
//         let aa = a.aabb; let bb = b.aabb;
//         if !aa.overlaps(&bb) { return None; }
//         let ca = (aa.min + aa.max) * 0.5;
//         let cb = (bb.min + bb.max) * 0.5;
//         let px = (aa.max.x - bb.min.x).min(bb.max.x - aa.min.x);
//         let py = (aa.max.y - bb.min.y).min(bb.max.y - aa.min.y);
//         let pz = (aa.max.z - bb.min.z).min(bb.max.z - aa.min.z);
//         let (mut normal, depth) = if px <= py && px <= pz {
//             let dir = if cb.x > ca.x { 1.0 } else { -1.0 }; (Vec3::new(dir, 0.0, 0.0), px)
//         } else if py <= pz {
//             let dir = if cb.y > ca.y { 1.0 } else { -1.0 }; (Vec3::new(0.0, dir, 0.0), py)
//         } else {
//             let dir = if cb.z > ca.z { 1.0 } else { -1.0 }; (Vec3::new(0.0, 0.0, dir), pz)
//         };
//         if depth <= 0.0 { return None; }
//         let n_len = normal.length();
//         if n_len == 0.0 { return None; }
//         normal /= n_len;
//         Some(Contact { a_collider: ci, b_collider: cj, normal, depth })
//     }
//
//     pub(super) fn contact_sphere_sphere(&self, ci: usize, cj: usize) -> Option<Contact> {
//         let a = &self.colliders[ci];
//         let b = &self.colliders[cj];
//         let (ra, rb) = match (a.shape, b.shape) {
//             (Shape::Sphere { r: r1 }, Shape::Sphere { r: r2 }) => (r1, r2),
//             _ => return None,
//         };
//         let pa = self.bodies.pose(a.body.0).pos;
//         let pb = self.bodies.pose(b.body.0).pos;
//         let d = pb - pa;
//         let dist2 = d.length_squared();
//         let rsum = ra + rb;
//         if dist2 >= rsum * rsum { return None; }
//         let dist = dist2.sqrt();
//         let normal = if dist > 1.0e-6 { d / dist } else { Vec3::new(1.0, 0.0, 0.0) };
//         let depth  = rsum - dist;
//         Some(Contact { a_collider: ci, b_collider: cj, normal, depth })
//     }
//
//     pub(super) fn contact_sphere_box(&self, ci: usize, cj: usize) -> Option<Contact> {
//         let (si, bi) = match (self.colliders[ci].shape, self.colliders[cj].shape) {
//             (Shape::Sphere { .. }, Shape::Box { .. }) => (ci, cj),
//             (Shape::Box { .. }, Shape::Sphere { .. }) => (cj, ci),
//             _ => return None,
//         };
//         let s = &self.colliders[si];
//         let b = &self.colliders[bi];
//         let r = match s.shape { Shape::Sphere { r } => r, _ => unreachable!() };
//         let ps = self.bodies.pose(s.body.0).pos;
//         let bb = b.aabb;
//         let q = clamp_vec3(ps, bb.min, bb.max);
//         let mut n = ps - q; // box -> sphere
//         let dist = n.length();
//         if dist >= r { return None; }
//         if dist > 1.0e-6 { n /= dist; } else { n = Vec3::new(0.0, 1.0, 0.0); }
//         let depth = r - dist;
//         let normal = -n; // always A(sphere) -> B(box)
//         Some(Contact { a_collider: si, b_collider: bi, normal, depth })
//     }
//
//     pub(super) fn contact_capsule_box(&self, ci: usize, cj: usize) -> Option<Contact> {
//         let (cap_i, box_i) = match (self.colliders[ci].shape, self.colliders[cj].shape) {
//             (Shape::Capsule { .. }, Shape::Box { .. }) => (ci, cj),
//             (Shape::Box { .. }, Shape::Capsule { .. }) => (cj, ci),
//             _ => return None,
//         };
//         let cap = &self.colliders[cap_i];
//         let bx = &self.colliders[box_i];
//         let (r, hh) = match cap.shape { Shape::Capsule { r, hh } => (r, hh), _ => unreachable!() };
//         let pose = self.bodies.pose(cap.body.0);
//         let pa = pose.pos + (pose.rot * Vec3::new(0.0,  hh, 0.0));
//         let pb = pose.pos + (pose.rot * Vec3::new(0.0, -hh, 0.0));
//         let bb = bx.aabb;
//         let (p_seg, p_box) = closest_points_segment_aabb(pa, pb, bb.min, bb.max);
//         let mut n = p_seg - p_box; // box -> capsule axis
//         let dist = n.length();
//         if dist >= r { return None; }
//         if dist > 1.0e-6 { n /= dist; } else { n = Vec3::new(0.0, 1.0, 0.0); }
//         let depth = r - dist;
//         let normal = -n; // always A(capsule) -> B(box)
//         Some(Contact { a_collider: cap_i, b_collider: box_i, normal, depth })
//     }
// }
