use anyhow::{Result, anyhow};
use glam::{Mat4, Vec3, Quat};
use glam::Affine3A;

use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use crate::RigData;
use riftphys_core::{Isometry, vec3};
use riftphys_geom::Shape;
use riftphys_world::World;
use riftphys_articulation::*;

/* ─────────────────────────  Physics rig schema ───────────────────────── */

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LinkDef {
    pub name: String,
    pub mass: f32,
    pub shape: ColliderShape,          // capsule/box/sphere
    pub pose_ws: [f32; 7],             // (px,py,pz,qx,qy,qz,qw) initial
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag="type")]
pub enum ColliderShape {
    Capsule { r: f32, hh: f32 },
    Box     { hx: f32, hy: f32, hz: f32 },
    Sphere  { r: f32 },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag="type")]
pub enum JointDef {
    Ball {
        name: String, parent: String, child: String,
        frame_a: [f32; 7], frame_b: [f32; 7],            // (r,q) with r=xyz offset in local joint frame
        limits: Option<BallLimits>,
        drives: Option<Drive6>,                           // optional Rx,Ry,Rz drives
    },
    Hinge {
        name: String, parent: String, child: String, hinge_axis: usize,
        frame_a: [f32; 7], frame_b: [f32; 7],
        limit: Option<[f32;2]>,
        drive: Option<Drive1>,
    },
    D6 {
        name: String, parent: String, child: String,
        frame_a: [f32; 7], frame_b: [f32; 7],
        t_enabled: [bool;3], r_enabled: [bool;3],
        t_limit: Option<[[f32;2];3]>, r_limit: Option<[[f32;2];3]>,
        t_drive: Option<Drive3>,      r_drive: Option<Drive3>,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct BallLimits { pub swing_y: [f32;2], pub swing_z: [f32;2], pub twist: [f32;2] }
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Drive1 { pub target: f32, pub kp: f32, pub kd: f32 }
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Drive3 { pub target: [f32;3], pub kp: [f32;3], pub kd: [f32;3] }
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Drive6 { pub rx: Drive1, pub ry: Drive1, pub rz: Drive1 }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicsRig {
    pub links: Vec<LinkDef>,
    pub joints: Vec<JointDef>,
}

/* ─────────────────────────  Builder (humanoid heuristic) ───────────────────────── */

/// Build a coarse physics rig from a `RigData` skeleton.
/// Uses joint positions from the GLB to estimate segment lengths.
/// Assumes a human-ish naming. Deterministic.
pub fn humanoid_from_rig(rig: &RigData) -> Result<PhysicsRig> {
    fn canon(s: &str) -> String {
        s.to_ascii_lowercase()
            .chars()
            .filter(|c| c.is_ascii_alphanumeric() || *c == '_' || *c == '.' || *c == ':')
            .collect()
    }

    fn find_idx<'a>(names: &'a [String], side: Option<char>, tokens: &[&str]) -> Option<usize> {
        for (i, n) in names.iter().enumerate() {
            let c = canon(n);
            let side_ok = match side {
                Some('l') => c.contains("_l") || c.ends_with('l') || c.contains(".l") || c.contains("left"),
                Some('r') => c.contains("_r") || c.ends_with('r') || c.contains(".r") || c.contains("right"),
                _ => true,
            };
            if !side_ok { continue; }
            if tokens.iter().all(|t| c.contains(&t.to_ascii_lowercase())) { return Some(i); }
        }
        None
    }
    // Re-open the GLB to recover joint transforms for positions/directions
    let gltf = gltf::Gltf::open(&rig.source_file)?;
    let skins: Vec<gltf::Skin> = gltf.document.skins().collect();
    let skin = skins.get(rig.skin_index)
        .ok_or_else(|| anyhow!("skin {} not found in {}", rig.skin_index, rig.source_file))?;
    let nodes: Vec<gltf::Node> = skin.joints().collect();
    if nodes.is_empty() { return Err(anyhow!("no joints")); }

    // Global bind matrices from node hierarchy (deterministic order: index)
    let mut parent_of = vec![None::<usize>; gltf.document.nodes().count()];
    for n in gltf.document.nodes() {
        for ch in n.children() { parent_of[ch.index()] = Some(n.index()); }
    }
    let mut global: Vec<Mat4> = vec![Mat4::IDENTITY; nodes.len()];
    for (i, n) in nodes.iter().enumerate() {
        // accumulate up to root
        let mut m = Mat4::from_cols_array_2d(&n.transform().matrix());
        let mut cur = parent_of[n.index()];
        while let Some(pi) = cur {
            let pn = gltf.document.nodes().nth(pi).unwrap();
            m = Mat4::from_cols_array_2d(&pn.transform().matrix()) * m;
            cur = parent_of[pi];
        }
        global[i] = m;
    }

    // Helper: get joint index by (case-insensitive) name
    let mut name_to_idx: HashMap<String, usize> = HashMap::new();
    for (i, n) in nodes.iter().enumerate() {
        let name = n.name().unwrap_or("").to_ascii_lowercase();
        name_to_idx.insert(name, i);
    }
    let find = |names: &[&str]| -> Option<usize> {
        for &nm in names { if let Some(i) = name_to_idx.get(&nm.to_ascii_lowercase()) { return Some(*i); } }
        None
    };
    let names: Vec<String> = nodes.iter()
        .map(|n| n.name().unwrap_or("").to_string())
        .collect();

    // pelvis first
    let pelvis = names.iter().position(|n| {
        let c = canon(n);
        c.contains("pelvis") || c.contains("hips")
    }).unwrap_or_else(|| {
        // fallback: the first child of root or just 0
        1.min(names.len().saturating_sub(1))
    });

    // left chain
    let l_thigh = find_idx(&names, Some('l'), &["thigh"]).or_else(|| find_idx(&names, Some('l'), &["upleg"]))
        .ok_or_else(|| anyhow::anyhow!("left thigh not found"))?;
    let l_shank = find_idx(&names, Some('l'), &["calf"]).or_else(|| find_idx(&names, Some('l'), &["shin"]))
        .or_else(|| find_idx(&names, Some('l'), &["leg"]))
        .ok_or_else(|| anyhow::anyhow!("left shank not found"))?;
    let l_foot  = find_idx(&names, Some('l'), &["foot"]).or_else(|| find_idx(&names, Some('l'), &["ball"]))
        .ok_or_else(|| anyhow::anyhow!("left foot not found"))?;

    // right chain
    let r_thigh = find_idx(&names, Some('r'), &["thigh"]).or_else(|| find_idx(&names, Some('r'), &["upleg"]))
        .ok_or_else(|| anyhow::anyhow!("right thigh not found"))?;
    let r_shank = find_idx(&names, Some('r'), &["calf"]).or_else(|| find_idx(&names, Some('r'), &["shin"]))
        .or_else(|| find_idx(&names, Some('r'), &["leg"]))
        .ok_or_else(|| anyhow::anyhow!("right shank not found"))?;
    let r_foot  = find_idx(&names, Some('r'), &["foot"]).or_else(|| find_idx(&names, Some('r'), &["ball"]))
        .ok_or_else(|| anyhow::anyhow!("right foot not found"))?;
    // Positions from global matrices
    let p = |i: usize| -> Vec3 { let m = global[i]; Vec3::new(m.w_axis.x, m.w_axis.y, m.w_axis.z) };

    // Segment lengths (fallback constants if identical)
    let len = |a: usize, b: usize| -> f32 { (p(b) - p(a)).length().max(0.15) };

    // Link defs (capsule along local Y)
    let pelvis_pos = p(pelvis);
    let lt_len = len(l_thigh, l_shank);
    let ls_len = len(l_shank, l_foot);
    let rt_len = len(r_thigh, r_shank);
    let rs_len = len(r_shank, r_foot);

    let links = vec![
        LinkDef { name: "pelvis".into(), mass: 12.0,
            shape: ColliderShape::Capsule { r: 0.15, hh: 0.35 }, pose_ws: pack_iso(pelvis_pos, Quat::IDENTITY) },
        LinkDef { name: "l_thigh".into(), mass: 7.0,
            shape: ColliderShape::Capsule { r: 0.09, hh: lt_len * 0.5 },
            pose_ws: pack_iso(p(l_thigh), Quat::IDENTITY) },
        LinkDef { name: "l_shank".into(), mass: 4.5,
            shape: ColliderShape::Capsule { r: 0.08, hh: ls_len * 0.5 },
            pose_ws: pack_iso(p(l_shank), Quat::IDENTITY) },
        LinkDef { name: "l_foot".into(), mass: 2.0,
            shape: ColliderShape::Capsule { r: 0.05, hh: 0.12 }, // rocker
            pose_ws: pack_iso(p(l_foot), Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2)) },
        LinkDef { name: "r_thigh".into(), mass: 7.0,
            shape: ColliderShape::Capsule { r: 0.09, hh: rt_len * 0.5 },
            pose_ws: pack_iso(p(r_thigh), Quat::IDENTITY) },
        LinkDef { name: "r_shank".into(), mass: 4.5,
            shape: ColliderShape::Capsule { r: 0.08, hh: rs_len * 0.5 },
            pose_ws: pack_iso(p(r_shank), Quat::IDENTITY) },
        LinkDef { name: "r_foot".into(), mass: 2.0,
            shape: ColliderShape::Capsule { r: 0.05, hh: 0.12 },
            pose_ws: pack_iso(p(r_foot), Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2)) },
    ];

    // Joint frames: local offsets kept zero for simplicity (anchors at COM); tune later if desired
    let ziso = pack_iso(Vec3::ZERO, Quat::IDENTITY);
    let joints = vec![
        JointDef::Ball { name:"l_hip".into(), parent:"pelvis".into(), child:"l_thigh".into(),
            frame_a: ziso, frame_b: ziso,
            limits: Some(BallLimits { swing_y:[-0.6,0.8], swing_z:[-0.4,0.6], twist:[-0.8,0.8] }),
            drives: None },
        JointDef::Hinge { name:"l_knee".into(), parent:"l_thigh".into(), child:"l_shank".into(), hinge_axis: 2, // Rz
            frame_a: ziso, frame_b: ziso, limit: Some([0.0, 2.2]), drive: None },
        JointDef::Ball { name:"l_ankle".into(), parent:"l_shank".into(), child:"l_foot".into(),
            frame_a: ziso, frame_b: ziso,
            limits: Some(BallLimits { swing_y:[-0.6,0.6], swing_z:[-0.4,0.4], twist:[-0.5,0.5] }),
            drives: None },

        JointDef::Ball { name:"r_hip".into(), parent:"pelvis".into(), child:"r_thigh".into(),
            frame_a: ziso, frame_b: ziso,
            limits: Some(BallLimits { swing_y:[-0.6,0.8], swing_z:[-0.4,0.6], twist:[-0.8,0.8] }),
            drives: None },
        JointDef::Hinge { name:"r_knee".into(), parent:"r_thigh".into(), child:"r_shank".into(), hinge_axis: 2,
            frame_a: ziso, frame_b: ziso, limit: Some([0.0, 2.2]), drive: None },
        JointDef::Ball { name:"r_ankle".into(), parent:"r_shank".into(), child:"r_foot".into(),
            frame_a: ziso, frame_b: ziso,
            limits: Some(BallLimits { swing_y:[-0.6,0.6], swing_z:[-0.4,0.4], twist:[-0.5,0.5] }),
            drives: None },
    ];

    Ok(PhysicsRig { links, joints })
}

fn pack_iso(p: Vec3, q: Quat) -> [f32;7] { [p.x,p.y,p.z, q.x,q.y,q.z,q.w] }
fn unpack_iso(a: [f32;7]) -> Isometry {
    Isometry { pos: vec3(a[0],a[1],a[2]), rot: Quat::from_xyzw(a[3],a[4],a[5],a[6]).normalize() }
}

/* ─────────────────────────  Loader: spawn into World ───────────────────────── */

pub struct RigMap {
    pub body: HashMap<String, riftphys_core::BodyId>,
    pub joint: HashMap<String, riftphys_core::JointId>,
}
impl RigMap {
    pub fn body(&self, name: &str) -> riftphys_core::BodyId { *self.body.get(name).expect("body name") }
    pub fn joint(&self, name: &str) -> riftphys_core::JointId { *self.joint.get(name).expect("joint name") }
}

/// Create bodies, colliders, and joints in `world`. Deterministic creation order (sorted by name).
pub fn load_into_world(world: &mut World, rig: &PhysicsRig) -> Result<RigMap> {
    use riftphys_geom::{MassProps, Material};

    let mut body_ids: HashMap<String, riftphys_core::BodyId> = HashMap::new();
    // sort links by name for stable ids
    let mut links = rig.links.clone();
    links.sort_by(|a,b| a.name.cmp(&b.name));

    let mut mat_dyn = Material::default();
    mat_dyn.restitution = 0.0; mat_dyn.mu_s = 1.2; mat_dyn.mu_k = 1.0;

    for l in &links {
        let iso = unpack_iso(l.pose_ws);
        let mass = match l.shape {
            ColliderShape::Capsule { r, hh } => MassProps::from_capsule(r, hh, l.mass),
            ColliderShape::Box { hx, hy, hz } => MassProps::from_box(vec3(hx,hy,hz), l.mass),
            ColliderShape::Sphere { r }       => MassProps::from_sphere(r, l.mass),
        };
        let id = world.add_body(iso, riftphys_core::Velocity::default(), mass, true);
        match l.shape {
            ColliderShape::Capsule { r, hh } => world.add_collider(id, Shape::Capsule { r, hh }, mat_dyn),
            ColliderShape::Box { hx, hy, hz } => world.add_collider(id, Shape::Box { hx, hy, hz }, mat_dyn),
            ColliderShape::Sphere { r }       => world.add_collider(id, Shape::Sphere { r }, mat_dyn),
        };
        body_ids.insert(l.name.clone(), id);
    }

    // joints (also sort by name)
    let mut joints_v = rig.joints.clone();
    joints_v.sort_by(|a,b| name_of(a).cmp(name_of(b)));
    let mut joint_ids: HashMap<String, riftphys_core::JointId> = HashMap::new();

    for j in &joints_v {
        match j {
            JointDef::Ball { name, parent, child, frame_a, frame_b, limits, drives: _ } => {
                let a = *body_ids.get(parent).ok_or_else(|| anyhow!("missing body {}", parent))?;
                let b = *body_ids.get(child).ok_or_else(|| anyhow!("missing body {}", child))?;
                let fa = unpack_iso(*frame_a); let fb = unpack_iso(*frame_b);
                let jid = world.add_ball_joint(a,b,fa,fb);
                if let Some(lm) = limits {
                    // lock with D6 limit rows by converting the ball to D6 (optional later)
                    // For now we rely on ball preset (no explicit per-axis limits in this minimal example).
                    let _ = lm;
                }
                joint_ids.insert(name.clone(), jid);
            }
            JointDef::Hinge { name, parent, child, hinge_axis, frame_a, frame_b, limit:_, drive:_ } => {
                let a = *body_ids.get(parent).ok_or_else(|| anyhow!("missing body {}", parent))?;
                let b = *body_ids.get(child).ok_or_else(|| anyhow!("missing body {}", child))?;
                let fa = unpack_iso(*frame_a); let fb = unpack_iso(*frame_b);
                let jid = world.add_hinge_joint(a,b,fa,fb,*hinge_axis);
                joint_ids.insert(name.clone(), jid);
            }
            JointDef::D6 { name, parent, child, frame_a, frame_b, t_enabled, r_enabled, .. } => {
                let a = *body_ids.get(parent).ok_or_else(|| anyhow!("missing body {}", parent))?;
                let b = *body_ids.get(child).ok_or_else(|| anyhow!("missing body {}", child))?;
                let fa = unpack_iso(*frame_a); let fb = unpack_iso(*frame_b);
                let mut d6 = D6Joint {
                    a, b, fa, fb,
                    t: [Default::default(),Default::default(),Default::default()],
                    r: [Default::default(),Default::default(),Default::default()],
                };
                for i in 0..3 { d6.t[i].enabled = t_enabled[i]; d6.r[i].enabled = r_enabled[i]; }
                let jid = world.add_d6_joint(d6);
                joint_ids.insert(name.clone(), jid);
            }
        }
    }

    Ok(RigMap { body: body_ids, joint: joint_ids })
}

fn name_of(j: &JointDef) -> &String {
    match j {
        JointDef::Ball   { name, .. } => name,
        JointDef::Hinge  { name, .. } => name,
        JointDef::D6     { name, .. } => name,
    }
}
