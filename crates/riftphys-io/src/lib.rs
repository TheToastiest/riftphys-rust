#![deny(missing_docs)]

//! IO + tooling helpers for importing skeleton rigs from glTF/GLB,
//! hashing them deterministically, and emitting stable JSON artifacts.

use anyhow::{anyhow, Context, Result};
use blake3::Hasher;
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Physics-rig derivation + loading helpers.
pub mod rig_physics;

/// A single skeleton joint in the imported rig.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Joint {
    /// Joint display name (best-effort from node name; fallback `node_<index>`).
    pub name: String,
    /// Parent joint index in this rig, or -1 for root.
    pub parent: i16,
    /// Inverse bind matrix (column-major, glTF order).
    pub inverse_bind_colmajor_4x4: [f32; 16],
}

/// Stable serialized rig format emitted from a glTF/GLB skin.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RigData {
    /// Schema version for this JSON layout.
    pub version: u32,
    /// Original glTF/GLB file path used for import (provenance only; not hashed).
    pub source_file: String,
    /// Which skin index was used from the source file.
    pub skin_index: usize,
    /// Skin/skeleton name (from glTF skin name or a fallback).
    pub skeleton_name: String,
    /// Joints in deterministic order (skin.joints() order).
    pub joints: Vec<Joint>,
}

/// Compute a stable blake3 hash of this rig's topology & numeric data.
///
/// Notes:
/// - `source_file` is intentionally excluded (machine-dependent).
/// - Float bytes are hashed as stored in the JSON (little-endian f32).
pub fn rig_hash(rig: &RigData) -> [u8; 32] {
    let mut h = Hasher::new();
    h.update(b"RIGv1\0");
    h.update(&rig.version.to_le_bytes());
    h.update(rig.skeleton_name.as_bytes());
    h.update(&(rig.skin_index as u64).to_le_bytes());

    for j in &rig.joints {
        let n = j.name.as_bytes();
        h.update(&(n.len() as u64).to_le_bytes());
        h.update(n);
        h.update(&j.parent.to_le_bytes());
        for f in j.inverse_bind_colmajor_4x4 {
            h.update(&f.to_le_bytes());
        }
    }

    *h.finalize().as_bytes()
}

/// Write a `RigData` JSON file at `out_path`.
pub fn write_rig_json(rig: &RigData, out_path: &Path, pretty: bool) -> Result<()> {
    let json = if pretty {
        serde_json::to_string_pretty(rig)?
    } else {
        serde_json::to_string(rig)?
    };
    std::fs::write(out_path, json)?;
    Ok(())
}

/// Extract a `RigData` from a glTF/GLB file. Uses the given `skin_index` or 0.
///
/// This reads:
/// - joint list + nearest-joint parent indices
/// - inverse bind matrices (Mat4<f32>, column-major)
pub fn extract_rig_from_gltf(path: &Path, skin_index: Option<usize>) -> Result<RigData> {
    let (doc, buffers, _images) = gltf::import(path)
        .with_context(|| format!("failed to import glTF/GLB: {}", path.display()))?;

    // Choose skin
    let mut skins: Vec<gltf::Skin<'_>> = doc.skins().collect();
    if skins.is_empty() {
        return Err(anyhow!("no skins found in {}", path.display()));
    }

    let idx = skin_index.unwrap_or(0);
    if idx >= skins.len() {
        return Err(anyhow!(
            "skin index {} out of bounds (0..{})",
            idx,
            skins.len() - 1
        ));
    }

    let skin = skins.swap_remove(idx);
    let skel_name = skin.name().unwrap_or("Skin").to_string();

    // Joint nodes and lookup: node_index -> joint_index
    let joint_nodes: Vec<gltf::Node<'_>> = skin.joints().collect();
    if joint_nodes.is_empty() {
        return Err(anyhow!("skin {} has zero joints", idx));
    }

    let mut node_to_joint = std::collections::HashMap::<usize, usize>::new();
    for (i, n) in joint_nodes.iter().enumerate() {
        node_to_joint.insert(n.index(), i);
    }

    // Build a global parent map: node_index -> parent_node_index
    let node_count = doc.nodes().count();
    let mut parent_of: Vec<Option<usize>> = vec![None; node_count];
    for n in doc.nodes() {
        for ch in n.children() {
            parent_of[ch.index()] = Some(n.index());
        }
    }

    // For each joint node: walk up until nearest ancestor that is also a joint.
    let mut parents: Vec<i16> = Vec::with_capacity(joint_nodes.len());
    for n in &joint_nodes {
        let mut cur = parent_of[n.index()];
        let mut found = -1i16;
        while let Some(pi) = cur {
            if let Some(&ji) = node_to_joint.get(&pi) {
                found = ji as i16;
                break;
            }
            cur = parent_of[pi];
        }
        parents.push(found);
    }

    // Read inverseBindMatrices manually from accessor view (Mat4<f32>, 64 bytes each).
    let ibm_mats: Vec<[f32; 16]> = if let Some(acc) = skin.inverse_bind_matrices() {
        // Validate expected accessor layout (helps catch weird exports early).
        if acc.data_type() != gltf::accessor::DataType::F32 {
            return Err(anyhow!("inverseBindMatrices accessor is not f32"));
        }
        if acc.dimensions() != gltf::accessor::Dimensions::Mat4 {
            return Err(anyhow!("inverseBindMatrices accessor is not Mat4"));
        }

        let view = acc
            .view()
            .ok_or_else(|| anyhow!("inverseBindMatrices has no buffer view"))?;

        let buf_index = view.buffer().index();
        let raw = &buffers[buf_index].0;

        let stride = view.stride().unwrap_or(64) as usize;
        let base = (view.offset() + acc.offset()) as usize;
        let want = joint_nodes.len();

        let mut out = Vec::with_capacity(want);
        for i in 0..want {
            let offs = base + i * stride;

            if offs + 64 <= raw.len() {
                let mut m = [0.0f32; 16];
                for k in 0..16 {
                    let o = offs + k * 4;
                    m[k] = f32::from_le_bytes([raw[o], raw[o + 1], raw[o + 2], raw[o + 3]]);
                }
                out.push(m);
            } else {
                // Deterministic fallback if buffer is short.
                out.push([
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0,
                ]);
            }
        }
        out
    } else {
        vec![
            [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ];
            joint_nodes.len()
        ]
    };

    // Build joints vec (stable joint order = skin.joints() order).
    let mut joints = Vec::with_capacity(joint_nodes.len());
    for (i, node) in joint_nodes.iter().enumerate() {
        let name = node
            .name()
            .map(|s| s.to_string())
            .unwrap_or_else(|| format!("node_{}", node.index()));

        joints.push(Joint {
            name,
            parent: parents[i],
            inverse_bind_colmajor_4x4: ibm_mats[i],
        });
    }

    Ok(RigData {
        version: 1,
        source_file: path.to_string_lossy().to_string(),
        skin_index: idx,
        skeleton_name: skel_name,
        joints,
    })
}

/// Utility: hex-encode a 32-byte hash.
pub fn hex32(x: [u8; 32]) -> String {
    use std::fmt::Write;
    let mut s = String::with_capacity(64);
    for b in &x {
        let _ = write!(s, "{:02x}", b);
    }
    s
}
