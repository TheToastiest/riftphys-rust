use anyhow::{anyhow, Context, Result};
use blake3::Hasher;
use glam::{Mat4, Vec3};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, path::Path};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    /// Parent joint index in this rig, or -1 for root.
    pub parent: i16,
    /// Inverse bind matrix (column-major, glTF order).
    pub inverse_bind_colmajor_4x4: [f32; 16],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RigData {
    pub version: u32,                // bump if layout changes
    pub source_file: String,         // original path (for provenance)
    pub skin_index: usize,           // which skin we used
    pub skeleton_name: String,       // from glTF skin name or synthesized
    pub joints: Vec<Joint>,
}

/// Compute a stable blake3 hash of this rig's topology & numeric data.
pub fn rig_hash(rig: &RigData) -> [u8; 32] {
    let mut h = Hasher::new();
    h.update(b"RIGv1\0");
    h.update(&(rig.version.to_le_bytes()));
    h.update(rig.skeleton_name.as_bytes());
    h.update(&(rig.skin_index as u64).to_le_bytes());
    // Joints in order
    for j in &rig.joints {
        // Name length + bytes (stable)
        let n = j.name.as_bytes();
        h.update(&(n.len() as u64).to_le_bytes());
        h.update(n);
        h.update(&(j.parent as i16).to_le_bytes());
        // 16 f32, little-endian
        for f in j.inverse_bind_colmajor_4x4 {
            h.update(&f.to_le_bytes());
        }
    }
    *h.finalize().as_bytes()
}

/// Write rig to JSON at `out_path`. If `pretty=true`, pretty-print JSON.
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
pub fn extract_rig_from_gltf(path: &Path, skin_index: Option<usize>) -> Result<RigData> {
    // Import buffers so we can read inverse bind matrices
    let (doc, buffers, _images) = gltf::import(path)
        .with_context(|| format!("failed to import glTF/GLB: {}", path.display()))?;

    // Choose skin
    let mut skins: Vec<gltf::Skin<'_>> = doc.skins().collect();
    if skins.is_empty() {
        return Err(anyhow!("no skins found in {}", path.display()));
    }
    let idx = skin_index.unwrap_or(0);
    if idx >= skins.len() {
        return Err(anyhow!("skin index {} out of bounds (0..{})", idx, skins.len() - 1));
    }
    let skin = skins.swap_remove(idx);
    let skel_name = skin.name().unwrap_or("Skin").to_string();

    // Joint nodes and a fast lookup
    let joint_nodes: Vec<gltf::Node> = skin.joints().collect();
    if joint_nodes.is_empty() {
        return Err(anyhow!("skin {} has zero joints", idx));
    }
    let mut node_to_joint = std::collections::HashMap::<usize, usize>::new();
    for (i, n) in joint_nodes.iter().enumerate() {
        node_to_joint.insert(n.index(), i);
    }

    // Build a global parent map: for each node, remember its direct parent (if any)
    let node_count = doc.nodes().count();
    let mut parent_of: Vec<Option<usize>> = vec![None; node_count];
    for n in doc.nodes() {
        for ch in n.children() {
            parent_of[ch.index()] = Some(n.index());
        }
    }

    // For each joint node, walk up parents until we find the nearest ancestor that is also a joint
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

    // Read inverseBindMatrices manually from buffer view (64 bytes per mat4x4<f32> col-major)
    let ibm_mats: Vec<[f32; 16]> = if let Some(acc) = skin.inverse_bind_matrices() {
        let view = acc.view().ok_or_else(|| anyhow!("inverseBindMatrices has no buffer view"))?;
        let buf_index = view.buffer().index();
        let raw = &buffers[buf_index].0; // gltf::buffer::Data holds Vec<u8> at .0
        let stride = view.stride().unwrap_or(64) as usize;
        let base   = (view.offset() + acc.offset()) as usize;
        let count  = acc.count() as usize;

        let want = joint_nodes.len();
        let mut out = Vec::with_capacity(want);
        for i in 0..want {
            let offs = base + i.saturating_mul(stride);
            if offs + 64 <= raw.len() {
                let mut m = [0.0f32; 16];
                // 16 little-endian f32s in column-major order
                for k in 0..16 {
                    let b0 = raw[offs + k*4 + 0];
                    let b1 = raw[offs + k*4 + 1];
                    let b2 = raw[offs + k*4 + 2];
                    let b3 = raw[offs + k*4 + 3];
                    m[k] = f32::from_le_bytes([b0,b1,b2,b3]);
                }
                out.push(m);
            } else {
                // fallback identity if buffer is short (keeps determinism)
                out.push([1.0,0.0,0.0,0.0,
                    0.0,1.0,0.0,0.0,
                    0.0,0.0,1.0,0.0,
                    0.0,0.0,0.0,1.0]);
            }
        }
        out
    } else {
        // No IBM accessor → identities for each joint
        vec![[1.0,0.0,0.0,0.0,
                 0.0,1.0,0.0,0.0,
                 0.0,0.0,1.0,0.0,
                 0.0,0.0,0.0,1.0]; joint_nodes.len()]
    };

    // Build joints vec
    let mut joints = Vec::with_capacity(joint_nodes.len());
    for (i, node) in joint_nodes.iter().enumerate() {
        let name = node.name().map(|s| s.to_string()).unwrap_or_else(|| format!("node_{}", node.index()));
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
    let mut s = String::with_capacity(64);
    for b in &x {
        use std::fmt::Write;
        let _ = write!(s, "{:02x}", b);
    }
    s
}