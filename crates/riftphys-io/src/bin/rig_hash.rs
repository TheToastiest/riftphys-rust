use std::path::{Path, PathBuf};
use anyhow::{Context, Result, anyhow};
use clap::{Parser, ArgAction};
use riftphys_io::{extract_rig_from_gltf, rig_hash, write_rig_json};

#[derive(Parser, Debug)]
#[command(name="rig_hash", version, about="Import a GLB/GLTF skeleton and emit a stable RigData JSON + hash")]
struct Opts {
    /// Path to .glb/.gltf
    input: PathBuf,

    /// Skin index to import (default: 0)
    #[arg(long)]
    skin: Option<usize>,

    /// Skin name to import (case-insensitive). Takes precedence over --skin.
    #[arg(long)]
    skin_name: Option<String>,

    /// Only list skins and exit
    #[arg(long, action=ArgAction::SetTrue)]
    list: bool,

    /// Output .rig.json path (default: <input>.rig.json next to source)
    #[arg(long)]
    out: Option<PathBuf>,

    /// Pretty-print JSON
    #[arg(long, action=ArgAction::SetTrue)]
    pretty: bool,
}

fn resolve_skin_index(doc: &gltf::Document, num: Option<usize>, name: &Option<String>) -> Result<usize> {
    let skins: Vec<gltf::Skin> = doc.skins().collect();
    if skins.is_empty() {
        return Err(anyhow!("no skins in document"));
    }
    if let Some(ref n) = *name {
        let want = n.to_ascii_lowercase();
        for (i, s) in skins.iter().enumerate() {
            if s.name().map(|x| x.to_ascii_lowercase()) == Some(want.clone()) {
                return Ok(i);
            }
        }
        eprintln!("Skin named {:?} not found. Available:", n);
        for (i, s) in skins.iter().enumerate() {
            eprintln!("  [{}] {}", i, s.name().unwrap_or("(unnamed)"));
        }
        return Err(anyhow!("invalid --skin-name"));
    }
    let idx = num.unwrap_or(0);
    if idx >= skins.len() {
        eprintln!("Skin index {} out of bounds (0..{}). Available:", idx, skins.len()-1);
        for (i, s) in skins.iter().enumerate() {
            eprintln!("  [{}] {}", i, s.name().unwrap_or("(unnamed)"));
        }
        return Err(anyhow!("invalid --skin"));
    }
    Ok(idx)
}

fn main() -> Result<()> {
    let opts = Opts::parse();

    if opts.list {
        let gltf = gltf::Gltf::open(&opts.input)
            .with_context(|| format!("open {}", opts.input.display()))?;
        let skins: Vec<gltf::Skin> = gltf.document.skins().collect();
        if skins.is_empty() {
            println!("(no skins) {}", opts.input.display());
            return Ok(());
        }
        println!("skins in {}:", opts.input.display());
        for (i, s) in skins.iter().enumerate() {
            println!("  [{}] {}", i, s.name().unwrap_or("(unnamed)"));
        }
        return Ok(());
    }

    // Resolve skin (by name first, else index)
    let gltf = gltf::Gltf::open(&opts.input)
        .with_context(|| format!("open {}", opts.input.display()))?;
    let skin_idx = resolve_skin_index(&gltf.document, opts.skin, &opts.skin_name)?;

    // Extract & write
    let rig  = extract_rig_from_gltf(&opts.input, Some(skin_idx))?;
    let hash = rig_hash(&rig);
    let hex = {
        let mut s = String::with_capacity(64);
        for b in hash { use std::fmt::Write; let _ = write!(s, "{:02x}", b); }
        s
    };

    let out = opts.out.clone().unwrap_or_else(|| {
        let mut p = opts.input.clone();
        p.set_extension("rig.json");
        p
    });
    write_rig_json(&rig, &out, opts.pretty)?;

    let mut hash_path = out.clone();
    hash_path.set_extension("rig.blake3");
    std::fs::write(&hash_path, format!("{}\n", hex))?;

    println!("Rig:    {}", out.display());
    println!("Hash:   {}", hex);
    println!("Skin:   {}", rig.skin_index);
    println!("Joints: {}", rig.joints.len());
    println!("Name:   {}", rig.skeleton_name);
    Ok(())
}
