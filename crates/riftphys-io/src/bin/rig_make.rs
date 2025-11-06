use anyhow::Result;
use clap::Parser;
use std::{fs, path::PathBuf};
use riftphys_io::{RigData, rig_physics::humanoid_from_rig};

#[derive(Parser, Debug)]
#[command(name="rig_make", version, about="Build a PhysicsRig from a .rig.json (generated from a GLB)")]
struct Opts {
    /// Input .rig.json (from `rig_hash`)
    input: PathBuf,

    /// Output .physics.json (default: <input_stem>.physics.json next to input)
    #[arg(long)]
    out: Option<PathBuf>,

    /// Pretty-print JSON
    #[arg(long)]
    pretty: bool,
}

fn main() -> Result<()> {
    let opt = Opts::parse();

    // Load the serialized RigData (names, parents, source_file, skin index…)
    let s = fs::read_to_string(&opt.input)?;
    let rig: RigData = serde_json::from_str(&s)?;

    // Build a coarse physics rig from the GLB referenced in rig.source_file
    // (humanoid_from_rig re-opens the GLB to estimate segment lengths, etc.)
    let phys = humanoid_from_rig(&rig)?;

    // Decide output path
    let out = if let Some(p) = &opt.out {
        p.clone()
    } else {
        let mut p = opt.input.clone();
        p.set_file_name(format!(
            "{}.physics.json",
            p.file_stem().unwrap().to_string_lossy()
        ));
        p
    };

    // Write
    let json = if opt.pretty {
        serde_json::to_string_pretty(&phys)?
    } else {
        serde_json::to_string(&phys)?
    };
    fs::write(&out, json)?;

    println!("Physics rig: {}", out.display());
    Ok(())
}
