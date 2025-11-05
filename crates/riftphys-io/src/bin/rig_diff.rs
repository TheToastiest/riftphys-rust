use std::{fs, path::PathBuf, process::ExitCode};
use anyhow::{Result, anyhow};
use clap::{Parser};
use riftphys_io::RigData;

#[derive(Parser, Debug)]
#[command(name="rig_diff", version, about="Diff two .rig.json files (names, parents, matrices)")]
struct Opts {
    /// Left rig.json
    left: PathBuf,
    /// Right rig.json
    right: PathBuf,
    /// Epsilon for matrix element differences (abs)
    #[arg(long, default_value_t = 1.0e-5)]
    eps: f32,
    /// Max differences to print before truncating
    #[arg(long, default_value_t = 50)]
    max: usize,
}

fn load(path: &PathBuf) -> Result<RigData> {
    let s = fs::read_to_string(path)?;
    let rig: RigData = serde_json::from_str(&s)
        .map_err(|e| anyhow!("{}: {}", path.display(), e))?;
    Ok(rig)
}

fn max_abs_diff(a: &[f32;16], b: &[f32;16]) -> f32 {
    let mut m = 0.0f32;
    for i in 0..16 {
        let d = (a[i] - b[i]).abs();
        if d > m { m = d; }
    }
    m
}

fn main() -> ExitCode {
    if let Err(e) = run() {
        eprintln!("Error: {e}");
        return ExitCode::from(1);
    }
    ExitCode::from(0)
}

fn run() -> Result<()> {
    let opts = Opts::parse();
    let l = load(&opts.left)?;
    let r = load(&opts.right)?;

    let mut diffs = Vec::<String>::new();

    if l.version != r.version {
        diffs.push(format!("version: {} vs {}", l.version, r.version));
    }
    if l.skeleton_name != r.skeleton_name {
        diffs.push(format!("skeleton_name: {:?} vs {:?}", l.skeleton_name, r.skeleton_name));
    }
    if l.joints.len() != r.joints.len() {
        diffs.push(format!("joint count: {} vs {}", l.joints.len(), r.joints.len()));
    }

    let n = l.joints.len().min(r.joints.len());
    for i in 0..n {
        let lj = &l.joints[i];
        let rj = &r.joints[i];
        if lj.name != rj.name {
            diffs.push(format!("[{i}] name: {:?} vs {:?}", lj.name, rj.name));
        }
        if lj.parent != rj.parent {
            diffs.push(format!("[{i}] parent: {} vs {}", lj.parent, rj.parent));
        }
        let md = max_abs_diff(&lj.inverse_bind_colmajor_4x4, &rj.inverse_bind_colmajor_4x4);
        if md > opts.eps {
            diffs.push(format!("[{i}] inverse_bind max|Δ| = {:.6} > eps {:.6}", md, opts.eps));
        }
        if diffs.len() >= opts.max { break; }
    }

    if diffs.is_empty() {
        println!("OK ✅ rigs are equal within eps ({} joints vs {})", l.joints.len(), r.joints.len());
    } else {
        println!("DIFF ❌ (showing up to {} diffs):", opts.max);
        for d in diffs { println!("  - {}", d); }
    }
    Ok(())
}
