use anyhow::Result;
use clap::Parser;
use std::fs;
use riftphys_io::RigData;

#[derive(Parser, Debug)]
#[command(name="rig_names")]
struct Opts { path: String }

fn main() -> Result<()> {
    let o = Opts::parse();
    let s = fs::read_to_string(&o.path)?;
    let rig: RigData = serde_json::from_str(&s)?;
    for (i,j) in rig.joints.iter().enumerate() {
        println!("{:4}  {:>3}  {}", i, j.parent, j.name);
    }
    Ok(())
}
