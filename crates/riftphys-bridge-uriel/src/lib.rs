use anyhow::{Result, anyhow};
use serde::{Serialize,Deserialize};

// ADD these:
use riftphys_core::BodyId;
use riftphys_core::models::{AeroHandle, PropHandle};
use riftphys_gravity::GravitySpec;

#[derive(Serialize,Deserialize,Debug)]
pub struct SimulationDescriptor {
    pub dt: f32,
    pub gravity: GravityBlock,
    pub accel: Vec<AccelBlock>, // per-body overrides
}

#[derive(Serialize,Deserialize,Debug)]
pub struct GravityBlock {
    pub model: String,  // "Uniform" | "LayeredPlanet"
    pub params: serde_json::Value
}

#[derive(Serialize,Deserialize,Debug)]
pub struct AccelBlock {
    pub body_index: u32,
    pub ref_area_m2: f32,
    pub throttle01: f32,
    // These are registry indices you already have from register_aero/prop.
    // We'll wrap them into the proper handle types below.
    pub aero: Option<usize>,
    pub prop: Option<usize>,
}

pub fn validate_and_queue(world: &mut riftphys_world::World, sd: &SimulationDescriptor) -> Result<()> {
    if !(sd.dt > 0.0 && sd.dt <= 1.0) { return Err(anyhow!("dt out of range")); }

    // Gravity
    let gspec = match sd.gravity.model.as_str() {
        "Uniform" => {
            let a = sd.gravity.params.get("g").and_then(|v| v.as_array()).ok_or_else(|| anyhow!("uniform.g missing"))?;
            if a.len() != 3 { return Err(anyhow!("uniform.g must be len=3")); }
            let gx = a[0].as_f64().ok_or_else(|| anyhow!("gx"))? as f32;
            let gy = a[1].as_f64().ok_or_else(|| anyhow!("gy"))? as f32;
            let gz = a[2].as_f64().ok_or_else(|| anyhow!("gz"))? as f32;
            GravitySpec::Uniform { g: [gx, gy, gz] }
        }
        "LayeredPlanet" => {
            let sg = sd.gravity.params.get("surface_g").and_then(|v| v.as_f64()).ok_or_else(|| anyhow!("surface_g"))? as f32;
            let r  = sd.gravity.params.get("radius").and_then(|v| v.as_f64()).ok_or_else(|| anyhow!("radius"))? as f32;
            let c  = sd.gravity.params.get("center").and_then(|v| v.as_array()).ok_or_else(|| anyhow!("center"))?;
            if c.len()!=3 { return Err(anyhow!("center len!=3")); }
            let cx = c[0].as_f64().unwrap_or(0.0) as f32;
            let cy = c[1].as_f64().unwrap_or(0.0) as f32;
            let cz = c[2].as_f64().unwrap_or(0.0) as f32;
            let min_r = sd.gravity.params.get("min_r").and_then(|v| v.as_f64()).unwrap_or(1000.0) as f32;
            GravitySpec::LayeredPlanet { surface_g: sg, radius: r, center: [cx,cy,cz], min_r }
        }
        _ => return Err(anyhow!("unsupported gravity model")),
    };
    world.queue_gravity_swap(gspec);

    // Accel overrides
    for a in &sd.accel {
        let id = BodyId(a.body_index);
        // Wrap registry indices into the proper handle types
        // Wrap registry indices into handle types (usize → u32)
        let aero_h = a.aero.map(|i| AeroHandle(i as u32));
        let prop_h = a.prop.map(|i| PropHandle(i as u32));

        world.set_body_accel(id, aero_h, prop_h, a.ref_area_m2, a.throttle01, None);
    }
    Ok(())
}
