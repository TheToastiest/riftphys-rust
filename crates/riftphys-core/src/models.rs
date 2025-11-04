// core/src/models.rs
use std::sync::Arc;
use glam::Quat;
use crate::StepCtx;

#[derive(Copy, Clone, Debug)]
pub struct AeroHandle(pub u32);
#[derive(Copy, Clone, Debug)]
pub struct PropHandle(pub u32);
#[derive(Copy, Clone, Debug)]
pub struct AccelPackHandle { pub aero: Option<AeroHandle>, pub prop: Option<PropHandle> }

pub trait AeroModel: Send + Sync {
    fn accel_contrib(&self, ctx: &StepCtx, query: AeroQuery) -> [f32; 3]; // body-frame or world-frame as agreed
}

pub trait PropulsionModel: Send + Sync {
    fn accel_contrib(&self, ctx: &StepCtx, query: PropQuery) -> [f32; 3];
}
#[derive(Copy, Clone, Debug)]

pub struct AeroQuery {
    pub vel_world: [f32; 3],
    pub ang_vel_world: [f32; 3],
    pub orientation: Quat, // or your math type
    pub area: f32,         // reference area
    pub mass: f32,         // for a = F/m
    pub altitude: f32,
    pub alpha_rad: f32,
}
#[derive(Copy, Clone, Debug)]

pub struct PropQuery {
    pub throttle01: f32,
    pub forward_dir_world: [f32; 3],
    pub mass: f32,
}

pub struct ModelRegistry {
    aeros: Vec<Arc<dyn AeroModel>>,
    props: Vec<Arc<dyn PropulsionModel>>,
    active_pack: ModelPackId,
    packs: Vec<ModelPack>,
}

#[derive(Copy, Clone, Debug)]
pub struct ModelPackId(pub u32);

// which concrete models/params are active for each vehicle type or group
pub struct ModelPack {
    pub default_accel: AccelPackHandle,
}

impl ModelRegistry {
    pub fn new() -> Self { Self { aeros: vec![], props: vec![], active_pack: ModelPackId(0), packs: vec![ModelPack{ default_accel: AccelPackHandle{aero: None, prop: None}}] } }
    pub fn register_aero(&mut self, m: Arc<dyn AeroModel>) -> AeroHandle {
        let id = self.aeros.len() as u32; self.aeros.push(m); AeroHandle(id)
    }
    pub fn register_prop(&mut self, m: Arc<dyn PropulsionModel>) -> PropHandle {
        let id = self.props.len() as u32; self.props.push(m); PropHandle(id)
    }
    pub fn make_pack(&mut self, accel: AccelPackHandle) -> ModelPackId {
        let id = self.packs.len() as u32; self.packs.push(ModelPack{ default_accel: accel }); ModelPackId(id)
    }
    pub fn activate(&mut self, id: ModelPackId) { self.active_pack = id; }
    pub fn active_pack(&self) -> &ModelPack { &self.packs[self.active_pack.0 as usize] }
    pub fn aero(&self, h: AeroHandle) -> &dyn AeroModel { &*self.aeros[h.0 as usize] }
    pub fn prop(&self, h: PropHandle) -> &dyn PropulsionModel { &*self.props[h.0 as usize] }
}
