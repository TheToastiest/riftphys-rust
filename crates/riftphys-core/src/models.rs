use std::sync::Arc;

use crate::types::Quat;
use crate::StepCtx;

#[derive(Copy, Clone, Debug, Default)]
pub struct AeroHandle(pub u32);

#[derive(Copy, Clone, Debug, Default)]
pub struct PropHandle(pub u32);

#[derive(Copy, Clone, Debug, Default)]
pub struct AccelPackHandle {
    pub aero: Option<AeroHandle>,
    pub prop: Option<PropHandle>,
}

pub trait AeroModel: Send + Sync {
    fn accel_contrib(&self, ctx: &StepCtx, query: AeroQuery) -> [f32; 3];
}

pub trait PropulsionModel: Send + Sync {
    fn accel_contrib(&self, ctx: &StepCtx, query: PropQuery) -> [f32; 3];
}

#[derive(Copy, Clone, Debug, Default)]
pub struct AeroQuery {
    pub vel_world: [f32; 3],
    pub ang_vel_world: [f32; 3],
    pub orientation: Quat,
    pub area: f32,
    pub mass: f32,
    pub altitude: f32,
    pub alpha_rad: f32,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct PropQuery {
    pub throttle01: f32,
    pub forward_dir_world: [f32; 3],
    pub mass: f32,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct ModelPackId(pub u32);

pub struct ModelPack {
    pub default_accel: AccelPackHandle,
}

pub struct ModelRegistry {
    aeros: Vec<Arc<dyn AeroModel>>,
    props: Vec<Arc<dyn PropulsionModel>>,
    active_pack: ModelPackId,
    packs: Vec<ModelPack>,
}

impl ModelRegistry {
    pub fn new() -> Self {
        Self {
            aeros: vec![],
            props: vec![],
            active_pack: ModelPackId(0),
            packs: vec![ModelPack { default_accel: AccelPackHandle::default() }],
        }
    }

    pub fn register_aero(&mut self, m: Arc<dyn AeroModel>) -> AeroHandle {
        let id = self.aeros.len() as u32;
        self.aeros.push(m);
        AeroHandle(id)
    }

    pub fn register_prop(&mut self, m: Arc<dyn PropulsionModel>) -> PropHandle {
        let id = self.props.len() as u32;
        self.props.push(m);
        PropHandle(id)
    }

    pub fn make_pack(&mut self, accel: AccelPackHandle) -> ModelPackId {
        let id = self.packs.len() as u32;
        self.packs.push(ModelPack { default_accel: accel });
        ModelPackId(id)
    }

    pub fn activate(&mut self, id: ModelPackId) {
        debug_assert!((id.0 as usize) < self.packs.len());
        self.active_pack = id;
    }

    pub fn active_pack(&self) -> &ModelPack {
        let idx = self.active_pack.0 as usize;
        debug_assert!(idx < self.packs.len());
        &self.packs[idx]
    }

    pub fn aero(&self, h: AeroHandle) -> &dyn AeroModel {
        let idx = h.0 as usize;
        debug_assert!(idx < self.aeros.len());
        &*self.aeros[idx]
    }

    pub fn prop(&self, h: PropHandle) -> &dyn PropulsionModel {
        let idx = h.0 as usize;
        debug_assert!(idx < self.props.len());
        &*self.props[idx]
    }
}
