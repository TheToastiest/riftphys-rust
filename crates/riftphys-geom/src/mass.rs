use riftphys_core::types::{Mat3, Vec3};
use riftphys_materials;

#[derive(Copy,Clone,Debug)]
pub struct Material { pub density:f32, pub mu_s:f32, pub mu_k:f32, pub restitution:f32, pub id: riftphys_materials::MaterialId }
impl Default for Material {
    fn default()->Self{ Self{density:1000.0, mu_s:0.5, mu_k:0.4, restitution:0.0, id: riftphys_materials::MaterialId::Default}}
}

#[derive(Copy, Clone, Debug)]
pub struct MassProps {
    pub mass: f32,
    pub inv_mass: f32,
    pub inertia: Mat3,
}

impl MassProps {
    pub fn infinite() -> Self {
        Self { mass: f32::INFINITY, inv_mass: 0.0, inertia: Mat3::IDENTITY }
    }

    pub fn from_sphere(radius: f32, density: f32) -> Self {
        let vol = (4.0/3.0) * core::f32::consts::PI * radius*radius*radius;
        let m = density * vol;
        let ii = 0.4 * m * radius * radius;
        Self { mass: m, inv_mass: 1.0/m, inertia: Mat3::from_diagonal(Vec3::splat(ii).into()) }
    }

    pub fn from_box(half: Vec3, density: f32) -> Self {
        let dims = half * 2.0;
        let vol = dims.x * dims.y * dims.z;
        let m = density * vol;
        let x2 = dims.x * dims.x;
        let y2 = dims.y * dims.y;
        let z2 = dims.z * dims.z;
        let ix = (1.0/12.0) * m * (y2 + z2);
        let iy = (1.0/12.0) * m * (x2 + z2);
        let iz = (1.0/12.0) * m * (x2 + y2);
        Self { mass: m, inv_mass: 1.0/m, inertia: Mat3::from_diagonal(Vec3::new(ix, iy, iz).into()) }
    }

    pub fn from_capsule(radius: f32, half_h: f32, density: f32) -> Self {
        let h = half_h * 2.0;
        let vol_cyl = core::f32::consts::PI * radius*radius * h;
        let vol_sph = (4.0/3.0) * core::f32::consts::PI * radius*radius*radius;
        let m = density * (vol_cyl + vol_sph);
        let ix = 0.25 * m * radius*radius + (1.0/12.0) * m * h*h;
        let iy = 0.5 * m * radius*radius;
        let iz = ix;
        Self { mass: m, inv_mass: 1.0/m, inertia: Mat3::from_diagonal(Vec3::new(ix, iy, iz).into()) }
    }
}
