use riftphys_core::types::{Mat3, Vec3};
use riftphys_materials::materials::{MaterialId, phys_props};

const MIN_MASS: f32 = 1.0e-12;
const MIN_INERTIA: f32 = 1.0e-12;

#[derive(Copy, Clone, Debug)]
pub struct MassProps {
    pub mass: f32,
    pub inv_mass: f32,
    /// Local-space inertia tensor (diagonal for our primitives).
    pub inertia: Mat3,
}

impl MassProps {
    #[inline]
    pub fn infinite() -> Self {
        Self {
            mass: f32::INFINITY,
            inv_mass: 0.0,
            // Static bodies must have zero inverse inertia. Storing ZERO inertia forces
            // callers to handle it explicitly (no accidental “identity inverse”).
            inertia: Mat3::ZERO,
        }
    }

    #[inline(always)]
    fn valid_positive(x: f32) -> bool {
        x.is_finite() && x > 0.0
    }

    #[inline(always)]
    fn finalize(m: f32, inertia_diag: Vec3) -> Self {
        if !Self::valid_positive(m) || m < MIN_MASS {
            return Self::infinite();
        }

        // If inertia is non-finite or non-positive, treat as static rather than emitting NaNs/Infs.
        let ix = inertia_diag.x;
        let iy = inertia_diag.y;
        let iz = inertia_diag.z;
        if !(Self::valid_positive(ix) && Self::valid_positive(iy) && Self::valid_positive(iz)) {
            return Self::infinite();
        }

        Self {
            mass: m,
            inv_mass: 1.0 / m,
            inertia: Mat3::from_diagonal(inertia_diag.into()),
        }
    }

    /// Compute local-space inverse inertia for solver use (diagonal-only, deterministic).
    #[inline]
    pub fn inertia_inv_local(&self) -> Mat3 {
        if self.inv_mass == 0.0 {
            return Mat3::ZERO;
        }

        let ix = self.inertia.x_axis.x;
        let iy = self.inertia.y_axis.y;
        let iz = self.inertia.z_axis.z;

        let inv_ix = if ix.is_finite() && ix > MIN_INERTIA { 1.0 / ix } else { 0.0 };
        let inv_iy = if iy.is_finite() && iy > MIN_INERTIA { 1.0 / iy } else { 0.0 };
        let inv_iz = if iz.is_finite() && iz > MIN_INERTIA { 1.0 / iz } else { 0.0 };

        Mat3::from_diagonal(Vec3::new(inv_ix, inv_iy, inv_iz).into())

    }

    // ---- density-based primitives ----

    #[inline]
    pub fn from_sphere_density(radius: f32, density: f32) -> Self {
        let r = radius.abs();
        let rho = density;
        if !(Self::valid_positive(r) && Self::valid_positive(rho)) {
            return Self::infinite();
        }

        let r2 = r * r;
        let vol = (4.0 / 3.0) * core::f32::consts::PI * r2 * r;
        let m = rho * vol;

        // Solid sphere: I = 2/5 m r^2 = 0.4 m r^2
        let ii = 0.4 * m * r2;
        Self::finalize(m, Vec3::splat(ii))
    }

    #[inline]
    pub fn from_box_density(half: Vec3, density: f32) -> Self {
        let he = half.abs();
        let rho = density;
        if !(he.is_finite() && Self::valid_positive(rho)) {
            return Self::infinite();
        }

        let dims = he * 2.0;
        if !(Self::valid_positive(dims.x) && Self::valid_positive(dims.y) && Self::valid_positive(dims.z)) {
            return Self::infinite();
        }

        let vol = dims.x * dims.y * dims.z;
        let m = rho * vol;

        // Solid box about center:
        // Ix = 1/12 m (y^2 + z^2) etc (dims are full lengths).
        let x2 = dims.x * dims.x;
        let y2 = dims.y * dims.y;
        let z2 = dims.z * dims.z;

        let ix = (1.0 / 12.0) * m * (y2 + z2);
        let iy = (1.0 / 12.0) * m * (x2 + z2);
        let iz = (1.0 / 12.0) * m * (x2 + y2);

        Self::finalize(m, Vec3::new(ix, iy, iz))
    }

    #[inline]
    pub fn from_capsule_density(radius: f32, half_h: f32, density: f32) -> Self {
        let r = radius.abs();
        let hh = half_h.abs();
        let rho = density;

        if !(Self::valid_positive(r) && rho.is_finite() && rho > 0.0) {
            return Self::infinite();
        }

        let h = hh * 2.0;
        let r2 = r * r;

        // Volume: cylinder + sphere (note: “sphere” here double-counts the cylinder overlap, but
        // for gameplay-grade inertia this is fine; if you want exact capsule inertia later, we can.)
        let vol_cyl = core::f32::consts::PI * r2 * h;
        let vol_sph = (4.0 / 3.0) * core::f32::consts::PI * r2 * r;
        let m = rho * (vol_cyl + vol_sph);

        // Approx inertia (common game formula):
        // About X/Z: 1/4 m r^2 + 1/12 m h^2; about Y: 1/2 m r^2
        let ix = 0.25 * m * r2 + (1.0 / 12.0) * m * h * h;
        let iy = 0.5 * m * r2;
        let iz = ix;

        Self::finalize(m, Vec3::new(ix, iy, iz))
    }

    // ---- material-ID convenience wrappers ----

    #[inline]
    pub fn from_sphere(radius: f32, mat: MaterialId) -> Self {
        let rho = phys_props(mat).density;
        Self::from_sphere_density(radius, rho)
    }

    #[inline]
    pub fn from_box(half: Vec3, mat: MaterialId) -> Self {
        let rho = phys_props(mat).density;
        Self::from_box_density(half, rho)
    }

    #[inline]
    pub fn from_capsule(radius: f32, half_h: f32, mat: MaterialId) -> Self {
        let rho = phys_props(mat).density;
        Self::from_capsule_density(radius, half_h, rho)
    }
}
