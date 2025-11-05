use glam::{UVec2, Vec2, Vec3};

/// Regular grid heightfield. Heights are in world units (before any extra y_offset).
#[derive(Clone, Debug)]
pub struct HeightField {
    pub dims: UVec2,     // nx, nz (columns in x, rows in z)
    pub cell: Vec2,      // sx, sz (world units per cell)
    pub heights: Vec<f32>,
    pub min_y: f32,
    pub max_y: f32,
}

impl HeightField {
    pub fn from_heights(dims: UVec2, cell: Vec2, heights: Vec<f32>) -> Self {
        assert_eq!((dims.x as usize) * (dims.y as usize), heights.len());
        let (mut min_y, mut max_y) = (f32::INFINITY, f32::NEG_INFINITY);
        for &h in &heights { min_y = min_y.min(h); max_y = max_y.max(h); }
        Self { dims, cell, heights, min_y, max_y }
    }

    #[cfg(feature = "image")]
    pub fn from_png_bytes(png: &[u8], cell: Vec2, y_scale: f32) -> image::ImageResult<Self> {
        let img = image::load_from_memory(png)?.to_luma8();
        let (w, h) = img.dimensions();
        let mut heights = Vec::with_capacity((w*h) as usize);
        for z in 0..h {
            for x in 0..w {
                let v = img.get_pixel(x, z).0[0] as f32 / 255.0;
                heights.push(v * y_scale);
            }
        }
        Ok(Self::from_heights(UVec2::new(w, h), cell, heights))
    }

    #[inline] fn idx(&self, x: i32, z: i32) -> usize {
        (x as usize) + (z as usize) * (self.dims.x as usize)
    }
    #[inline] fn h(&self, x: i32, z: i32) -> f32 { self.heights[self.idx(x, z)] }

    /// Bilinear height at local (x,z) in **meters** where origin is HF (0,0).
    pub fn sample_height(&self, x: f32, z: f32) -> f32 {
        let nx = self.dims.x as i32; let nz = self.dims.y as i32;
        let sx = self.cell.x;        let sz = self.cell.y;
        let fx = (x / sx).clamp(0.0, (nx - 1) as f32 - 1e-5);
        let fz = (z / sz).clamp(0.0, (nz - 1) as f32 - 1e-5);
        let x0 = fx.floor() as i32; let x1 = (x0 + 1).min(nx - 1);
        let z0 = fz.floor() as i32; let z1 = (z0 + 1).min(nz - 1);
        let tx = fx - x0 as f32;    let tz = fz - z0 as f32;

        let h00 = self.h(x0, z0);
        let h10 = self.h(x1, z0);
        let h01 = self.h(x0, z1);
        let h11 = self.h(x1, z1);
        let a = h00 * (1.0 - tx) + h10 * tx;
        let b = h01 * (1.0 - tx) + h11 * tx;
        a * (1.0 - tz) + b * tz
    }

    /// Central-diff normal (unit). Samples 3x3 neighborhood around (x,z).
    pub fn sample_normal(&self, x: f32, z: f32) -> Vec3 {
        // sample ± one cell in x and z for gradient
        let hx0 = self.sample_height((x - self.cell.x).max(0.0), z);
        let hx1 = self.sample_height(x + self.cell.x, z);
        let hz0 = self.sample_height(x, (z - self.cell.y).max(0.0));
        let hz1 = self.sample_height(x, z + self.cell.y);

        // dh/dx, dh/dz scaled by cell size
        let ddx = (hx1 - hx0) / (2.0 * self.cell.x.max(1e-6));
        let ddz = (hz1 - hz0) / (2.0 * self.cell.y.max(1e-6));

        let n = Vec3::new(-ddx, 1.0, -ddz);
        n.normalize_or_zero()
    }
}
