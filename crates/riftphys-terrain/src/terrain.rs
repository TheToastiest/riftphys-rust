use glam::{UVec2, Vec2};
use riftphys_core::det::{q6, q6_unit_vec3, safe_contact_dist};
use riftphys_core::types::Vec3;

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
    pub fn from_heights(dims: UVec2, cell: Vec2, mut heights: Vec<f32>) -> Self {
        let nx = dims.x as usize;
        let nz = dims.y as usize;
        assert!(nx >= 1 && nz >= 1, "HeightField dims must be >= 1");
        assert_eq!(nx * nz, heights.len(), "HeightField heights len mismatch");

        // Quantize + sanitize cell sizes (no NaN/0)
        let sx = if cell.x.is_finite() && cell.x > 0.0 { q6(cell.x) } else { 1.0 };
        let sz = if cell.y.is_finite() && cell.y > 0.0 { q6(cell.y) } else { 1.0 };
        let cell = Vec2::new(sx, sz);

        let (mut min_y, mut max_y) = (f32::INFINITY, f32::NEG_INFINITY);

        for h in &mut heights {
            // NaN/inf in terrain is poison; clamp deterministically.
            let v = if h.is_finite() { *h } else { 0.0 };
            *h = q6(v);
            min_y = min_y.min(*h);
            max_y = max_y.max(*h);
        }

        // Quantize bounds too (so derived stuff doesn’t differ by ulps)
        min_y = q6(min_y);
        max_y = q6(max_y);

        Self { dims, cell, heights, min_y, max_y }
    }

    #[cfg(feature = "image")]
    pub fn from_png_bytes(png: &[u8], cell: Vec2, y_scale: f32) -> image::ImageResult<Self> {
        let img = image::load_from_memory(png)?.to_luma8();
        let (w, h) = img.dimensions();
        let mut heights = Vec::with_capacity((w * h) as usize);

        let ys = if y_scale.is_finite() { y_scale } else { 0.0 };

        for z in 0..h {
            for x in 0..w {
                let v = img.get_pixel(x, z).0[0] as f32 / 255.0;
                heights.push(v * ys);
            }
        }

        Ok(Self::from_heights(UVec2::new(w, h), cell, heights))
    }

    /// RAW32 (little-endian f32), square, mapped onto a world-space tile.
    pub fn from_raw32_square(bytes: &[u8], world_size: Vec2) -> Result<Self, String> {
        if bytes.len() % 4 != 0 {
            return Err(format!("raw32 length {} not multiple of 4", bytes.len()));
        }
        let samples = bytes.len() / 4;

        let n_f = (samples as f64).sqrt();
        let n = n_f.round() as u32;
        if (n as usize) * (n as usize) != samples {
            return Err(format!("raw32 samples {} not a perfect square", samples));
        }

        let mut heights = Vec::with_capacity(samples);
        for chunk in bytes.chunks_exact(4) {
            let mut h = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            if !h.is_finite() { h = 0.0; }
            heights.push(h);
        }

        let dims = UVec2::new(n.max(1), n.max(1));

        // If n==1, treat as a single sample cell (avoid divide-by-zero).
        let denom = (n as f32 - 1.0).max(1.0);
        let wsx = if world_size.x.is_finite() { world_size.x } else { 0.0 };
        let wsz = if world_size.y.is_finite() { world_size.y } else { 0.0 };

        let cell = Vec2::new(
            q6(wsx / denom),
            q6(wsz / denom),
        );

        Ok(Self::from_heights(dims, cell, heights))
    }

    #[inline]
    fn idx(&self, x: i32, z: i32) -> usize {
        (x as usize) + (z as usize) * (self.dims.x as usize)
    }

    #[inline]
    fn h(&self, x: i32, z: i32) -> f32 {
        self.heights[self.idx(x, z)]
    }

    #[inline]
    fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
        // Never panics if hi < lo; it becomes a single-point clamp.
        let (lo, hi) = if hi >= lo { (lo, hi) } else { (lo, lo) };
        x.max(lo).min(hi)
    }

    /// Bilinear height at local (x,z) in **meters** where origin is HF (0,0).
    pub fn sample_height(&self, x: f32, z: f32) -> f32 {
        let nx = self.dims.x as i32;
        let nz = self.dims.y as i32;

        if nx <= 1 && nz <= 1 {
            return self.heights[0];
        }

        let sx = if self.cell.x.is_finite() && self.cell.x > 0.0 { self.cell.x } else { 1.0 };
        let sz = if self.cell.y.is_finite() && self.cell.y > 0.0 { self.cell.y } else { 1.0 };

        // Convert world meters to grid coords. Clamp so x1/z1 stay in range.
        let max_fx = (nx - 1) as f32 - 1.0e-6;
        let max_fz = (nz - 1) as f32 - 1.0e-6;

        let fx = Self::clampf(x / sx, 0.0, max_fx);
        let fz = Self::clampf(z / sz, 0.0, max_fz);

        let x0 = fx.floor() as i32; let x1 = (x0 + 1).min(nx - 1);
        let z0 = fz.floor() as i32; let z1 = (z0 + 1).min(nz - 1);

        // Quantize lerp factors to reduce ulp drift.
        let tx = q6(fx - x0 as f32);
        let tz = q6(fz - z0 as f32);

        let h00 = self.h(x0, z0);
        let h10 = self.h(x1, z0);
        let h01 = self.h(x0, z1);
        let h11 = self.h(x1, z1);

        let a = h00 * (1.0 - tx) + h10 * tx;
        let b = h01 * (1.0 - tx) + h11 * tx;
        q6(a * (1.0 - tz) + b * tz)
    }

    /// Central-diff normal (unit, quantized). Samples ± one cell in x and z.
    pub fn sample_normal(&self, x: f32, z: f32) -> Vec3 {
        // Degenerate HF → deterministic up
        let nx = self.dims.x as i32;
        let nz = self.dims.y as i32;
        if nx <= 1 || nz <= 1 {
            return Vec3::new(0.0, 1.0, 0.0);
        }

        let sx = self.cell.x.max(1.0e-6);
        let sz = self.cell.y.max(1.0e-6);

        let hx0 = self.sample_height(x - sx, z);
        let hx1 = self.sample_height(x + sx, z);
        let hz0 = self.sample_height(x, z - sz);
        let hz1 = self.sample_height(x, z + sz);

        // ddx, ddz in world slope
        let ddx = (hx1 - hx0) * (0.5 / sx);
        let ddz = (hz1 - hz0) * (0.5 / sz);

        // Up is +Y. Surface normal points “uphill opposite gradient”.
        let n = Vec3::new(-ddx, 1.0, -ddz);
        q6_unit_vec3(n)
    }

    /// Useful for callers that want a deterministic “surface distance” check.
    #[inline]
    pub fn contact_dist(cd: f32) -> f32 {
        safe_contact_dist(cd)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_height_no_panic_on_1x1() {
        let hf = HeightField::from_heights(UVec2::new(1, 1), Vec2::new(1.0, 1.0), vec![2.0]);
        assert_eq!(hf.sample_height(0.0, 0.0), 2.0);
        let n = hf.sample_normal(0.0, 0.0);
        assert!((n.y - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn bilinear_basic() {
        // 2x2 grid: [0 1; 2 3] across x,z
        let hf = HeightField::from_heights(
            UVec2::new(2, 2),
            Vec2::new(1.0, 1.0),
            vec![0.0, 1.0,
                 2.0, 3.0],
        );
        let h = hf.sample_height(0.5, 0.5);
        // bilinear center = avg = 1.5
        assert!((h - 1.5).abs() < 1.0e-6);
    }

    #[test]
    fn normal_is_unitish() {
        let hf = HeightField::from_heights(
            UVec2::new(3, 3),
            Vec2::new(1.0, 1.0),
            vec![0.0, 0.0, 0.0,
                 0.0, 1.0, 2.0,
                 0.0, 2.0, 4.0],
        );
        let n = hf.sample_normal(1.0, 1.0);
        let len = (n.x*n.x + n.y*n.y + n.z*n.z).sqrt();
        assert!((len - 1.0).abs() < 1.0e-3);
    }
}
