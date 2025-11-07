// Minimal raycast compute — no render shading here.

struct Config {
  dims:   vec4<u32>,   // x,y,z; w pad
  params: vec4<f32>,   // x = voxel_size
};
struct MatVis {
  base: vec3<f32>,
  rough: f32,
  metal: f32,
  _pad: vec2<f32>,
};

@group(1) @binding(0)
var<storage, read> MAT: array<MatVis>;

struct RayGpu { o: vec4<f32>, d_tmax: vec4<f32> };
struct HitGpu { t: f32, nx: f32, ny: f32, nz: f32, cx: u32, cy: u32, cz: u32, hit: u32 };

@group(0) @binding(0) var<storage, read>       voxels: array<u32>;
@group(0) @binding(1) var<uniform>             cfg: Config;
@group(0) @binding(2) var<storage, read>       rays: array<RayGpu>;
@group(0) @binding(3) var<storage, read_write> hits: array<HitGpu>;

fn get_voxel(ix: u32, iy: u32, iz: u32) -> bool {
  let idx: u32 = ix + cfg.dims.x * (iy + cfg.dims.y * iz);
  let word = idx >> 5u;
  let bit  = idx & 31u;
  return ((voxels[word] >> bit) & 1u) != 0u;
}

fn aabb_min() -> vec3<f32> { return vec3<f32>(0.0, 0.0, 0.0); }
fn aabb_max() -> vec3<f32> {
  return vec3<f32>(f32(cfg.dims.x), f32(cfg.dims.y), f32(cfg.dims.z)) * cfg.params.x;
}

fn ray_aabb(o: vec3<f32>, d: vec3<f32>, mn: vec3<f32>, mx: vec3<f32>) -> vec2<f32> {
  let inv = 1.0 / d;
  let t0s = (mn - o) * inv;
  let t1s = (mx - o) * inv;
  let lo = min(t0s, t1s);
  let hi = max(t0s, t1s);
  let t0 = max(max(lo.x, lo.y), lo.z);
  let t1 = min(min(hi.x, hi.y), hi.z);
  return vec2<f32>(t0, t1);
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
  let i = gid.x;
  if (i >= arrayLength(&rays)) { return; }

  let r = rays[i];
  var o = r.o.xyz;
  var d = r.d_tmax.xyz;
  let tmax = r.d_tmax.w;

  let len = length(d);
  if (len < 1e-12) { hits[i].hit = 0u; return; }
  d = d / len;

  let mn = aabb_min();
  let mx = aabb_max();

  let ab = ray_aabb(o, d, mn, mx);
  var t0: f32 = max(ab.x, 0.0);
  let t1: f32 = ab.y;
  if (t1 < t0 || t0 > tmax) { hits[i].hit = 0u; return; }

  let p = o + d * t0;
  var cell: vec3<i32> = vec3<i32>(floor(p / cfg.params.x));
  cell = clamp(cell, vec3<i32>(0), vec3<i32>(i32(cfg.dims.x)-1, i32(cfg.dims.y)-1, i32(cfg.dims.z)-1));

  let sx: i32 = select(-1, 1, d.x >= 0.0);
  let sy: i32 = select(-1, 1, d.y >= 0.0);
  let sz: i32 = select(-1, 1, d.z >= 0.0);

  var t: f32 = t0;

  // First boundary times
  var tmaxx: f32;
  if (abs(d.x) < 1e-12) {
    tmaxx = 1e30;
  } else {
    let edge_x = f32(cell.x + select(0, 1, sx > 0)) * cfg.params.x;
    tmaxx = (edge_x - p.x) / d.x + t;
  }
  var tmaxy: f32;
  if (abs(d.y) < 1e-12) {
    tmaxy = 1e30;
  } else {
    let edge_y = f32(cell.y + select(0, 1, sy > 0)) * cfg.params.x;
    tmaxy = (edge_y - p.y) / d.y + t;
  }
  var tmaxz: f32;
  if (abs(d.z) < 1e-12) {
    tmaxz = 1e30;
  } else {
    let edge_z = f32(cell.z + select(0, 1, sz > 0)) * cfg.params.x;
    tmaxz = (edge_z - p.z) / d.z + t;
  }

  var tdx: f32; if (abs(d.x) < 1e-12) { tdx = 1e30; } else { tdx = cfg.params.x / abs(d.x); }
  var tdy: f32; if (abs(d.y) < 1e-12) { tdy = 1e30; } else { tdy = cfg.params.x / abs(d.y); }
  var tdz: f32; if (abs(d.z) < 1e-12) { tdz = 1e30; } else { tdz = cfg.params.x / abs(d.z); }

  var n: vec3<f32> = vec3<f32>(0.0, 0.0, 0.0);

  // Earliest-face normal if the very first cell is occupied.
  if (get_voxel(u32(cell.x), u32(cell.y), u32(cell.z))) {
    if (tmaxx <= tmaxy && tmaxx <= tmaxz) {
      n = select(vec3<f32>(1.0, 0.0, 0.0), vec3<f32>(-1.0, 0.0, 0.0), sx > 0);
    } else if (tmaxy <= tmaxz) {
      n = select(vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(0.0, -1.0, 0.0), sy > 0);
    } else {
      n = select(vec3<f32>(0.0, 0.0, 1.0), vec3<f32>(0.0, 0.0, -1.0), sz > 0);
    }
    if (t0 > tmax || t1 < t0) { hits[i].hit = 0u; return; }
    hits[i].hit = 1u;
    hits[i].t = t;
    hits[i].nx = n.x; hits[i].ny = n.y; hits[i].nz = n.z;
    hits[i].cx = u32(cell.x); hits[i].cy = u32(cell.y); hits[i].cz = u32(cell.z);
    return;
  }

  loop {
    if (cell.x < 0 || cell.y < 0 || cell.z < 0) { hits[i].hit = 0u; return; }
    if (u32(cell.x) >= cfg.dims.x || u32(cell.y) >= cfg.dims.y || u32(cell.z) >= cfg.dims.z) { hits[i].hit = 0u; return; }

    if (get_voxel(u32(cell.x), u32(cell.y), u32(cell.z))) {
      hits[i].hit = 1u;
      hits[i].t = t;
      hits[i].nx = n.x; hits[i].ny = n.y; hits[i].nz = n.z;
      hits[i].cx = u32(cell.x); hits[i].cy = u32(cell.y); hits[i].cz = u32(cell.z);
      return;
    }

    if (tmaxx <= tmaxy && tmaxx <= tmaxz) {
      if (tmaxx > t1 || tmaxx > tmax) { hits[i].hit = 0u; return; }
      t = tmaxx; tmaxx = tmaxx + tdx;
      n = select(vec3<f32>(1.0, 0.0, 0.0), vec3<f32>(-1.0, 0.0, 0.0), sx > 0);
      cell.x = cell.x + sx;
    } else if (tmaxy <= tmaxz) {
      if (tmaxy > t1 || tmaxy > tmax) { hits[i].hit = 0u; return; }
      t = tmaxy; tmaxy = tmaxy + tdy;
      n = select(vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(0.0, -1.0, 0.0), sy > 0);
      cell.y = cell.y + sy;
    } else {
      if (tmaxz > t1 || tmaxz > tmax) { hits[i].hit = 0u; return; }
      t = tmaxz; tmaxz = tmaxz + tdz;
      n = select(vec3<f32>(0.0, 0.0, 1.0), vec3<f32>(0.0, 0.0, -1.0), sz > 0);
      cell.z = cell.z + sz;
    }
  }
}
