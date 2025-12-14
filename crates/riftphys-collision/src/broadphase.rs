// riftphys-collision/src/broadphase.rs
use riftphys_geom::Aabb;

/// Deterministic 1D SAP along X with full AABB overlap, NaN-safe and stable.
/// Returns sorted (i,j) pairs with i < j.
pub fn pairs_sap(aabbs: &[Aabb]) -> Vec<(usize, usize)> {
    #[derive(Copy, Clone)]
    struct Elem {
        min_x: f32,
        max_x: f32,
        idx:   usize,
    }

    // Build projections; skip invalid boxes deterministically.
    let mut elems: Vec<Elem> = Vec::with_capacity(aabbs.len());
    for (i, a) in aabbs.iter().enumerate() {
        let mut mn = a.min.x;
        let mut mx = a.max.x;

        // Deterministic invalid cull.
        if !mn.is_finite() || !mx.is_finite() {
            continue;
        }
        if mn > mx {
            core::mem::swap(&mut mn, &mut mx);
        }

        elems.push(Elem { min_x: mn, max_x: mx, idx: i });
    }

    // Sort by min, then by index for stability.
    elems.sort_by(|a, b| a.min_x.total_cmp(&b.min_x).then(a.idx.cmp(&b.idx)));

    let mut active: Vec<Elem> = Vec::new();
    let mut out: Vec<(usize, usize)> = Vec::new();

    for e in elems {
        // Drop anything whose max.x < current min.x (using canonical max_x).
        active.retain(|a| a.max_x >= e.min_x);

        // Test against remaining active set.
        for a in &active {
            let (i, k) = if a.idx < e.idx { (a.idx, e.idx) } else { (e.idx, a.idx) };
            let aa = &aabbs[i];
            let bb = &aabbs[k];

            // overlaps() should be NaN-safe by construction; if not, NaNs will deterministically return false.
            if aa.overlaps(bb) {
                out.push((i, k));
            }
        }

        active.push(e);
    }

    // Stable sorted pair list for deterministic narrow-phase batching.
    out.sort_unstable();
    out
}
