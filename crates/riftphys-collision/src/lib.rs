use riftphys_geom::Aabb;

/// Deterministic 1D SAP along X with full AABB overlap, NaN-safe and stable.
pub fn pairs_sap(aabbs: &[Aabb]) -> Vec<(usize, usize)> {
    #[derive(Copy, Clone)]
    struct Elem { min: f32, max: f32, idx: usize }

    // Build projections; skip invalid boxes deterministicly
    let mut elems: Vec<Elem> = Vec::with_capacity(aabbs.len());
    for (i, a) in aabbs.iter().enumerate() {
        let mut mn = a.min.x;
        let mut mx = a.max.x;
        if !mn.is_finite() || !mx.is_finite() { continue; }
        if mn > mx { core::mem::swap(&mut mn, &mut mx); }
        elems.push(Elem { min: mn, max: mx, idx: i });
    }

    elems.sort_by(|a,b| a.min.total_cmp(&b.min).then(a.idx.cmp(&b.idx)));

    let mut active: Vec<usize> = Vec::new();
    let mut out: Vec<(usize,usize)> = Vec::new();

    for e in elems {
        active.retain(|&j| aabbs[j].max.x >= e.min);
        for &j in &active {
            let (i, k) = if j < e.idx { (j, e.idx) } else { (e.idx, j) };
            let aa = &aabbs[i]; let bb = &aabbs[k];
            if aa.overlaps(bb) { out.push((i,k)); }
        }
        active.push(e.idx);
    }

    out.sort_unstable();
    out
}
