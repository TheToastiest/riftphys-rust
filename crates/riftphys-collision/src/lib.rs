use riftphys_geom::Aabb;

/// Deterministic 1D SAP along X with full AABB test in YZ, stable pair order.
pub fn pairs_sap(aabbs: &[Aabb]) -> Vec<(usize, usize)> {
    #[derive(Copy, Clone)]
    struct Elem { min: f32, max: f32, idx: usize }
    let mut elems: Vec<Elem> = aabbs.iter().enumerate()
        .map(|(i,a)| Elem { min: a.min.x, max: a.max.x, idx: i }).collect();
    elems.sort_by(|a,b| a.min.partial_cmp(&b.min).unwrap().then(a.idx.cmp(&b.idx)));

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
    out.sort(); // stable, deterministic order
    out
}
