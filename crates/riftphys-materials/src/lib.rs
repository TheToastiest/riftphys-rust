#[derive(Copy,Clone,Debug,Eq,PartialEq,Hash)]
pub enum MaterialId { Default, Ice, Rubber, Steel, Grit }

#[derive(Copy,Clone,Debug)]
pub struct MatProps { pub mu_s:f32, pub mu_k:f32, pub restitution:f32 }

pub fn props(id: MaterialId) -> MatProps {
    match id {
        MaterialId::Default => MatProps{mu_s:0.5, mu_k:0.4, restitution:0.0},
        MaterialId::Ice     => MatProps{mu_s:0.02,mu_k:0.01,restitution:0.02},
        MaterialId::Rubber  => MatProps{mu_s:1.0, mu_k:0.9, restitution:0.8},
        MaterialId::Steel   => MatProps{mu_s:0.6, mu_k:0.5, restitution:0.1},
        MaterialId::Grit    => MatProps{mu_s:1.2, mu_k:1.0, restitution:0.0},

    }
}

/// Deterministic pairwise combine (override table first; else geometric mean)
pub fn combine(a:MaterialId,b:MaterialId)->MatProps{
    use MaterialId::*;
    // Example override: Ice×Rubber reduces restitution a bit more
    if (a==Ice && b==Rubber) || (a==Rubber && b==Ice) {
        return MatProps{mu_s: (props(a).mu_s*props(b).mu_s).sqrt(),
            mu_k: (props(a).mu_k*props(b).mu_k).sqrt(),
            restitution: 0.05};
    }
    let pa=props(a); let pb=props(b);
    MatProps{
        mu_s:(pa.mu_s*pb.mu_s).sqrt(),
        mu_k:(pa.mu_k*pb.mu_k).sqrt(),
        restitution: pa.restitution.max(pb.restitution),
    }
}
