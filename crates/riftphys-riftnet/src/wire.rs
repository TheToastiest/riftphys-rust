// add at top if not present
use bytemuck::{Pod, Zeroable};

pub const MAGIC: u32 = 0x3053_5052;
pub const VER:   u16 = 2;                 // bump to v2
pub const KIND_SNAPSHOT: u8 = 1;

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct NetHdr {
    pub magic: u32,
    pub ver:   u16,
    pub kind:  u8,
    pub flags: u8,
    pub tick:  u64,
    pub epoch: u64,
    pub total: u16,
    pub index: u16,
    pub payload_len: u32,
}

/// kind: 0=Unknown, 1=Sphere, 2=Box, 3=CapsuleY
/// For CapsuleY we encode scale as (sx=r, sy=(hh+r)/2, sz=r) so the viewer can reconstruct hh.
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct NetEnt {
    pub id: u32,
    pub px: f32, pub py: f32, pub pz: f32,
    pub qx: f32, pub qy: f32, pub qz: f32, pub qw: f32,
    pub sx: f32, pub sy: f32, pub sz: f32,
    pub kind: u32,
}

pub const MAX_PAYLOAD: usize = 1200;

pub fn pack_snapshot(tick: u64, epoch: u64, ents: &[NetEnt]) -> Vec<Vec<u8>> {
    let ent_sz = core::mem::size_of::<NetEnt>();
    let per = (MAX_PAYLOAD / ent_sz).max(1);
    let total = ((ents.len() + per - 1) / per) as u16;

    let mut out = Vec::with_capacity(total as usize);
    for (i, chunk) in ents.chunks(per).enumerate() {
        let payload_len = (chunk.len() * ent_sz) as u32;
        let hdr = NetHdr {
            magic: MAGIC, ver: VER, kind: KIND_SNAPSHOT, flags: 0,
            tick, epoch, total, index: i as u16, payload_len,
        };
        let mut buf = Vec::with_capacity(core::mem::size_of::<NetHdr>() + payload_len as usize);
        buf.extend_from_slice(bytemuck::bytes_of(&hdr));
        buf.extend_from_slice(bytemuck::cast_slice(chunk));
        out.push(buf);
    }
    out
}

pub fn parse_chunk(buf: &[u8]) -> Option<(NetHdr, &[NetEnt])> {
    if buf.len() < core::mem::size_of::<NetHdr>() { return None; }
    let (hbytes, pbytes) = buf.split_at(core::mem::size_of::<NetHdr>());
    let hdr: NetHdr = *bytemuck::from_bytes(hbytes);
    if hdr.magic != MAGIC || hdr.kind != KIND_SNAPSHOT { return None; }
    if !(hdr.ver == 2) { return None; } // both sides move together
    if (hdr.payload_len as usize) != pbytes.len() { return None; }
    let ents: &[NetEnt] = bytemuck::try_cast_slice(pbytes).ok()?;
    Some((hdr, ents))
}
