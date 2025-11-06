use riftphys_riftnet::*;
use std::{ffi::CString, mem::size_of, ptr};
const MAGIC: u32 = 0x3053_5052; // "RPS0"
const VER: u16 = 1;
const KIND_SNAP: u8 = 1;
const MAX_PAYLOAD: usize = 1200; // conservative UDP payload

#[repr(C, packed)]
pub struct NetHdr {
    pub magic: u32, pub ver: u16, pub kind: u8, pub flags: u8,
    pub tick: u64,  pub epoch: u64, pub total: u16, pub index: u16, pub payload_len: u32,
}
#[repr(C, packed)]
pub struct NetEnt { pub id: u32, pub px:f32, pub py:f32, pub pz:f32, pub qx:f32, pub qy:f32, pub qz:f32, pub qw:f32 }

pub struct NetServer {
    h: RiftServerHandle,
    buf: Vec<u8>,
}
unsafe extern "C" fn on_event(_ev: *const RiftEvent, _user: *mut std::ffi::c_void) {
    // Optional: log connects/disconnects; not needed for broadcast
}
impl NetServer {
    pub fn start(host: &str, port: u16) -> Option<Self> {
        unsafe {
            let host_c = CString::new(host).ok()?;
            let cfg = RiftServerConfig {
                host_address: host_c.as_ptr(),
                port,
                event_callback: Some(on_event),
                user_data: ptr::null_mut(),
            };
            // Keep host_c alive until after create()
            let h = rift_server_create(&cfg);
            if h.is_null() { return None; }
            if matches!(rift_server_start(h), RiftResult::RIFT_SUCCESS) {
                Some(Self { h, buf: Vec::with_capacity(4096) })
            } else {
                rift_server_destroy(h);
                None
            }
        }
    }
    pub fn stop(&mut self) {
        unsafe { rift_server_stop(self.h); rift_server_destroy(self.h); }
    }

    /// Broadcasts a snapshot by chunking entities into ~MTU slices
    pub fn broadcast_snapshot(&mut self, tick: u64, epoch: u64, ents: &[NetEnt]) {
        let per = (MAX_PAYLOAD / std::mem::size_of::<NetEnt>()).max(1);
        let total = ((ents.len() + per - 1) / per) as u16;
        for (i, chunk) in ents.chunks(per).enumerate() {
            let mut hdr = NetHdr {
                magic: MAGIC, ver: VER, kind: KIND_SNAP, flags: 0,
                tick, epoch, total, index: i as u16, payload_len: (chunk.len() * size_of::<NetEnt>()) as u32,
            };
            self.buf.clear();
            // Safety: packed POD -> bytes
            let hdr_bytes = unsafe {
                std::slice::from_raw_parts((&hdr as *const _) as *const u8, size_of::<NetHdr>())
            };
            self.buf.extend_from_slice(hdr_bytes);
            let chunk_bytes = unsafe {
                std::slice::from_raw_parts(chunk.as_ptr() as *const u8, chunk.len()*size_of::<NetEnt>())
            };
            self.buf.extend_from_slice(chunk_bytes);
            unsafe {
                let _ = rift_server_broadcast(self.h, self.buf.as_ptr(), self.buf.len());
            }
        }
    }
}
