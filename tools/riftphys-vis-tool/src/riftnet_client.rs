// tools/riftphys-vis-tool/src/riftnet_client.rs
use riftphys_riftnet::{self as rnet, sys};
use riftphys_riftnet::wire::{parse_chunk, NetEnt, NetHdr, MAGIC, VER, KIND_SNAPSHOT};
use std::{ffi::{c_void, CString}, sync::Arc};
use parking_lot::Mutex;
use std::collections::BTreeMap;

pub struct NetViewer {
    h: sys::RiftClientHandle,
    pending: Arc<Mutex<BTreeMap<(u64,u64), Pending>>>,
    out: Arc<Mutex<Vec<NetEnt>>>,   // latest complete snapshot (overwrite each time)
}
struct Pending { total:u16, got:u16, buf: Vec<u8> }

struct Ctx {
    pending: Arc<Mutex<BTreeMap<(u64,u64), Pending>>>,
    out: Arc<Mutex<Vec<NetEnt>>>,
}

unsafe extern "C" fn on_event(ev: *const sys::RiftEvent, user: *mut c_void) {
    if ev.is_null() { return; }
    let ev = &*ev;
    if let sys::RiftEventType::RIFT_EVENT_PACKET_RECEIVED = ev.r#type {
        let pkt = ev.data.packet;
        let data = std::slice::from_raw_parts(pkt.data, pkt.size);
        if let Some((hdr, ents)) = parse_chunk(data) {
            let ctx = &*(user as *const Ctx);
            let key = (hdr.epoch, hdr.tick);
            let mut map = ctx.pending.lock();
            let entry = map.entry(key).or_insert_with(|| Pending {
                total: hdr.total, got: 0, buf: Vec::with_capacity((ents.len() * std::mem::size_of::<NetEnt>()) * (hdr.total as usize) + 64)
            });
            // append this chunk's raw payload
            entry.buf.extend_from_slice(bytemuck::cast_slice(ents));
            entry.got += 1;

            if entry.got == entry.total {
                // completed snapshot → publish
                let n = entry.buf.len() / std::mem::size_of::<NetEnt>();
                let slice = unsafe { std::slice::from_raw_parts(entry.buf.as_ptr() as *const NetEnt, n) };
                let mut out = ctx.out.lock();
                out.clear();
                out.extend_from_slice(slice);
                map.remove(&key);
            }
        }
    }
}

impl NetViewer {
    pub fn connect(addr: &str, port: u16, out: Arc<Mutex<Vec<NetEnt>>>) -> Option<Self> {
        unsafe {
            let pending = Arc::new(Mutex::new(BTreeMap::new()));
            let ctx = Box::new(Ctx { pending: pending.clone(), out: out.clone() });
            let cfg = sys::RiftClientConfig {
                event_callback: Some(on_event),
                user_data: Box::into_raw(ctx) as *mut c_void,
            };
            let h = sys::rift_client_create(&cfg);
            if h.is_null() { return None; }
            let host = CString::new(addr).ok()?;
            if matches!(sys::rift_client_connect(h, host.as_ptr(), port), sys::RiftResult::RIFT_SUCCESS) {
                Some(Self { h, pending, out })
            } else {
                sys::rift_client_destroy(h);
                None
            }
        }
    }
    pub fn poll(&self) {
        unsafe { let _ = sys::rift_client_poll(self.h, 0); }
    }
    pub fn disconnect(&self) { unsafe { sys::rift_client_disconnect(self.h); } }
}

impl Drop for NetViewer {
    fn drop(&mut self) {
        unsafe {
            self.disconnect();
            sys::rift_client_destroy(self.h);
            // reclaim the Ctx we heap-allocated
            let raw = {
                let pmap = self.pending.clone();
                let out  = self.out.clone();
                // reconstruct the same Ctx layout
                Box::new(Ctx { pending: pmap, out })
            };
            let _ = raw; // nothing to do; we leaked Box in connect intentionally; safe on process exit
        }
    }
}
