// crates/riftphys-riftnet/src/sys.rs
// MIT – runtime loader for RiftNet.dll matching riftnet_c.h
#![allow(non_camel_case_types, non_snake_case, dead_code)]

use libloading::{Library, Symbol};
use once_cell::sync::OnceCell;
use std::{
    ffi::{c_void, CString},
    os::raw::{c_char, c_uchar, c_uint},
    path::PathBuf,
};

pub type size_t = usize;
pub type RiftClientId = u64;

/* ---------- ABI version ---------- */
pub const RIFTNET_ABI_VERSION: u32 = 0x0001_0000;

/* ---------- Basic structs/types (repr C) ---------- */
#[repr(C)]
#[derive(Copy, Clone)]
pub struct RiftPacket {
    pub data: *const c_uchar,
    pub size: size_t,
    pub sender_id: RiftClientId, // 0 on client
}

#[repr(C)]
#[derive(Copy, Clone)]
pub enum RiftEventType {
    RIFT_EVENT_SERVER_START = 0,
    RIFT_EVENT_SERVER_STOP = 1,
    RIFT_EVENT_CLIENT_CONNECTED = 2,
    RIFT_EVENT_CLIENT_DISCONNECTED = 3,
    RIFT_EVENT_PACKET_RECEIVED = 4,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub union RiftEventData {
    pub packet: RiftPacket,
    pub client_id: RiftClientId,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RiftEvent {
    pub r#type: RiftEventType,
    pub data: RiftEventData,
}

pub type RiftEventCallback =
Option<unsafe extern "C" fn(event: *const RiftEvent, user_data: *mut c_void)>;

#[repr(C)]
pub struct RiftServerConfig {
    pub host_address: *const c_char,
    pub port: u16,
    pub event_callback: RiftEventCallback,
    pub user_data: *mut c_void,
}

#[repr(C)]
pub struct RiftClientConfig {
    pub event_callback: RiftEventCallback,
    pub user_data: *mut c_void,
}

// Opaque handles
#[repr(C)]
pub struct RiftServer {
    _priv: [u8; 0],
}
#[repr(C)]
pub struct RiftClient {
    _priv: [u8; 0],
}
pub type RiftServerHandle = *mut RiftServer;
pub type RiftClientHandle = *mut RiftClient;

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RiftResult {
    RIFT_SUCCESS = 0,
    RIFT_ERROR_GENERIC = -1,
    RIFT_ERROR_INVALID_HANDLE = -2,
    RIFT_ERROR_INVALID_PARAMETER = -3,
    RIFT_ERROR_SOCKET_CREATION_FAILED = -4,
    RIFT_ERROR_SOCKET_BIND_FAILED = -5,
    RIFT_ERROR_CONNECTION_FAILED = -6,
    RIFT_ERROR_SEND_FAILED = -7,
    RIFT_ERROR_IOCP_CREATION_FAILED = -8,
}

/* ---------- Runtime loader helpers ---------- */

fn dll_path() -> PathBuf {
    // 1) Explicit env override (absolute or relative)
    if let Some(val) = std::env::var_os("RIFTNET_DLL") {
        return PathBuf::from(val);
    }
    // 2) Default: <exe_dir>\RiftNet.dll
    let mut p = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    if let Ok(exec) = std::env::current_exe() {
        if let Some(parent) = exec.parent() {
            p = parent.to_path_buf();
        }
    }
    p.push("RiftNet.dll");
    p
}

unsafe fn load_sym<T: Copy>(lib: &Library, name: &str) -> T {
    // ensure NUL-terminated symbol name
    let c = CString::new(name).expect("symbol name");
    let sym: Symbol<T> = lib
        .get(c.as_bytes_with_nul())
        .unwrap_or_else(|_| panic!("riftnet: missing symbol `{}` in DLL", name));
    *sym // copy function pointer (T: Copy)
}

/* ---------- Function pointer table ---------- */
struct Api {
    rift_get_abi_version: unsafe extern "C" fn() -> c_uint,

    rift_client_create: unsafe extern "C" fn(*const RiftClientConfig) -> RiftClientHandle,
    rift_client_destroy: unsafe extern "C" fn(RiftClientHandle),
    rift_client_connect:
        unsafe extern "C" fn(RiftClientHandle, *const c_char, u16) -> RiftResult,
    rift_client_disconnect: unsafe extern "C" fn(RiftClientHandle),
    rift_client_send:
        unsafe extern "C" fn(RiftClientHandle, *const c_uchar, size_t) -> RiftResult,
    rift_client_poll: unsafe extern "C" fn(RiftClientHandle, c_uint) -> RiftResult,

    rift_server_create: unsafe extern "C" fn(*const RiftServerConfig) -> RiftServerHandle,
    rift_server_destroy: unsafe extern "C" fn(RiftServerHandle),
    rift_server_start: unsafe extern "C" fn(RiftServerHandle) -> RiftResult,
    rift_server_stop: unsafe extern "C" fn(RiftServerHandle),
    rift_server_send: unsafe extern "C" fn(
        RiftServerHandle,
        RiftClientId,
        *const c_uchar,
        size_t,
    ) -> RiftResult,
    rift_server_broadcast:
        unsafe extern "C" fn(RiftServerHandle, *const c_uchar, size_t) -> RiftResult,
}

static LIB: OnceCell<Library> = OnceCell::new();
static API: OnceCell<Api> = OnceCell::new();

fn api() -> &'static Api {
    API.get_or_init(|| unsafe {
        let path = dll_path();
        let lib = LIB.get_or_init(|| {
            Library::new(&path)
                .unwrap_or_else(|e| panic!("riftnet: Load {} failed: {e:?}", path.display()))
        });

        let a = Api {
            rift_get_abi_version: load_sym(lib, "rift_get_abi_version"),

            rift_client_create: load_sym(lib, "rift_client_create"),
            rift_client_destroy: load_sym(lib, "rift_client_destroy"),
            rift_client_connect: load_sym(lib, "rift_client_connect"),
            rift_client_disconnect: load_sym(lib, "rift_client_disconnect"),
            rift_client_send: load_sym(lib, "rift_client_send"),
            rift_client_poll: load_sym(lib, "rift_client_poll"),

            rift_server_create: load_sym(lib, "rift_server_create"),
            rift_server_destroy: load_sym(lib, "rift_server_destroy"),
            rift_server_start: load_sym(lib, "rift_server_start"),
            rift_server_stop: load_sym(lib, "rift_server_stop"),
            rift_server_send: load_sym(lib, "rift_server_send"),
            rift_server_broadcast: load_sym(lib, "rift_server_broadcast"),
        };

        let ver = (a.rift_get_abi_version)();
        assert_eq!(
            ver as u32,
            RIFTNET_ABI_VERSION,
            "riftnet: ABI mismatch: got {ver:#x}, need {RIFTNET_ABI_VERSION:#x}"
        );
        a
    })
}

/* ---------- Public wrappers (call these) ---------- */

pub unsafe fn rift_get_abi_version() -> c_uint {
    (api().rift_get_abi_version)()
}

pub unsafe fn rift_client_create(cfg: *const RiftClientConfig) -> RiftClientHandle {
    (api().rift_client_create)(cfg)
}
pub unsafe fn rift_client_destroy(h: RiftClientHandle) {
    (api().rift_client_destroy)(h)
}
pub unsafe fn rift_client_connect(
    h: RiftClientHandle,
    host: *const c_char,
    port: u16,
) -> RiftResult {
    (api().rift_client_connect)(h, host, port)
}
pub unsafe fn rift_client_disconnect(h: RiftClientHandle) {
    (api().rift_client_disconnect)(h)
}
pub unsafe fn rift_client_send(
    h: RiftClientHandle,
    data: *const c_uchar,
    size: size_t,
) -> RiftResult {
    (api().rift_client_send)(h, data, size)
}
pub unsafe fn rift_client_poll(h: RiftClientHandle, max_millis: c_uint) -> RiftResult {
    (api().rift_client_poll)(h, max_millis)
}

pub unsafe fn rift_server_create(cfg: *const RiftServerConfig) -> RiftServerHandle {
    (api().rift_server_create)(cfg)
}
pub unsafe fn rift_server_destroy(h: RiftServerHandle) {
    (api().rift_server_destroy)(h)
}
pub unsafe fn rift_server_start(h: RiftServerHandle) -> RiftResult {
    (api().rift_server_start)(h)
}
pub unsafe fn rift_server_stop(h: RiftServerHandle) {
    (api().rift_server_stop)(h)
}
pub unsafe fn rift_server_send(
    h: RiftServerHandle,
    cid: RiftClientId,
    data: *const c_uchar,
    size: size_t,
) -> RiftResult {
    (api().rift_server_send)(h, cid, data, size)
}
pub unsafe fn rift_server_broadcast(
    h: RiftServerHandle,
    data: *const c_uchar,
    size: size_t,
) -> RiftResult {
    (api().rift_server_broadcast)(h, data, size)
}
