use riftphys_core::{StepStage, schedule_digest, Vec3};

/* ---------------------- Schedule recorder ---------------------- */
#[derive(Default)]
pub struct ScheduleRecorder {
    stages: Vec<StepStage>,
}
impl ScheduleRecorder {
    pub fn new() -> Self { Self { stages: Vec::new() } }
    pub fn push(&mut self, s: StepStage) { self.stages.push(s); }
    pub fn clear(&mut self) { self.stages.clear(); }
    pub fn digest(&self) -> [u8; 32] { schedule_digest(&self.stages) }
}

/* ---------------------- Debug settings ---------------------- */
#[derive(Copy, Clone, Debug)]
pub struct DebugSettings {
    /// Print a summary every N ticks (0 = disabled)
    pub print_every: u32,
    pub show_bodies: bool,
    pub show_contacts: bool,
    pub show_impulses: bool, // world-side printer may ignore until wired
    pub show_energy: bool,
    pub max_lines: usize,    // clamp output lines
    pub json_every: u32,
}
impl Default for DebugSettings {
    fn default() -> Self {
        Self {
            print_every: 0,
            show_bodies: false,
            show_contacts: false,
            show_impulses: false,
            show_energy: false,
            max_lines: 200,
            json_every: 0,
        }
    }
}

/* ---------------------- Telemetry events ---------------------- */
#[derive(Copy, Clone, Debug)]
pub enum LedgerEvent {
    // Integration
    Integrate { id: u32, a: Vec3, dv: Vec3 },

    // Contacts detected in narrowphase
    ContactBegin { a_collider: usize, b_collider: usize, normal: Vec3, depth: f32 },

    // Solver
    ImpulseN { a: u32, b: u32, jn: f32 },
    ImpulseT { a: u32, b: u32, jt1: f32, jt2: f32 },
    PosCorr  { a: u32, b: u32, corr: f32 },

    // Joints (XPBD distance)
    JointDistance { a: u32, b: u32, lambda: f32, compliance: f32 },

    // CCD
    CCDHit { id: u32, toi: f32 },

    AeroProp { id: u32, t_n: f32, d_n: f32, speed: f32 },
    // riftphys-viz/src/telemetry.rs
    VehicleWheel { body: u32, wheel: u32, x: f32, f_susp: f32, f_long: f32, f_lat: f32 },
    BalanceAccel { id: u32, ax: f32, az: f32 },

}

impl LedgerEvent {
    pub fn to_json(&self) -> String {
        match *self {
            LedgerEvent::Integrate { id, a, dv } =>
                format!(r#"{{"t":"I","id":{},"ax":{:.6},"ay":{:.6},"az":{:.6},"dvx":{:.6},"dvy":{:.6},"dvz":{:.6}}}"#,
                        id,a.x,a.y,a.z,dv.x,dv.y,dv.z),
            LedgerEvent::ContactBegin { a_collider, b_collider, normal, depth } =>
                format!(r#"{{"t":"C","a":{},"b":{},"nx":{:.6},"ny":{:.6},"nz":{:.6},"d":{:.6}}}"#,
                        a_collider,b_collider,normal.x,normal.y,normal.z,depth),
            LedgerEvent::ImpulseN { a, b, jn } =>
                format!(r#"{{"t":"N","a":{},"b":{},"jn":{:.6}}}"#, a,b,jn),
            LedgerEvent::ImpulseT { a, b, jt1, jt2 } =>
                format!(r#"{{"t":"T","a":{},"b":{},"jt1":{:.6},"jt2":{:.6}}}"#, a,b,jt1,jt2),
            LedgerEvent::PosCorr { a, b, corr } =>
                format!(r#"{{"t":"P","a":{},"b":{},"corr":{:.6}}}"#, a,b,corr),
            LedgerEvent::JointDistance { a, b, lambda, compliance } =>
                format!(r#"{{"t":"J","a":{},"b":{},"lam":{:.6},"c":{:.6}}}"#, a,b,lambda,compliance),
            LedgerEvent::CCDHit { id, toi } =>
                format!(r#"{{"t":"X","id":{},"toi":{:.6}}}"#, id, toi),
            LedgerEvent::AeroProp { id, t_n, d_n, speed } =>
                format!(r#"{{"t":"A","id":{},"tN":{:.6},"dN":{:.6},"speed":{:.6}}}"#, id, t_n, d_n, speed),
            LedgerEvent::VehicleWheel { body, wheel, x, f_susp, f_long, f_lat } =>
                format!(r#"{{"t":"VW","id":{},"w":{},"x":{:.6},"fs":{:.6},"fx":{:.6},"fy":{:.6}}}"#,
                        body, wheel, x, f_susp, f_long, f_lat),
            LedgerEvent::BalanceAccel { id, ax, az } =>
                format!(r#"{{"t":"BA","id":{},"ax":{:.6},"az":{:.6}}}"#, id, ax, az),
        }
    }
}

/* ---------------------- Ledger buffer ---------------------- */
pub struct Ledger {
    events: Vec<LedgerEvent>,
    cap: usize,
}

impl Ledger {
    /// Create a ledger with capacity `cap` events per tick.
    pub fn new(cap: usize) -> Self {
        Self { events: Vec::with_capacity(cap), cap }
    }
    /// Remove all events for the next tick.
    pub fn clear(&mut self) { self.events.clear(); }
    /// Append one event (drops if over cap).
    pub fn push(&mut self, e: LedgerEvent) {
        if self.events.len() < self.cap {
            self.events.push(e);
        }
    }
    /// Current number of events.
    pub fn len(&self) -> usize { self.events.len() }
    /// Iterate events (handy for printers).
    pub fn iter(&self) -> impl Iterator<Item = &LedgerEvent> { self.events.iter() }

    /// Write events as JSONL to `dir/telemetry_######.jsonl` (deterministic order).
    pub fn write_jsonl(&self, dir: &str, tick: u64) -> std::io::Result<()> {
        use std::fs::{create_dir_all, OpenOptions};
        use std::io::Write;

        create_dir_all(dir)?;
        let path = format!("{}/telemetry_{:06}.jsonl", dir, tick);
        let mut f = OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(path)?;
        for e in self.events.iter() {
            let line = e.to_json();
            f.write_all(line.as_bytes())?;
            f.write_all(b"\n")?;
        }
        Ok(())
    }
}
