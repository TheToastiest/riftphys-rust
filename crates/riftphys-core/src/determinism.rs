#[derive(Copy, Clone, Debug)]
pub struct DeterminismContract {
    pub fixed_dt: f32,
    pub float: &'static str,
    pub fma: bool,
    pub iterations: u32,
    pub stable_sorts: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct Units {
    pub length: &'static str,
    pub mass:   &'static str,
    pub time:   &'static str,
}

impl DeterminismContract {
    pub fn default_contract() -> Self {
        Self {
            fixed_dt: 1.0/60.0,
            float: "f32",
            fma: false,
            iterations: 8,
            stable_sorts: true,
        }
    }
}
