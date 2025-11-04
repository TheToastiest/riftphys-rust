mod balance;
// pub mod guard;
pub use balance::{BalanceParams, BalanceCtrl};
use riftphys_core::Scalar;

#[derive(Copy, Clone, Debug)]
pub struct GuardParams {
    pub rest_guard:    Scalar,
    pub rest_brace:    Scalar,
    pub k_guard:       Scalar,
    pub k_brace:       Scalar,
    pub brace_time:    Scalar, // seconds to hold brace after hit
}

#[derive(Copy, Clone, Debug)]
pub enum GuardState { Guard, Bracing(Scalar) }

#[derive(Copy, Clone, Debug)]
pub struct GuardCtrl {
    pub params:   GuardParams,
    pub state:    GuardState,
}

impl GuardCtrl {
    pub fn new(params: GuardParams) -> Self {
        Self { params, state: GuardState::Guard }
    }

    pub fn on_contact(&mut self) {
        self.state = GuardState::Bracing(self.params.brace_time);
    }

    /// Advance one tick; returns (rest, compliance)
    pub fn step(&mut self, dt: Scalar) -> (Scalar, Scalar) {
        match self.state {
            GuardState::Guard => (self.params.rest_guard, self.params.k_guard),
            GuardState::Bracing(mut tleft) => {
                tleft = (tleft - dt).max(0.0);
                if tleft == 0.0 {
                    self.state = GuardState::Guard;
                    (self.params.rest_guard, self.params.k_guard)
                } else {
                    self.state = GuardState::Bracing(tleft);
                    (self.params.rest_brace, self.params.k_brace)
                }
            }
        }
    }
}
