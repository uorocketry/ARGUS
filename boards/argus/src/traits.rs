//! This module should be refactored out.
use crate::States;
use messages::state::State;

use crate::StateMachineContext;

pub struct Context {}

impl StateMachineContext for Context {}

impl From<States> for State {
    fn from(value: States) -> Self {
        match value {
            States::Idle => State::Idle,
            States::Calibration => State::Calibration,
            States::Recovery => State::Recovery,
            States::Collection => State::Collection,
            States::Initializing => State::Initializing,
            States::Processing => State::Processing,
            States::Fault => State::Fault,
        }
    }
}
