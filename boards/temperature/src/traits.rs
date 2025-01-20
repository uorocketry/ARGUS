//! This module should be refactored out.
use crate::States;
use messages::state::DeviceState;

use crate::StateMachineContext;

pub struct Context {}

impl StateMachineContext for Context {}

impl From<States> for DeviceState {
    fn from(value: States) -> Self {
        match value {
            States::Idle => DeviceState::Idle,
            States::Calibration => DeviceState::Calibration,
            States::Recovery => DeviceState::Recovery,
            States::Collection => DeviceState::Collection,
            States::Init => DeviceState::Init,
            States::Processing => DeviceState::Processing,
            States::Fault => DeviceState::Fault,
        }
    }
}
