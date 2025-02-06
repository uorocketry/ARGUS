use smlang::statemachine;

statemachine! {
    transitions: {
        *Init + Start = Idle,
        Idle | Recovery + WantsCollection = Collection,
        Idle + NoConfig = Calibration,
        Collection + WantsProcessing = Processing,
        Calibration + Configured = Idle,
        Fault + FaultCleared = Idle,
        _ + FaultDetected = Fault,
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use crate::traits;

    #[test]
    pub fn sm_should_transition_between_states_on_event() {
        let mut sm = StateMachine::new(traits::Context {});
    
        // Should be in init by default
        assert!(*sm.state() == States::Init);
    
        // Should transition to idle on start
        _ = sm.process_event(Events::Start).unwrap();
        assert!(*sm.state() == States::Idle);
    
        // Should transition to calibration when there is no config
        _ = sm.process_event(Events::NoConfig).unwrap();
        assert!(*sm.state() == States::Calibration);
    
        // Should transition back to idle when configured
        _ = sm.process_event(Events::Configured).unwrap();
        assert!(*sm.state() == States::Idle);
    
        // Should transition to collection when asked
        _ = sm.process_event(Events::WantsCollection).unwrap();
        assert!(*sm.state() == States::Collection);
    
        // Should transition to processing when asked
        _ = sm.process_event(Events::WantsProcessing).unwrap();
        assert!(*sm.state() == States::Processing);
    }

    #[test]
    pub fn sm_should_handle_fault_in_any_state() {
        let mut sm = StateMachine::new(traits::Context {});
    
        // Should be in init by default
        assert!(*sm.state() == States::Init);
    
        // Should transition to fault on fault detected
        _ = sm.process_event(Events::FaultDetected).unwrap();
        assert!(*sm.state() == States::Fault);
    
        // Should transition to idle on fault cleared
        _ = sm.process_event(Events::FaultCleared).unwrap();
        assert!(*sm.state() == States::Idle);

        // Check fault transition from another state
        _ = sm.process_event(Events::WantsCollection).unwrap();
        _ = sm.process_event(Events::FaultDetected).unwrap();
        assert!(*sm.state() == States::Fault);

        // Should transition to idle on fault cleared
        _ = sm.process_event(Events::FaultCleared).unwrap();
        assert!(*sm.state() == States::Idle);
    }
}
