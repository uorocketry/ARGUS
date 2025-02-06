use common_arm::HydraError;
use messages::command::RadioRate;
use messages::state::State;
use messages::CanMessage;
use messages::Temperature;
use stm32h7xx_hal::rcc::ResetReason;

#[derive(Clone)]
pub struct DataManager {
    pub state: Option<State>,
    pub reset_reason: Option<ResetReason>,
    pub logging_rate: Option<RadioRate>,
    pub recovery_sensing: Option<CanMessage>,
    pub temperature: Option<[Temperature; 8]>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            state: None,
            reset_reason: None,
            logging_rate: Some(RadioRate::Slow), // start slow.
            recovery_sensing: None,
            temperature: None,
        }
    }

    pub fn get_logging_rate(&mut self) -> RadioRate {
        if let Some(rate) = self.logging_rate.take() {
            let rate_cln = rate.clone();
            self.logging_rate = Some(rate);
            return rate_cln;
        }
        self.logging_rate = Some(RadioRate::Slow);
        RadioRate::Slow
    }

    pub fn set_reset_reason(&mut self, reset: ResetReason) {
        self.reset_reason = Some(reset);
    }

    /// Handle an incomming command from the network.
    pub fn handle_command(&mut self, data: CanMessage) -> Result<(), HydraError> {
        match data.data {
            messages::CanData::Common(common) => match common {
                messages::Common::Command(command) => match command {
                    messages::command::Command::RadioRateChange(rate) => {
                        self.logging_rate = Some(rate.rate);
                    }
                    _ => {}
                },
                _ => {}
            },
            _ => {}
        }

        Ok(())
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
