use ads126x::{
    register::{DataRate, Mode1Register, Mode2Register},
    ADCCommand, Ads126x,
};

use common_arm::spawn;
use stm32h7xx_hal::{
    gpio::{Output, Pin, PushPull},
    spi::Spi,
};

use crate::app::delay;

// There is an option to use interrupts using the data ready pins, but for now we will poll.
pub struct AdcManager {
    pub spi: Spi<stm32h7xx_hal::pac::SPI4, stm32h7xx_hal::spi::Enabled, u8>,
    pub adc1: Ads126x<Pin<'C', 11, Output<PushPull>>>,
    pub adc2: Ads126x<Pin<'E', 0, Output<PushPull>>>,
    pub adc1_cs: Pin<'C', 10, Output<PushPull>>,
    pub adc2_cs: Pin<'D', 2, Output<PushPull>>,
}

impl AdcManager {
    pub fn new(
        spi: Spi<stm32h7xx_hal::pac::SPI4, stm32h7xx_hal::spi::Enabled, u8>,
        adc1_rst: Pin<'C', 11, Output<PushPull>>,
        adc2_rst: Pin<'E', 0, Output<PushPull>>,
        adc1_cs: Pin<'C', 10, Output<PushPull>>,
        adc2_cs: Pin<'D', 2, Output<PushPull>>,
    ) -> Self {
        Self {
            spi,
            adc1: Ads126x::new(adc1_rst),
            adc2: Ads126x::new(adc2_rst),
            adc1_cs,
            adc2_cs,
        }
    }

    pub fn init_adc1(&mut self) -> Result<(), ads126x::error::ADS126xError> {
        self.adc2_cs.set_high();
        self.adc1_cs.set_low();
        self.adc1.set_reset_high()?;

        match spawn!(delay, 1000) {
            Ok(_) => (),
            Err(_) => panic!("Failed ADC 1 init."),
        }

        let mut mode1_cfg = Mode1Register::default();
        mode1_cfg.set_filter(ads126x::register::DigitalFilter::Sinc1);
        self.adc1.set_mode1(&mode1_cfg, &mut self.spi)?;

        let mut mode2_cfg = Mode2Register::default();
        mode2_cfg.set_dr(DataRate::SPS1200);
        self.adc1.set_mode2(&mode2_cfg, &mut self.spi)?;

        self.adc1.send_command(ADCCommand::START1, &mut self.spi)?;
        self.adc1.send_command(ADCCommand::START2, &mut self.spi)?;

        Ok(())
    }

    pub fn init_adc2(&mut self) -> Result<(), ads126x::error::ADS126xError> {
        self.adc1_cs.set_high();
        self.adc2_cs.set_low();
        self.adc2.set_reset_high()?;

        match spawn!(delay, 1000) {
            Ok(_) => (),
            Err(_) => panic!("Failed ADC 2 init."),
        }

        let mut mode1_cfg = Mode1Register::default();
        mode1_cfg.set_filter(ads126x::register::DigitalFilter::Sinc1);
        self.adc1.set_mode1(&mode1_cfg, &mut self.spi)?;

        let mut mode2_cfg = Mode2Register::default();
        mode2_cfg.set_dr(DataRate::SPS1200);
        self.adc1.set_mode2(&mode2_cfg, &mut self.spi)?;

        self.adc1.send_command(ADCCommand::START1, &mut self.spi)?;
        self.adc1.send_command(ADCCommand::START2, &mut self.spi)?;

        Ok(())
    }
}
