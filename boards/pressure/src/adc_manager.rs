use ads126x::{
    register::{DataRate, Mode1Register, Mode2Register},
    ADCCommand, Ads126x,
};

use crate::app::delay;
use common_arm::spawn;
use defmt::info;
use stm32h7xx_hal::nb;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::{
    gpio::{Output, Pin, PushPull},
    spi::Spi,
};

// There is an option to use interrupts using the data ready pins, but for now we will poll.
pub struct AdcManager {
    pub spi: Spi<stm32h7xx_hal::pac::SPI4, stm32h7xx_hal::spi::Enabled, u8>,
    pub adc1: Ads126x<Pin<'C', 11, Output<PushPull>>>,
    pub adc2: Ads126x<Pin<'D', 1, Output<PushPull>>>,
    pub adc1_cs: Pin<'C', 10, Output<PushPull>>,
    pub adc2_cs: Pin<'D', 2, Output<PushPull>>,
}

impl AdcManager {
    pub fn new(
        spi: Spi<stm32h7xx_hal::pac::SPI4, stm32h7xx_hal::spi::Enabled, u8>,
        adc1_rst: Pin<'C', 11, Output<PushPull>>,
        adc2_rst: Pin<'D', 1, Output<PushPull>>,
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
        self.select_adc1();
        self.adc1.set_reset_high()?;

        // 2^16 cycles of delay
        cortex_m::asm::delay(65536);

        // stop conversions
        self.adc1.send_command(ADCCommand::STOP1, &mut self.spi)?;
        self.adc1.send_command(ADCCommand::STOP2, &mut self.spi)?;

        // setup the Power register
        let mut power_cfg = ads126x::register::PowerRegister::default();
        power_cfg.clear_reset();
        self.adc1.set_power(&power_cfg, &mut self.spi)?;

        // Verify none custom config works first
        // setup mode 1 and mode 2 registers
        // let mut mode1_cfg = Mode1Register::default();
        // mode1_cfg.set_filter(ads126x::register::DigitalFilter::Sinc1);
        // self.adc1.set_mode1(&mode1_cfg, &mut self.spi)?;

        // let mut mode2_cfg = Mode2Register::default();
        // mode2_cfg.set_dr(DataRate::SPS1200);
        // self.adc1.set_mode2(&mode2_cfg, &mut self.spi)?;

        // // read back the mode1 and mode2 registers to verify
        // let mode1_cfg_real = self.adc1.get_mode1(&mut self.spi)?;
        // let mode2_cfg_real = self.adc1.get_mode2(&mut self.spi)?;

        // // verify
        // info!("Mode1: {:#010b}", mode1_cfg_real.bits());
        // info!("Mode2: {:#010b}", mode2_cfg_real.bits());
        // assert!(mode1_cfg.difference(mode1_cfg_real).is_empty());
        // assert!(mode2_cfg.difference(mode2_cfg_real).is_empty());

        // start conversions
        self.adc1.send_command(ADCCommand::START1, &mut self.spi)?;
        self.adc1.send_command(ADCCommand::START2, &mut self.spi)?;

        Ok(())
    }

    // pub fn init_adc2(&mut self) -> Result<(), ads126x::error::ADS126xError> {
    //     self.select_adc2();
    //     self.adc2.set_reset_high()?;
    //     info!("ADC2 reset");

    //     cortex_m::asm::delay(65536);

    //     let mut mode1_cfg = Mode1Register::default();
    //     mode1_cfg.set_filter(ads126x::register::DigitalFilter::Sinc1);
    //     self.adc1.set_mode1(&mode1_cfg, &mut self.spi)?;
    //     info!("ADC2 mode1 set");

    //     let mut mode2_cfg = Mode2Register::default();
    //     mode2_cfg.set_dr(DataRate::SPS1200);
    //     self.adc1.set_mode2(&mode2_cfg, &mut self.spi)?;
    //     info!("ADC2 mode2 set");

    //     self.adc1.send_command(ADCCommand::START1, &mut self.spi)?;
    //     info!("ADC2 start1");
    //     self.adc1.send_command(ADCCommand::START2, &mut self.spi)?;
    //     info!("ADC2 start2");
    //     Ok(())
    // }

    pub fn select_adc1(&mut self) {
        self.adc2_cs.set_high();
        self.adc1_cs.set_low();
    }

    // pub fn select_adc2(&mut self) {
    //     self.adc1_cs.set_high();
    //     self.adc2_cs.set_low();
    // }

    pub fn set_adc1_inpmux(
        &mut self,
        negative: ads126x::register::NegativeInpMux,
        positive: ads126x::register::PositiveInpMux,
    ) -> Result<(), ads126x::error::ADS126xError> {
        self.select_adc1();
        let mut reg = ads126x::register::InpMuxRegister::default();
        reg.set_muxn(negative);
        reg.set_muxp(positive);
        self.adc1.set_inpmux(&reg, &mut self.spi)
    }

    /*
    There are possibly 4,5, or 6 bytes of data to read from ADC1. There is an optonal status byte first and an optional CRC/CHK byte last.
    There are possibly 3,4, or 5 bytes of data to read from ADC2. There is an optonal status byte first and a fixed-value byte equal to 00h (zero pad byte) and an optional CRC/CHK byte.
    We can poll and just keep checking the ADC1 or ADC2 new data bit.
    ADC does not respond to commands until the read operation is complete, or terminated by CS going high.
    The data bytes are from the 32-bit conversion word.

     */

    pub fn read_adc1_data(
        &mut self,
        negative: ads126x::register::NegativeInpMux,
        positive: ads126x::register::PositiveInpMux,
    ) -> Result<i32, ads126x::error::ADS126xError> {
        self.select_adc1();
        // configure the input mux, buf first read the current config
        let inpmux_reg = self.adc1.get_inpmux(&mut self.spi)?;
        if inpmux_reg.get_muxn() != negative || inpmux_reg.get_muxp() != positive {
            let mut reg = ads126x::register::InpMuxRegister::default();
            reg.set_muxn(negative);
            reg.set_muxp(positive);
            self.adc1.set_inpmux(&reg, &mut self.spi)?;
        }

        info!("Input mux set");
        // ask for data
        /*
        If no command is intended, keep the DIN pin low during readback. If an input command is
        sent during readback, the ADC executes the command, and data interruption may result.
         */
        self.adc1.read_data1(&mut self.spi)
    }
}
