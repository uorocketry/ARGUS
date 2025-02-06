use ads126x::{
    register::{Mode2Register, NegativeInpMux, PositiveInpMux},
    ADCCommand, Ads126x,
};

use defmt::info;
use embedded_hal::digital::v2::OutputPin;
use heapless::Vec;
use messages::sensor::AdcSensor;
use stm32h7xx_hal::{
    gpio::{Output, Pin, PushPull},
    spi::Spi,
};

// There is an option to use interrupts using the data ready pins, but for now we will poll.
pub struct AdcManager<GpioPin>
where
    GpioPin: OutputPin,
{
    pub spi: Spi<stm32h7xx_hal::pac::SPI4, stm32h7xx_hal::spi::Enabled, u8>,
    pub adc1: Ads126x<Pin<'C', 11, Output<PushPull>>>,
    pub adc2: Ads126x<GpioPin>,
    pub adc1_cs: Pin<'C', 10, Output<PushPull>>,
    pub adc2_cs: Pin<'D', 2, Output<PushPull>>,
    pub adc1_sensors: (u8, Vec<AdcSensor, 4>), // (index, sensors)
    pub adc2_sensors: (u8, Vec<AdcSensor, 4>),
}

impl<GpioPin> AdcManager<GpioPin>
where
    GpioPin: OutputPin,
{
    pub fn new(
        spi: Spi<stm32h7xx_hal::pac::SPI4, stm32h7xx_hal::spi::Enabled, u8>,
        adc1_rst: Pin<'C', 11, Output<PushPull>>,
        adc2_rst: GpioPin,
        adc1_cs: Pin<'C', 10, Output<PushPull>>,
        adc2_cs: Pin<'D', 2, Output<PushPull>>,
    ) -> Self {
        let sensor1 = AdcSensor {
            adc: 1,
            positive_input: PositiveInpMux::AIN0,
            negative_input: NegativeInpMux::AIN1,
        };

        let sensor2 = AdcSensor {
            adc: 1,
            positive_input: PositiveInpMux::AIN2,
            negative_input: NegativeInpMux::AIN3,
        };

        let sensor3 = AdcSensor {
            adc: 1,
            positive_input: PositiveInpMux::AIN4,
            negative_input: NegativeInpMux::AIN5,
        };

        let sensor4 = AdcSensor {
            adc: 1,
            positive_input: PositiveInpMux::AIN6,
            negative_input: NegativeInpMux::AIN7,
        };

        let sensor5 = AdcSensor {
            adc: 2,
            positive_input: PositiveInpMux::AIN0,
            negative_input: NegativeInpMux::AIN1,
        };

        let sensor6 = AdcSensor {
            adc: 2,
            positive_input: PositiveInpMux::AIN2,
            negative_input: NegativeInpMux::AIN3,
        };

        let sensor7 = AdcSensor {
            adc: 2,
            positive_input: PositiveInpMux::AIN4,
            negative_input: NegativeInpMux::AIN5,
        };

        let sensor8 = AdcSensor {
            adc: 2,
            positive_input: PositiveInpMux::AIN6,
            negative_input: NegativeInpMux::AIN7,
        };

        // insert the sensors
        let adc1_sensors = Vec::from_slice(&[sensor1, sensor2, sensor3, sensor4])
            .expect("Cannot create adc1_sensors vector.");
        
        let adc2_sensors = Vec::from_slice(&[sensor5, sensor6, sensor7, sensor8])
            .expect("Cannot create adc2_sensors vector.");

        Self {
            spi,
            adc1: Ads126x::new(adc1_rst),
            adc2: Ads126x::new(adc2_rst),
            adc1_cs,
            adc2_cs,
            adc1_sensors: (0, adc1_sensors),
            adc2_sensors: (0, adc2_sensors),
        }
    }

    pub fn init_adc1(&mut self) -> Result<(), ads126x::error::ADS126xError> {
        self.select_adc1();
        self.adc1.set_reset_high().unwrap();

        // TODO: move these values to const, or pass in as a parameter (preferably pass as parameter)
        cortex_m::asm::delay(65536 * (96_000_000 / 6_000_000)); // delay 2^16 cycles of the ADC 

        // self.adc1.send_command(ADCCommand::RESET, &mut self.spi)?;

        // // setup the Power register
        #[cfg(any(feature = "temperature", feature = "pressure"))]
        {
            // We need to enable vbias
            let mut power_cfg = ads126x::register::PowerRegister::default();
            power_cfg.set_vbias(true);
            self.adc1.set_power(&power_cfg, &mut self.spi)?;
            // Set gain
            let mut mode2_cfg = Mode2Register::default();
            mode2_cfg.set_gain(ads126x::register::PGAGain::VV8);
            self.adc1.set_mode2(&mode2_cfg, &mut self.spi);
        }
        // let mut mode0_cfg = ads126x::register::Mode0Register::default();

        // Verify none custom config works first
        // setup mode 1 and mode 2 registers
        // let mut mode1_cfg = Mode1Register::default();
        // mode1_cfg.set_filter(ads126x::register::DigitalFilter::Sinc1);
        // self.adc1.set_mode1(&mode1_cfg, &mut self.spi)?;

        // let mut mode2_cfg = Mode2Register::default();
        // mode2_cfg.set_dr(DataRate::SPS1200);
        // self.adc1.set_mode2(&mode2_cfg, &mut self.spi)?;

        // read back the mode1 and mode2 registers to verify
        // let mode1_cfg_real = self.adc1.get_mode1(&mut self.spi)?;

        // let mode2_cfg_real = self.adc1.get_mode2(&mut self.spi)?;

        // verify
        // info!("Mode1: {:#010b}", mode1_cfg_real.bits());
        // info!("Mode2: {:#010b}", mode2_cfg_real.bits());
        // assert!(mode1_cfg.difference(mode1_cfg_real).is_empty());
        // assert!(mode2_cfg.difference(mode2_cfg_real).is_empty());

        // start conversions
        self.set_adc1_inpmux(&mut self.adc1_sensors.1[0].clone())?;

        self.adc1.send_command(ADCCommand::START1, &mut self.spi)?;

        self.adc1.send_command(ADCCommand::START2, &mut self.spi)?;

        self.adc1.send_command(ADCCommand::RDATA1, &mut self.spi)?;

        Ok(())
    }

    pub fn init_adc2(&mut self) -> Result<(), ads126x::error::ADS126xError> {
        self.select_adc1();
        self.adc2.set_reset_high()?;

        // 2^16 cycles of delay
        cortex_m::asm::delay(65536 * (96_000_000 / 6_000_000));
        
        #[cfg(feature = "temperature")]
        {
            // We need to enable vbias
            let mut power_cfg = ads126x::register::PowerRegister::default();
            power_cfg.set_vbias(true);
            self.adc2.set_power(&power_cfg, &mut self.spi)?;
            // Set gain
            let mut mode2_cfg = Mode2Register::default();
            mode2_cfg.set_gain(ads126x::register::PGAGain::VV8);
            self.adc2.set_mode2(&mode2_cfg, &mut self.spi);
        }
        // // stop conversions
        // self.adc2.send_command(ADCCommand::STOP1, &mut self.spi)?;
        // self.adc2.send_command(ADCCommand::STOP2, &mut self.spi)?;

        // // setup the Power register
        // let mut power_cfg = ads126x::register::PowerRegister::default();
        // power_cfg.clear_reset();
        // self.adc2.set_power(&power_cfg, &mut self.spi)?;

        // // Verify none custom config works first
        // // setup mode 1 and mode 2 registers
        // let mut mode1_cfg = Mode1Register::default();
        // mode1_cfg.set_filter(ads126x::register::DigitalFilter::Sinc1);
        // self.adc2.set_mode1(&mode1_cfg, &mut self.spi)?;

        // let mut mode2_cfg = Mode2Register::default();
        // mode2_cfg.set_dr(DataRate::SPS1200);
        // self.adc2.set_mode2(&mode2_cfg, &mut self.spi)?;

        // // read back the mode1 and mode2 registers to verify
        // let mode1_cfg_real = self.adc2.get_mode1(&mut self.spi)?;
        // let mode2_cfg_real = self.adc2.get_mode2(&mut self.spi)?;

        // // verify
        // info!("Mode1: {:#010b}", mode1_cfg_real.bits());
        // info!("Mode2: {:#010b}", mode2_cfg_real.bits());
        // // assert!(mode1_cfg.difference(mode1_cfg_real).is_empty());
        // // assert!(mode2_cfg.difference(mode2_cfg_real).is_empty());

        // // start conversions    // abstract these functions

        self.adc2.send_command(ADCCommand::START1, &mut self.spi)?;
        // self.adc2.send_command(ADCCommand::START2, &mut self.spi)?;

        self.adc2.send_command(ADCCommand::START2, &mut self.spi)?;

        self.adc2.send_command(ADCCommand::RDATA1, &mut self.spi)?;
        Ok(())
    }

    pub fn select_adc1(&mut self) {
        self.adc2_cs.set_high();
        self.adc1_cs.set_low();
    }

    pub fn select_adc2(&mut self) {
        self.adc1_cs.set_high();
        self.adc2_cs.set_low();
    }

    pub fn set_adc1_inpmux(
        &mut self,
        sensor: &mut AdcSensor,
    ) -> Result<(), ads126x::error::ADS126xError> {
        self.select_adc1();
        let mut reg = ads126x::register::InpMuxRegister::default();
        reg.set_muxn(&sensor.negative_input);
        reg.set_muxp(&sensor.positive_input);
        info!("Setting ADC1 InpMux: {:#010b}", reg.bits());
        self.adc1.set_inpmux(&reg, &mut self.spi)?;
        // verify the register
        let mut reg_real = self.adc1.get_inpmux(&mut self.spi)?;

        info!("Real ADC1 InpMux: {:#010b}", reg_real.bits());
        // assert_eq!(reg.bits(), reg_real.bits());
        Ok(())
    }

    pub fn set_adc2_inpmux(
        &mut self,
        sensor: &mut AdcSensor,
    ) -> Result<(), ads126x::error::ADS126xError> {
        self.select_adc2();
        let mut reg = ads126x::register::InpMuxRegister::default();
        reg.set_muxn(&sensor.negative_input);
        reg.set_muxp(&sensor.positive_input);
        self.adc2.set_inpmux(&reg, &mut self.spi)
    }

    /*
    There are possibly 4,5, or 6 bytes of data to read from ADC1. There is an optonal status byte first and an optional CRC/CHK byte last.
    There are possibly 3,4, or 5 bytes of data to read from ADC2. There is an optonal status byte first and a fixed-value byte equal to 00h (zero pad byte) and an optional CRC/CHK byte.
    We can poll and just keep checking the ADC1 or ADC2 new data bit.
    ADC does not respond to commands until the read operation is complete, or terminated by CS going high.
    The data bytes are from the 32-bit conversion word.

     */

    // abstract these functions

    pub fn read_adc1_data(
        &mut self,
    ) -> Result<(ads126x::register::StatusRegister, i32, u8), ads126x::error::ADS126xError> {
        self.select_adc1();
        self.adc1.read_data1(&mut self.spi)
    }

    pub fn read_adc2_data(
        &mut self,
    ) -> Result<(ads126x::register::StatusRegister, i32, u8), ads126x::error::ADS126xError> {
        self.select_adc2();
        self.adc2.read_data1(&mut self.spi)
    }

    // abstract these functions

    pub fn select_next_adc1_sensor(&mut self) {
        self.adc1_cs.set_high();
        self.adc1_cs.set_low();
        // select the next sensor based on round robin
        let current_index = (self.adc1_sensors.0 as usize + 1) % self.adc1_sensors.1.len();

        // set the inputmux
        let mut sensor = &mut self.adc1_sensors.1[current_index].clone();
        self.set_adc1_inpmux(&mut sensor).unwrap();

        // update the index
        self.adc1_sensors.0 = current_index as u8;
    }

    pub fn select_next_adc2_sensor(&mut self) {
        // select the next sensor
    }
}
