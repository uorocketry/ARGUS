#![no_std]
#![no_main]

use ads126x::register::{DataRate, Mode1Register, Mode2Register};
use ads126x::{ADCCommand, Ads126x};
use common_arm::SdManager;
use defmt::info;
use panic_probe as _;
use stm32h7xx_hal::gpio::{Output, Pin, PushPull, PA4};
use stm32h7xx_hal::nb;
use stm32h7xx_hal::pac;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::spi::{self, Spi};

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
        self.adc2_cs.set_high();
        self.adc1_cs.set_low();
        self.adc1.set_reset_high()?;

        // 2^16 cycles of delay
        cortex_m::asm::delay(65536);

        let mut mode1_cfg = Mode1Register::default();
        mode1_cfg.set_filter(ads126x::register::DigitalFilter::Sinc1);
        self.adc1.set_mode1(&mode1_cfg, &mut self.spi)?;

        let mut mode2_cfg = Mode2Register::default();
        mode2_cfg.set_dr(DataRate::SPS1200);
        self.adc1.set_mode2(&mode2_cfg, &mut self.spi)?;

        let mode1_cfg_real = self.adc1.get_mode1(&mut self.spi)?;
        let mode2_cfg_real = self.adc1.get_mode2(&mut self.spi)?;

        // verify
        info!("Mode1: {:#010b}", mode1_cfg_real.bits());
        info!("Mode2: {:#010b}", mode2_cfg_real.bits());
        assert!(mode1_cfg.difference(mode1_cfg_real).is_empty());
        assert!(mode2_cfg.difference(mode2_cfg_real).is_empty());

        self.adc1.send_command(ADCCommand::START1, &mut self.spi)?;
        // self.adc1.send_command(ADCCommand::START2, &mut self.spi)?;

        Ok(())
    }

    pub fn init_adc2(&mut self) -> Result<(), ads126x::error::ADS126xError> {
        self.adc1_cs.set_high();
        self.adc2_cs.set_low();
        self.adc2.set_reset_high()?;
        info!("ADC2 reset");

        cortex_m::asm::delay(65536);

        let mut mode1_cfg = Mode1Register::default();
        mode1_cfg.set_filter(ads126x::register::DigitalFilter::Sinc1);
        self.adc1.set_mode1(&mode1_cfg, &mut self.spi)?;
        info!("ADC2 mode1 set");

        let mut mode2_cfg = Mode2Register::default();
        mode2_cfg.set_dr(DataRate::SPS1200);
        self.adc1.set_mode2(&mode2_cfg, &mut self.spi)?;
        info!("ADC2 mode2 set");

        self.adc1.send_command(ADCCommand::START1, &mut self.spi)?;
        info!("ADC2 start1");
        self.adc1.send_command(ADCCommand::START2, &mut self.spi)?;
        info!("ADC2 start2");
        Ok(())
    }

    pub fn read_adc1_data(
        &mut self,
        negative: ads126x::register::NegativeInpMux,
        positive: ads126x::register::PositiveInpMux,
    ) -> Result<[u8; 4], ads126x::error::ADS126xError> {
        info!("setting pins");
        // check if our inputmux is set to use the right pins.
        self.adc2_cs.set_high(); // disable adc 2
        self.adc1_cs.set_low(); // enable adc 1
        info!("Pins set");
        // configure the input mux
        let mut reg = ads126x::register::InpMuxRegister::default();
        reg.set_muxn(negative);
        reg.set_muxp(positive);
        self.adc1.set_inpmux(&reg, &mut self.spi)?;
        info!("Input mux set");
        // ask for data
        self.adc1.send_command(ADCCommand::RDATA1, &mut self.spi)?;
        cortex_m::asm::delay(240_000_000);

        info!("Data requested");
        // read 4 bytes of data from the spi
        let mut data = [0; 4];
        for i in 0..4 {
            data[i] = self
                .spi
                .read()
                .map_err(|_| ads126x::error::ADS126xError::IO)?;
            info!("Data read");
        }

        Ok(data)
    }
}

#[defmt_test::tests]
mod tests {
    use stm32h7xx_hal::hal::adc;

    use super::*;

    #[init]
    fn init() -> AdcManager {
        let cp = cortex_m::Peripherals::take().unwrap();
        let dp = pac::Peripherals::take().unwrap();

        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        info!("Power enabled");
        // RCC
        let mut rcc = dp.RCC.constrain();
        info!("RCC constrained");
        let reset = rcc.get_reset_reason();

        info!("Reset reason: {:?}", reset);

        let ccdr = rcc
            // .use_hse(48.MHz()) // check the clock hardware
            // .sys_ck(200.MHz())
            .freeze(pwrcfg, &dp.SYSCFG);
        info!("RCC configured");

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        // ADC setup
        let adc_spi: stm32h7xx_hal::spi::Spi<
            stm32h7xx_hal::stm32::SPI4,
            stm32h7xx_hal::spi::Enabled,
            u8,
        > = dp.SPI4.spi(
            (
                gpioe.pe2.into_alternate(),
                gpioe.pe5.into_alternate(),
                gpioe.pe6.into_alternate(),
            ),
            stm32h7xx_hal::spi::Config::new(spi::MODE_0),
            1.MHz(),
            ccdr.peripheral.SPI4,
            &ccdr.clocks,
        );

        let adc1_cs = gpioc.pc10.into_push_pull_output();
        let adc2_cs = gpiod.pd2.into_push_pull_output();

        let adc1_rst = gpioc.pc11.into_push_pull_output();
        let adc2_rst = gpiod.pd1.into_push_pull_output();

        let mut adc_manager = AdcManager::new(adc_spi, adc1_rst, adc2_rst, adc1_cs, adc2_cs);

        adc_manager.init_adc1().ok();

        // adc_manager.init_adc2().ok();

        adc_manager
    }

    #[test]
    fn read_data_adc1(adc_manager: &mut AdcManager) {
        adc_manager.adc2_cs.set_high();
        adc_manager.adc1_cs.set_low();

        let data = adc_manager.read_adc1_data(
            ads126x::register::NegativeInpMux::AIN1,
            ads126x::register::PositiveInpMux::AIN0,
        );
        if let Ok(data) = data {
            info!("Data: {:?}", data);
        } else {
            panic!("Error reading data");
        }
    }
}
