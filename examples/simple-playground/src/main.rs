#![no_std]
#![no_main]

use common_arm as _;
use cortex_m_rt::entry;
use defmt::info;
use panic_probe as _;
use stm32h7xx_hal::pac;
use stm32h7xx_hal::prelude::*;

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    info!("Power enabled");
    // RCC
    let mut rcc = dp.RCC.constrain();
    let reset = rcc.get_reset_reason();

    info!("Reset reason: {:?}", reset);

    let ccdr = rcc
        .pll1_p_ck(32.MHz())
        // .pclk4(32.MHz())
        // .per_ck(32.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    info!("RCC configured");
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    let mut adc_spi: stm32h7xx_hal::spi::Spi<
        stm32h7xx_hal::stm32::SPI4,
        stm32h7xx_hal::spi::Enabled,
        u8,
    > = dp.SPI4.spi(
        (
            gpioe.pe2.into_alternate::<5>(),
            gpioe.pe5.into_alternate(),
            gpioe.pe6.into_alternate(),
        ),
        stm32h7xx_hal::spi::Config::new(stm32h7xx_hal::spi::MODE_1), // datasheet mentioned a mode 1 per datasheet
        8.MHz(),                                                     // 125 ns
        ccdr.peripheral.SPI4,
        &ccdr.clocks,
    );

    let mut adc1_rst = gpioc.pc11.into_push_pull_output();
    adc1_rst.set_high();

    let mut adc_cs = gpioc.pc10.into_push_pull_output().internal_pull_up(true);

    loop {
        let mut buffer = [0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        adc_cs.set_low();
        adc_spi.transfer(&mut buffer);
        adc_cs.set_high();
        info!("Buffer: {:?}", buffer);
    }
}
