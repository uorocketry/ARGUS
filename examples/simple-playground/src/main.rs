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
    let _cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    info!("Power enabled");
    // RCC
    let mut rcc = dp.RCC.constrain();
    let reset = rcc.get_reset_reason();

    info!("Reset reason: {:?}", reset);

    let _ccdr = rcc.freeze(pwrcfg, &dp.SYSCFG);
    info!("RCC configured");

    loop {
        info!("Hello, world!");
    }
}
