#![no_std]
#![no_main]

use common_arm::SdManager;
use defmt::info;
use panic_probe as _;
use stm32h7xx_hal::gpio::{Output, PushPull, PA4};
use stm32h7xx_hal::pac;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::spi;

struct State {
    sd_manager: SdManager<
        stm32h7xx_hal::spi::Spi<stm32h7xx_hal::pac::SPI1, stm32h7xx_hal::spi::Enabled>,
        PA4<Output<PushPull>>,
    >,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> State {
        let _cp = cortex_m::Peripherals::take().unwrap();
        let dp = pac::Peripherals::take().unwrap();

        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        info!("Power enabled");
        // RCC
        let mut rcc = dp.RCC.constrain();
        let reset = rcc.get_reset_reason();

        info!("Reset reason: {:?}", reset);

        let ccdr = rcc
            .use_hse(48.MHz()) // check the clock hardware
            .sys_ck(200.MHz())
            .freeze(pwrcfg, &dp.SYSCFG);
        info!("RCC configured");

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

        let spi_sd: stm32h7xx_hal::spi::Spi<
            stm32h7xx_hal::stm32::SPI1,
            stm32h7xx_hal::spi::Enabled,
            u8,
        > = dp.SPI1.spi(
            (
                gpioa.pa5.into_alternate::<5>(),
                gpioa.pa6.into_alternate(),
                gpioa.pa7.into_alternate(),
            ),
            spi::Config::new(spi::MODE_0),
            16.MHz(),
            ccdr.peripheral.SPI1,
            &ccdr.clocks,
        );

        let cs_sd = gpioa.pa4.into_push_pull_output();

        let sd_manager = SdManager::new(spi_sd, cs_sd);
        State { sd_manager }
    }

    #[test]
    fn writing_file(state: &mut State) {
        let sd_manager = &mut state.sd_manager;

        let mut test_file = sd_manager
            .open_file("testing.txt")
            .expect("Cannot open file");
        sd_manager
            .write(&mut test_file, b"Hello this is a test!")
            .expect("Could not write file.");
        sd_manager
            .write_str(&mut test_file, "Testing Strings")
            .expect("Could not write string");
        sd_manager
            .close_file(test_file)
            .expect("Could not close file."); // we are done with the file so destroy it
    }
}
