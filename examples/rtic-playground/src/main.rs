#![no_std]
#![no_main]

use common_arm::ErrorManager;
use defmt::info;
use panic_probe as _;
use rtic_monotonics::systick::prelude::*;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::{
    gpio::{Output, PushPull, PA2, PA3},
    rcc,
};

systick_monotonic!(Mono, 500);

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2, SPI3, SPI2])]
mod app {

    use super::*;

    #[shared]
    struct SharedResources {
        em: ErrorManager,
    }
    #[local]
    struct LocalResources {
        led_red: PA2<Output<PushPull>>,
        led_green: PA3<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources) {
        let core = ctx.core;

        let pwr = ctx.device.PWR.constrain();
        // We could use smps, but the board is not designed for it
        // let pwrcfg = example_power!(pwr).freeze();
        let mut pwrcfg = pwr.freeze();

        info!("Power enabled");
        pwrcfg.backup().unwrap();
        info!("Backup domain enabled");
        // RCC
        let rcc = ctx.device.RCC.constrain();

        let ccdr = rcc
            .use_hse(48.MHz()) // check the clock hardware
            .sys_ck(200.MHz())
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(32.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);
        info!("RCC configured");

        /* Monotonic clock */
        Mono::start(core.SYST, 200_000_000);

        let em = ErrorManager::new();
        blink::spawn().ok();

        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let led_red = gpioa.pa2.into_push_pull_output();
        let led_green = gpioa.pa3.into_push_pull_output();

        info!("Online");

        (
            SharedResources { em },
            LocalResources { led_red, led_green },
        )
    }

    #[task(priority = 1, local = [led_red, led_green], shared = [&em])]
    async fn blink(cx: blink::Context) {
        loop {
            info!("Blinking");
            // check for errors.
            if cx.shared.em.has_error() {
                cx.local.led_red.toggle();
                Mono::delay(500.millis()).await;
            } else {
                cx.local.led_green.toggle();
                Mono::delay(2000.millis()).await;
            }
        }
    }
}
