#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod types;

use chrono::NaiveDate;
use common_arm::*;
use communication::CanManager;
use core::cell::RefCell;
use core::num::{NonZeroU16, NonZeroU8};
use cortex_m::interrupt::Mutex;
use data_manager::DataManager;
use defmt::info;
use fdcan::{
    config::NominalBitTiming,
    filter::{StandardFilter, StandardFilterSlot},
};
use messages::{sensor, Data};
use panic_probe as _;
use rtic_monotonics::systick::prelude::*;
use rtic_sync::{channel::*, make_channel};
use stm32h7xx_hal::gpio::gpioa::{PA2, PA3};
use stm32h7xx_hal::gpio::gpioc::PC5;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::rtc;
use stm32h7xx_hal::{rcc, rcc::rec};
use stm32h7xx_hal::spi;
use types::COM_ID; // global logger

const DATA_CHANNEL_CAPACITY: usize = 10;

systick_monotonic!(Mono, 500);

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    // stm32h7xx_hal::pac::SCB::sys_reset()
    cortex_m::asm::udf()
}

static RTC: Mutex<RefCell<Option<rtc::Rtc>>> = Mutex::new(RefCell::new(None));

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2, SPI3, SPI2])]
mod app {

    use messages::Message;

    use super::*;

    #[shared]
    struct SharedResources {
        data_manager: DataManager,
        em: ErrorManager,
        sd_manager: SdManager<
            stm32h7xx_hal::spi::Spi<stm32h7xx_hal::pac::SPI1, stm32h7xx_hal::spi::Enabled>,
            PC5<Output<PushPull>>,
        >,
        can_manager: CanManager,
    }
    #[local]
    struct LocalResources {
        led_red: PA2<Output<PushPull>>,
        led_green: PA3<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources) {
        // channel setup
        let (_s, r) = make_channel!(Message, DATA_CHANNEL_CAPACITY);

        let core = ctx.core;

        /* Logging Setup */
        HydraLogging::set_ground_station_callback(queue_gs_message);

        let pwr = ctx.device.PWR.constrain();
        // We could use smps, but the board is not designed for it
        // let pwrcfg = example_power!(pwr).freeze();
        let mut pwrcfg = pwr.freeze();

        info!("Power enabled");
        let backup = pwrcfg.backup().unwrap();
        info!("Backup domain enabled");
        // RCC
        let mut rcc = ctx.device.RCC.constrain();
        let reset = rcc.get_reset_reason();

        let ccdr = rcc
            .use_hse(48.MHz()) // check the clock hardware
            .sys_ck(200.MHz())
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(32.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);
        info!("RCC configured");

        // GPIO
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);


        let fdcan_prec: rec::Fdcan = ccdr
            .peripheral
            .FDCAN
            .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        let can_manager = CanManager::new(
            fdcan_prec,
            ctx.device.FDCAN2,
            gpiob.pb12,
            gpiob.pb13,
        );

        assert_eq!(ccdr.clocks.pll1_q_ck().unwrap().raw(), 32_000_000);
        info!("PLL1Q:");

        let spi_sd: stm32h7xx_hal::spi::Spi<
            stm32h7xx_hal::stm32::SPI1,
            stm32h7xx_hal::spi::Enabled,
            u8,
        > = ctx.device.SPI1.spi(
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

        let cs_sd = gpioc.pc5.into_push_pull_output();

        let sd_manager = SdManager::new(spi_sd, cs_sd);

        // leds
        let led_red = gpioa.pa2.into_push_pull_output();
        let led_green = gpioa.pa3.into_push_pull_output();

        let mut rtc = stm32h7xx_hal::rtc::Rtc::open_or_init(
            ctx.device.RTC,
            backup.RTC,
            stm32h7xx_hal::rtc::RtcClock::Lsi,
            &ccdr.clocks,
        );

        // TODO: Get current time from some source
        let now = NaiveDate::from_ymd_opt(2001, 1, 1)
            .unwrap()
            .and_hms_opt(0, 0, 0)
            .unwrap();

        rtc.set_date_time(now);

        cortex_m::interrupt::free(|cs| {
            RTC.borrow(cs).replace(Some(rtc));
        });

        /* Monotonic clock */
        Mono::start(core.SYST, 200_000_000);

        let mut data_manager = DataManager::new();
        data_manager.set_reset_reason(reset);
        let em = ErrorManager::new();
        blink::spawn().ok();
        send_data_internal::spawn(r).ok();
        reset_reason_send::spawn().ok();
        info!("Online");

        (
            SharedResources {
                data_manager,
                em,
                sd_manager,
                can_manager,
            },
            LocalResources {
                led_red,
                led_green,
            },
        )
    }

    #[task(priority = 3, shared = [data_manager, &em])]
    async fn reset_reason_send(mut cx: reset_reason_send::Context) {
        let reason = cx
            .shared
            .data_manager
            .lock(|data_manager| data_manager.clone_reset_reason());
        match reason {
            Some(reason) => {
                let x = match reason {
                    stm32h7xx_hal::rcc::ResetReason::BrownoutReset => sensor::ResetReason::BrownoutReset,
                    stm32h7xx_hal::rcc::ResetReason::CpuReset => sensor::ResetReason::CpuReset,
                    stm32h7xx_hal::rcc::ResetReason::D1EntersDStandbyErroneouslyOrCpuEntersCStopErroneously => sensor::ResetReason::D1EntersDStandbyErroneouslyOrCpuEntersCStopErroneously,
                    stm32h7xx_hal::rcc::ResetReason::D1ExitsDStandbyMode => sensor::ResetReason::D1ExitsDStandbyMode,
                    stm32h7xx_hal::rcc::ResetReason::D2ExitsDStandbyMode => sensor::ResetReason::D2ExitsDStandbyMode,
                    stm32h7xx_hal::rcc::ResetReason::GenericWatchdogReset => sensor::ResetReason::GenericWatchdogReset,
                    stm32h7xx_hal::rcc::ResetReason::IndependentWatchdogReset => sensor::ResetReason::IndependentWatchdogReset,
                    stm32h7xx_hal::rcc::ResetReason::PinReset => sensor::ResetReason::PinReset,
                    stm32h7xx_hal::rcc::ResetReason::PowerOnReset => sensor::ResetReason::PowerOnReset,
                    stm32h7xx_hal::rcc::ResetReason::SystemReset => sensor::ResetReason::SystemReset,
                    stm32h7xx_hal::rcc::ResetReason::Unknown { rcc_rsr } => sensor::ResetReason::Unknown { rcc_rsr },
                    stm32h7xx_hal::rcc::ResetReason::WindowWatchdogReset => sensor::ResetReason::WindowWatchdogReset,
                };
                let message =
                    messages::Message::new(Mono::now().ticks(), COM_ID, sensor::Sensor::new(x));

                // cx.shared.em.run(|| {
                //     spawn!(send_data_internal, message)?;
                //     Ok(())
                // })
            }
            None => return,
        }
    }

    // /**
    //  * Sends information about the sensors.
    //  */
    // #[task(priority = 3, shared = [data_manager, &em])]
    // async fn sensor_send(mut cx: sensor_send::Context) {
    //     loop {
    //         let (sensors, logging_rate) = cx.shared.data_manager.lock(|data_manager| {
    //             (data_manager.take_sensors(), data_manager.get_logging_rate())
    //         });

    //         cx.shared.em.run(|| {
    //             for msg in sensors {
    //                 match msg {
    //                     Some(x) => {
    //                         // info!("Sending sensor data {}", x.clone());
    //                         spawn!(send_gs, x)?;
    //                         //                     spawn!(sd_dump, x)?;
    //                     }
    //                     None => {
    //                         info!("No sensor data to send");
    //                         continue;
    //                     }
    //                 }
    //             }

    //             Ok(())
    //         });
    //         match logging_rate {
    //             RadioRate::Fast => {
    //                 Mono::delay(100.millis()).await;
    //             }
    //             RadioRate::Slow => {
    //                 Mono::delay(250.millis()).await;
    //             }
    //         }
    //     }
    // }

    /// Receives a log message from the custom logger so that it can be sent over the radio.
    pub fn queue_gs_message(d: impl Into<Data>) {
        info!("Queueing message");
        let message = messages::Message::new(Mono::now().ticks(), COM_ID, d.into());
        info!("{:?}", message);
        // send_in::spawn(message).ok();
    }

    #[task( priority = 3, binds = FDCAN2_IT0, shared = [&em, can_manager, data_manager])]
    fn can_data(mut cx: can_data::Context) {
        cx.shared.can_manager.lock(|can| {
            {
                cx.shared.em.run(|| {
                    can.process_data()?;
                    Ok(())
                })
            }
        });
    }

    #[task(priority = 2, shared = [&em, can_manager, data_manager])]
    async fn send_data_internal(
        mut cx: send_data_internal::Context,
        mut receiver: Receiver<'static, Message, DATA_CHANNEL_CAPACITY>,
    ) {
        loop {
            if let Ok(m) = receiver.recv().await {
                cx.shared.can_manager.lock(|can| {
                    cx.shared.em.run(|| {
                        can.send_message(m)?;
                        Ok(())
                    })
                });
            }
        }
    }

    #[task(priority = 1, local = [led_red, led_green], shared = [&em])]
    async fn blink(cx: blink::Context) {
        loop {
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