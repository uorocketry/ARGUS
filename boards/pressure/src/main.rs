#![no_std]
#![no_main]

// mod state_machine;
pub mod adc_manager;
mod can_manager;
mod data_manager;
mod time_manager;
mod traits;
mod types;

use adc_manager::AdcManager;
use can_manager::CanManager;
use chrono::NaiveDate;
use common_arm::*;
use data_manager::DataManager;
use defmt::info;
use messages::CanMessage;
use panic_probe as _;
use rtic_monotonics::systick::prelude::*;
use rtic_sync::{channel::*, make_channel};
use smlang::statemachine;
use stm32h7xx_hal::gpio::gpioa::{PA2, PA3};
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::gpio::PA4;
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::hal::spi;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::rtc;
use stm32h7xx_hal::{rcc, rcc::rec};
use types::COM_ID; // global logger

const DATA_CHANNEL_CAPACITY: usize = 10;

systick_monotonic!(Mono, 500);

statemachine! {
    transitions: {
        *Init + Start = Idle,
        Idle | Recovery + WantsCollection = Collection,
        Idle + NoConfig = Calibration,
        Collection + WantsProcessing = Processing,
        Calibration + Configured = Idle,
        Fault + FaultCleared = Idle,
        _ + FaultDetected = Fault,
    }
}

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI0, EXTI2, SPI3, SPI2])]
mod app {
    use messages::CanData;
    use stm32h7xx_hal::{
        gpio::{Edge, ExtiPin, Pin},
        hal::adc,
    };

    use super::*;

    #[shared]
    struct SharedResources {
        data_manager: DataManager,
        em: ErrorManager,
        // sd_manager: SdManager<
        //     stm32h7xx_hal::spi::Spi<stm32h7xx_hal::pac::SPI1, stm32h7xx_hal::spi::Enabled>,
        //     PA4<Output<PushPull>>,
        // >,
        // can_command_manager: CanManager<stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN1>>,
        // can_data_manager: CanManager<stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>>,
        rtc: rtc::Rtc,
        adc_manager: AdcManager,
    }

    #[local]
    struct LocalResources {
        state_machine: StateMachine<traits::Context>,
        can_sender: Sender<'static, CanMessage, DATA_CHANNEL_CAPACITY>,
        led_red: PA2<Output<PushPull>>,
        led_green: PA3<Output<PushPull>>,
        adc1_int: Pin<'A', 15, stm32h7xx_hal::gpio::Input>,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources) {
        // channel setup
        let (can_sender, can_receiver) = make_channel!(CanMessage, DATA_CHANNEL_CAPACITY);

        let core = ctx.core;

        /* Logging Setup */
        // turn off logging for the moment
        // HydraLogging::set_ground_station_callback(queue_log_message);

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
        let fdcan_prec_unsafe = unsafe { rcc.steal_peripheral_rec() }
            .FDCAN
            .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        let ccdr = rcc
            // .use_hse(48.MHz()) // check the clock hardware
            // .sys_ck(200.MHz())
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(32.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);
        info!("RCC configured");
        let fdcan_prec = ccdr
            .peripheral
            .FDCAN
            .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        // GPIO
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioe = ctx.device.GPIOE.split(ccdr.peripheral.GPIOE);

        info!("PWM enabled");
        // assert_eq!(ccdr.clocks.pll1_q_ck().unwrap().raw(), 32_000_000);
        info!("PLL1Q:");
        // https://github.com/stm32-rs/stm32h7xx-hal/issues/369 This needs to be stolen. Grrr I hate the imaturity of the stm32-hal
        // let can2: fdcan::FdCan<
        //     stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>,
        //     fdcan::ConfigMode,
        // > = {
        //     let rx = gpiob.pb12.into_alternate().speed(Speed::VeryHigh);
        //     let tx = gpiob.pb13.into_alternate().speed(Speed::VeryHigh);
        //     ctx.device.FDCAN2.fdcan(tx, rx, fdcan_prec)
        // };

        // let can_data_manager = CanManager::new(can2);

        // let can1: fdcan::FdCan<
        //     stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN1>,
        //     fdcan::ConfigMode,
        // > = {
        //     let rx = gpioa.pa11.into_alternate().speed(Speed::VeryHigh);
        //     let tx = gpioa.pa12.into_alternate().speed(Speed::VeryHigh);
        //     ctx.device.FDCAN1.fdcan(tx, rx, fdcan_prec_unsafe)
        // };

        // let can_command_manager = CanManager::new(can1);

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
            stm32h7xx_hal::spi::Config::new(spi::MODE_1),
            6.MHz(),
            ccdr.peripheral.SPI1,
            &ccdr.clocks,
        );

        let cs_sd = gpioa.pa4.into_push_pull_output();

        // let sd_manager = SdManager::new(spi_sd, cs_sd);

        // ADC setup
        let adc_spi: stm32h7xx_hal::spi::Spi<
            stm32h7xx_hal::stm32::SPI4,
            stm32h7xx_hal::spi::Enabled,
            u8,
        > = ctx.device.SPI4.spi(
            (
                gpioe.pe2.into_alternate(),
                gpioe.pe5.into_alternate(),
                gpioe.pe6.into_alternate(),
            ),
            stm32h7xx_hal::spi::Config::new(spi::MODE_1), // mode 1 per datasheet
            8.MHz(),                                      // 125 ns
            ccdr.peripheral.SPI4,
            &ccdr.clocks,
        );

        let adc1_cs = gpioc.pc10.into_push_pull_output();
        let adc2_cs = gpiod.pd2.into_push_pull_output();

        let adc1_rst = gpioc.pc11.into_push_pull_output();
        let adc2_rst = gpiod.pd1.into_push_pull_output();

        let mut adc_manager = AdcManager::new(adc_spi, adc1_rst, adc2_rst, adc1_cs, adc2_cs);
        adc_manager.init_adc1().ok();

        // leds
        let led_red = gpioa.pa2.into_push_pull_output();
        let led_green = gpioa.pa3.into_push_pull_output();

        let mut rtc = stm32h7xx_hal::rtc::Rtc::open_or_init(
            ctx.device.RTC,
            backup.RTC,
            stm32h7xx_hal::rtc::RtcClock::Lsi,
            &ccdr.clocks,
        );

        // TODO: Get current time from some source, this should be the responsibility of pheonix to sync the boards with GPS time.
        let now = NaiveDate::from_ymd_opt(2001, 1, 1)
            .unwrap()
            .and_hms_opt(0, 0, 0)
            .unwrap();
        rtc.set_date_time(now);
        let mut syscfg = ctx.device.SYSCFG;
        let mut exti = ctx.device.EXTI;
        // setup interupt drdy pins
        let mut adc1_int = gpioa.pa15.into_pull_up_input();
        adc1_int.make_interrupt_source(&mut syscfg);
        adc1_int.trigger_on_edge(&mut exti, Edge::Rising);
        adc1_int.enable_interrupt(&mut exti);

        /* Monotonic clock */
        Mono::start(core.SYST, 200_000_000);

        let mut data_manager = DataManager::new();
        data_manager.set_reset_reason(reset);
        let em = ErrorManager::new();
        let state_machine = StateMachine::new(traits::Context {});

        blink::spawn().ok();
        // send_data_internal::spawn(can_receiver).ok();
        reset_reason_send::spawn().ok();
        state_send::spawn().ok();
        read_adc1::spawn().ok();
        info!("Online");

        (
            SharedResources {
                data_manager,
                em,
                // sd_manager,
                // can_command_manager,
                // can_data_manager,
                rtc,
                adc_manager,
            },
            LocalResources {
                adc1_int,
                can_sender,
                led_red,
                led_green,
                state_machine,
            },
        )
    }

    #[task(priority = 3, binds = EXTI15_10, shared = [adc_manager], local = [adc1_int])]
    fn adc1_data_ready(mut cx: adc1_data_ready::Context) {
        info!("new data available come through");
        cx.shared.adc_manager.lock(|adc_manager| {
            let data = adc_manager.read_adc1_data(
                ads126x::register::NegativeInpMux::AIN1,
                ads126x::register::PositiveInpMux::AIN0,
            );
            match data {
                Ok(data) => {
                    info!("data: {:?}", data);
                }
                Err(_) => {
                    info!("Error reading data");
                }
            }
        });
        cx.local.adc1_int.clear_interrupt_pending_bit();
    }

    #[task(priority = 3, shared = [adc_manager, &em, rtc])]
    async fn read_adc1(mut cx: read_adc1::Context) {
        loop {
            info!("Reading ADC1");
            cx.shared.adc_manager.lock(|adc_manager| {
                let data = adc_manager.read_adc1_data(
                    ads126x::register::NegativeInpMux::AIN1,
                    ads126x::register::PositiveInpMux::AIN0,
                );
                match data {
                    Ok(data) => {
                        info!("data: {:?}", data);
                    }
                    Err(_) => {
                        info!("Error reading data");
                    }
                }
            });
            Mono::delay(10.millis()).await
        }
    }

    #[task(priority = 3, shared = [data_manager, &em, rtc])]
    async fn reset_reason_send(mut cx: reset_reason_send::Context) {
        let reason = cx
            .shared
            .data_manager
            .lock(|data_manager| data_manager.reset_reason.take());
        match reason {
            Some(reason) => {
                let message = messages::CanMessage::new(
                    cx.shared
                        .rtc
                        .lock(|rtc| messages::FormattedNaiveDateTime(rtc.date_time().unwrap())),
                    COM_ID,
                    CanData::Common(reason.into()),
                );

                cx.shared.em.run(|| {
                    spawn!(queue_data_internal, message)?;
                    Ok(())
                });
            }
            None => return,
        }
    }

    #[task(priority = 3)]
    async fn delay(_cx: delay::Context, delay: u32) {
        Mono::delay(delay.millis()).await;
    }

    #[task(shared = [data_manager, &em, rtc])]
    async fn state_send(mut cx: state_send::Context) {
        let state_data = cx
            .shared
            .data_manager
            .lock(|data_manager| data_manager.state.clone());
        cx.shared.em.run(|| {
            if let Some(x) = state_data {
                let can_data: CanData = CanData::Common(x.into());
                let message = CanMessage::new(
                    cx.shared
                        .rtc
                        .lock(|rtc| messages::FormattedNaiveDateTime(rtc.date_time().unwrap())),
                    COM_ID,
                    can_data,
                );
                cx.shared.em.run(|| {
                    spawn!(queue_data_internal, message)?;
                    Ok(())
                });
            } // if there is none we still return since we simply don't have data yet.
            Ok(())
        });
        Mono::delay(5.secs()).await;
        // spawn_after!(state_send, ExtU64::secs(5)).ok();
    }

    /**
     * Sends information about the sensors.
     */
    #[task(priority = 3, shared = [data_manager, rtc, &em])]
    async fn sensor_send(mut cx: sensor_send::Context) {
        let sensors = cx
            .shared
            .data_manager
            .lock(|data_manager| data_manager.temperature.take());

        cx.shared.em.run(|| {
            match sensors {
                Some(x) => {
                    for sensor in x.iter() {
                        let message = CanMessage::new(
                            messages::FormattedNaiveDateTime(
                                cx.shared.rtc.lock(|rtc| rtc.date_time().unwrap()),
                            ),
                            COM_ID,
                            CanData::Temperature(*sensor),
                        );
                        spawn!(queue_data_internal, message)?;
                    }
                }
                None => {
                    info!("No sensor data to send");
                }
            }
            Ok(())
        });
    }

    /// Callback for our logging library to access the needed resources.
    pub fn queue_log_message(d: impl Into<CanData>) {
        send_log_intermediate::spawn(d.into()).ok();
    }

    #[task(priority = 3, local = [can_sender], shared = [&em])]
    async fn queue_data_internal(cx: queue_data_internal::Context, m: CanMessage) {
        match cx.local.can_sender.send(m).await {
            // Preferably clean this up to be handled by the error manager.
            Ok(_) => {}
            Err(_) => {
                info!("Failed to send data");
            }
        }
    }

    #[task(priority = 3, shared = [rtc, &em])]
    async fn send_log_intermediate(mut cx: send_log_intermediate::Context, m: CanData) {
        cx.shared.em.run(|| {
            cx.shared.rtc.lock(|rtc| {
                let message = messages::CanMessage::new(
                    messages::FormattedNaiveDateTime(rtc.date_time().unwrap()),
                    COM_ID,
                    m,
                );

                spawn!(queue_data_internal, message)?;
                Ok(())
            })
        });
    }

    // #[task(priority = 2, binds = FDCAN1_IT0, shared = [can_command_manager, data_manager, &em])]
    // fn can_command(mut cx: can_command::Context) {
    //     // info!("CAN Command");
    //     cx.shared.can_command_manager.lock(|can| {
    //         cx.shared
    //             .data_manager
    //             .lock(|data_manager| cx.shared.em.run(|| can.process_data(data_manager)));
    //     })
    // }

    // #[task( priority = 3, binds = FDCAN2_IT0, shared = [&em, can_data_manager, data_manager])]
    // fn can_data(mut cx: can_data::Context) {
    //     cx.shared.can_data_manager.lock(|can| {
    //         {
    //             cx.shared.data_manager.lock(|data_manager| {
    //                 cx.shared.em.run(|| {
    //                     can.process_data(data_manager)?;
    //                     Ok(())
    //                 })
    //             })
    //         }
    //     });
    // }

    // #[task(priority = 2, shared = [&em, can_data_manager, data_manager])]
    // async fn send_data_internal(
    //     mut cx: send_data_internal::Context,
    //     mut receiver: Receiver<'static, CanMessage, DATA_CHANNEL_CAPACITY>,
    // ) {
    //     loop {
    //         if let Ok(m) = receiver.recv().await {
    //             cx.shared.can_data_manager.lock(|can| {
    //                 cx.shared.em.run(|| {
    //                     can.send_message(m)?;
    //                     Ok(())
    //                 })
    //             });
    //         }
    //     }
    // }

    // #[task(priority = 2, shared = [&em, can_command_manager, data_manager])]
    // async fn send_command_internal(mut cx: send_command_internal::Context, m: CanMessage) {
    //     cx.shared.can_command_manager.lock(|can| {
    //         cx.shared.em.run(|| {
    //             can.send_message(m)?;
    //             Ok(())
    //         })
    //     });
    // }

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

    #[task(priority = 3, shared = [&em])]
    async fn sleep_system(_cx: sleep_system::Context) {
        // in here we can stop the ADCs.
    }
}
