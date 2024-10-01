use crate::data_manager::DataManager;
use crate::types::COM_ID;
use common_arm::HydraError;
use defmt::info;
use fdcan::{
    config::DataBitTiming, frame::{FrameFormat, TxFrameHeader}, id::StandardId
};
use messages::Message;
use postcard::from_bytes;
use fdcan::{
    config::NominalBitTiming,
    filter::{StandardFilter, StandardFilterSlot},
};
use stm32h7xx_hal::{can::CanExt, gpio::Speed};
use core::num::{NonZeroU16, NonZeroU8};
use stm32h7xx_hal::rcc::rec::Fdcan;

pub struct CanManager {
    can: fdcan::FdCan<
        stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>,
        fdcan::NormalOperationMode,
    >,
}

impl CanManager {
    
    pub fn new(
        fdcan_prec: Fdcan,
        can_peripheral: stm32h7xx_hal::pac::FDCAN2,
        rx_pin: stm32h7xx_hal::gpio::gpiob::PB12,
        tx_pin: stm32h7xx_hal::gpio::gpiob::PB13,
    ) -> Self {
        let btr = NominalBitTiming {
            prescaler: NonZeroU16::new(10).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        };

        let mut can: fdcan::FdCan<
        stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>,
        fdcan::ConfigMode,
        > = can_peripheral.fdcan(tx_pin.into_alternate().speed(Speed::VeryHigh), rx_pin.into_alternate().speed(Speed::VeryHigh) , fdcan_prec);

        can.set_protocol_exception_handling(false);

        can.set_nominal_bit_timing(btr);
        let data_bit_timing = DataBitTiming {
            prescaler: NonZeroU8::new(10).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(4).unwrap(),
            transceiver_delay_compensation: true,
        };
        can.set_data_bit_timing(data_bit_timing);

        can.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );

        can.set_standard_filter(
            StandardFilterSlot::_1,
            StandardFilter::accept_all_into_fifo0(),
        );

        can.set_standard_filter(
            StandardFilterSlot::_2,
            StandardFilter::accept_all_into_fifo0(),
        );

        can.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);

        can.enable_interrupt_line(fdcan::interrupt::InterruptLine::_0, true);

        let config = can
            .get_config()
            .set_frame_transmit(fdcan::config::FrameTransmissionConfig::AllowFdCanAndBRS);
        can.apply_config(config);


        Self { can: can.into_normal() }
    }
    pub fn send_message(&mut self, m: Message) -> Result<(), HydraError> {
        let mut buf = [0u8; 64];
        let payload = postcard::to_slice(&m, &mut buf)?;
        let header = TxFrameHeader {
            len: payload.len() as u8, // switch to const as this never changes or swtich on message type of known size
            id: StandardId::new(COM_ID.into()).unwrap().into(),
            frame_format: FrameFormat::Fdcan,
            bit_rate_switching: false,
            marker: None,
        };
        // self.can.abort(fdcan::Mailbox::_2); // this is needed if boards are not in sync (if they are not in sync that is a bigger problem)

        stm32h7xx_hal::nb::block!(self.can.transmit(header, payload))?;

        Ok(())
    }
    pub fn process_data(&mut self) -> Result<(), HydraError> {
        let mut buf = [0u8; 64];
        while self.can.receive0(&mut buf).is_ok() {
            if let Ok(data) = from_bytes::<Message>(&buf) {
                info!("Received message {}", data.clone());
            } else if let Err(e) = from_bytes::<Message>(&buf) {
                info!("Error: {:?}", e);
            }
        }
        self.can
            .clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
        Ok(())
    }
}