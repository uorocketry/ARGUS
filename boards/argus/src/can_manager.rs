use core::num::{NonZeroU16, NonZeroU8};

use crate::data_manager::DataManager;
use crate::types::COM_ID;
use common_arm::HydraError;
use defmt::info;
use fdcan::Instance;
use fdcan::{
    config::NominalBitTiming,
    filter::{StandardFilter, StandardFilterSlot},
    frame::{FrameFormat, TxFrameHeader},
    id::StandardId,
};
use messages::CanMessage;
use postcard::from_bytes;

/// Clock configuration is out of scope for this builder
/// easiest way to avoid alloc is to use no generics
pub struct CanManager<I: Instance> {
    // can: fdcan::FdCan<I, fdcan::InternalLoopbackMode>,
    can: fdcan::FdCan<I, fdcan::NormalOperationMode>,
}

impl<I: Instance> CanManager<I> {
    pub fn new(mut can: fdcan::FdCan<I, fdcan::ConfigMode>) -> Self {
        // let data_bit_timing = DataBitTiming {
        //     prescaler: NonZeroU8::new(10).unwrap(),
        //     seg1: NonZeroU8::new(13).unwrap(),
        //     seg2: NonZeroU8::new(2).unwrap(),
        //     sync_jump_width: NonZeroU8::new(4).unwrap(),
        //     transceiver_delay_compensation: true,
        // };
        // can_data.set_automatic_retransmit(false); // data can be dropped due to its volume.

        // can_command.set_data_bit_timing(data_bit_timing);
        let btr = NominalBitTiming {
            prescaler: NonZeroU16::new(10).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        };

        // Why?
        can.set_protocol_exception_handling(false);

        can.set_nominal_bit_timing(btr);
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

        Self {
            // can: can.into_internal_loopback(),
            can: can.into_normal(),
        }
    }
    pub fn send_message(&mut self, m: CanMessage) -> Result<(), HydraError> {
        let mut buf = [0u8; 64];
        let payload = postcard::to_slice(&m, &mut buf)?;
        let header = TxFrameHeader {
            len: payload.len() as u8, // switch to const as this never changes or swtich on message type of known size
            id: StandardId::new(COM_ID.into()).unwrap().into(),
            frame_format: FrameFormat::Fdcan,
            bit_rate_switching: false,
            marker: None,
        };
        self.can.transmit(header, payload)?;
        Ok(())
    }
    pub fn process_data(&mut self, data_manager: &mut DataManager) -> Result<(), HydraError> {
        let mut buf = [0u8; 64];
        while self.can.receive0(&mut buf).is_ok() {
            if let Ok(data) = from_bytes::<CanMessage>(&buf) {
                data_manager.handle_command(data.clone())?;
                info!("Received: {:?}", data);
            } else {
                info!("Error: {:?}", from_bytes::<CanMessage>(&buf).unwrap_err());
            }
        }
        Ok(())
    }
}
