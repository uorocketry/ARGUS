#![no_std]
#![no_main]

pub mod error;
pub mod register;

use bitflags::Flags;
use cortex_m::asm::delay;
use cortex_m::prelude::_embedded_hal_blocking_spi_Transfer;
use defmt::info;
use embedded_hal::blocking::spi::Transactional;
use error::ADS126xError;
use register::{
    Adc2CfgRegister, Adc2MuxRegister, GpioConRegister, GpioDatRegister, GpioDirRegister,
    IdRegister, IdacMagRegister, IdacMuxRegister, InpMuxRegister, InterfaceRegister, Mode0Register,
    Mode1Register, Mode2Register, PowerRegister, RefMuxRegister, Register, StatusRegister,
    TdacnRegister, TdacpRegister,
};

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::FullDuplex;
use heapless::Vec;

/// The [`Result`] type for ADS126x operations.
pub type Result<T> = core::result::Result<T, ADS126xError>;

pub struct Ads126x<GpioPin>
where
    GpioPin: OutputPin,
{
    reset_pin: GpioPin,
}

pub enum ADCCommand {
    NOP,
    RESET,
    START1,
    STOP1,
    START2,
    STOP2,
    RDATA1,
    RDATA2,
    SYOCAL1,
    SYGCAL1,
    SFOCAL1,
    SYOCAL2,
    SYGCAL2,
    SFOCAL2,
    RREG(Register, u8), // (register address, number of registers)
    WREG(Register, u8), // (register address, number of registers)
}

impl<GpioPin> Ads126x<GpioPin>
where
    GpioPin: OutputPin,
{
    pub fn new(reset_pin: GpioPin) -> Self {
        Self { reset_pin }
    }

    // consolidate this logic to one function.
    pub fn set_reset_high(&mut self) -> Result<()> {
        self.reset_pin.set_high().map_err(|_| ADS126xError::IO)?;
        Ok(())
    }

    pub fn set_reset_low(&mut self) -> Result<()> {
        self.reset_pin.set_low().map_err(|_| ADS126xError::IO)?;
        Ok(())
    }

    /// to issue read data command call read_data1 or read_data2.
    pub fn send_command<SPI>(&mut self, command: ADCCommand, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        let (opcode1, opcode2) = match command {
            ADCCommand::NOP => (0x00, None),
            ADCCommand::RESET => (0x06, None),
            ADCCommand::START1 => (0x08, None),
            ADCCommand::STOP1 => (0x0A, None),
            ADCCommand::START2 => (0x0C, None),
            ADCCommand::STOP2 => (0x0E, None),
            ADCCommand::RDATA1 => (0x12, None),
            ADCCommand::RDATA2 => (0x14, None),
            ADCCommand::SYOCAL1 => (0x16, None),
            ADCCommand::SYGCAL1 => (0x17, None),
            ADCCommand::SFOCAL1 => (0x19, None),
            ADCCommand::SYOCAL2 => (0x1B, None),
            ADCCommand::SYGCAL2 => (0x1C, None),
            ADCCommand::SFOCAL2 => (0x1E, None),
            ADCCommand::RREG(addr, num) => (0x20 | addr as u8, Some(num)),
            ADCCommand::WREG(addr, num) => (0x40 | addr as u8, Some(num)),
        };
        info!("Sending opcode 1: {:#04x}", opcode1);

        spi.send(opcode1).map_err(|_| ADS126xError::IO)?;
        if let Some(opcode2) = opcode2 {
            spi.send(opcode2).map_err(|_| ADS126xError::IO)?;
        }

        Ok(())
    }

    pub fn read_data1<SPI>(&mut self, spi: &mut SPI) -> Result<i32>
    where
        SPI: FullDuplex<u8> + _embedded_hal_blocking_spi_Transfer<u8>,
    {
        self.send_command(ADCCommand::RDATA1, spi)?;
        let mut buffer: [u8; 4] = [0; 4];
        for i in 0..4 {
            buffer[i] = nb::block!(spi.read()).map_err(|_| ADS126xError::IO)?;
        }
        let data: i32 = i32::from_be_bytes(buffer);
        Ok(data)
    }

    /// Reads data from multiple registers starting at the provided register.
    /// To read a single register, see [`ADS126x::read_register`].
    ///
    /// Vector returns byte for each register read in order registers were read (increasing address).
    pub fn read_multiple_registers<SPI>(
        &mut self,
        reg: Register,
        num: u8,
        spi: &mut SPI,
    ) -> Result<Vec<u8, 27>>
    where
        SPI: FullDuplex<u8>,
    {
        if num > 27 {
            return Err(ADS126xError::InvalidInputData);
        }
        self.send_command(ADCCommand::RREG(reg, num - 1), spi)?;
        let mut buffer: Vec<u8, 27> = Vec::new();
        for _ in 0..num {
            buffer
                .push(spi.read().map_err(|_| ADS126xError::IO)?)
                .map_err(|_| ADS126xError::InvalidInputData)?;
        }
        Ok(buffer)
    }
    /// Reads data from only the single provided register.
    /// To read multiple registers, see [`ADS126x::read_multiple_registers`].
    pub fn read_register<SPI>(&mut self, reg: Register, spi: &mut SPI) -> Result<u8>
    where
        SPI: FullDuplex<u8>,
    {
        // zero since number of registers read - 1, so 1-1=0.
        self.send_command(ADCCommand::RREG(reg, 0), spi)?;
        let data = spi.read().map_err(|_| ADS126xError::IO)?;
        Ok(data)
    }

    /// Writes data to multiple registers starting at the provided register.
    /// To write data to a single register, see [`ADS126x::write_register`].
    ///
    /// Data has byte for each register in order registers are written to (increasing address).
    pub fn write_multiple_registers<SPI>(
        &mut self,
        reg: Register,
        data: &[u8],
        spi: &mut SPI,
    ) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        if data.len() > 27 {
            return Err(ADS126xError::InvalidInputData);
        }
        self.send_command(ADCCommand::WREG(reg, data.len() as u8 - 1), spi)?;
        for &byte in data {
            spi.send(byte).map_err(|_| ADS126xError::IO)?;
        }
        Ok(())
    }

    /// Writes data to only the single provided register.
    /// To write data to multiple registers, see [`ADS126x::write_multiple_registers`].
    pub fn write_register<SPI>(&mut self, reg: Register, data: u8, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.send_command(ADCCommand::WREG(reg, 0), spi)?;
        info!("Writing {:#010b} ", data);
        // panic!();
        nb::block!(spi.send(data)).map_err(|_| ADS126xError::IO)
    }

    pub fn get_id<SPI>(&mut self, spi: &mut SPI) -> Result<IdRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::ID, spi)?;
        let data = IdRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn get_power<SPI>(&mut self, spi: &mut SPI) -> Result<PowerRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::POWER, spi)?;
        let data = PowerRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_power<SPI>(&mut self, reg: &PowerRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::POWER, reg.bits(), spi)
    }

    pub fn get_interface<SPI>(&mut self, spi: &mut SPI) -> Result<InterfaceRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::INTERFACE, spi)?;
        let data = InterfaceRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_interface<SPI>(&mut self, reg: &InterfaceRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::INTERFACE, reg.bits(), spi)
    }

    pub fn get_mode0<SPI>(&mut self, spi: &mut SPI) -> Result<Mode0Register>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::MODE0, spi)?;
        let data = Mode0Register::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_mode0<SPI>(&mut self, reg: &Mode0Register, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::MODE0, reg.bits(), spi)
    }

    pub fn get_mode1<SPI>(&mut self, spi: &mut SPI) -> Result<Mode1Register>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::MODE1, spi)?;
        let data = Mode1Register::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_mode1<SPI>(&mut self, reg: &Mode1Register, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        info!("Setting register to {:#010b}", reg.bits());
        self.write_register(Register::MODE1, reg.bits(), spi)
    }

    pub fn get_mode2<SPI>(&mut self, spi: &mut SPI) -> Result<Mode2Register>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::MODE2, spi)?;
        let data = Mode2Register::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_mode2<SPI>(&mut self, reg: &Mode2Register, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::MODE2, reg.bits(), spi)
    }

    pub fn get_inpmux<SPI>(&mut self, spi: &mut SPI) -> Result<InpMuxRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::INPMUX, spi)?;
        let data = InpMuxRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_inpmux<SPI>(&mut self, reg: &InpMuxRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::INPMUX, reg.bits(), spi)
    }

    pub fn get_ofcal<SPI>(&mut self, spi: &mut SPI) -> Result<u32>
    where
        SPI: FullDuplex<u8>,
    {
        let bytes = self.read_multiple_registers(Register::OFCAL0, 3, spi)?; // [OFCAL0, OFCAL1, OFCAL2]
        let res = (bytes[2] as u32) << 16 | (bytes[1] as u32) << 8 | (bytes[0] as u32);
        Ok(res)
    }

    pub fn set_ofcal<SPI>(&mut self, ofcal: u32, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        // Will not panic as & 0xFF ensures values are u8
        let res: [u8; 3] = [
            u8::try_from(ofcal & 0xFF).unwrap(),
            u8::try_from((ofcal >> 8) & 0xFF).unwrap(),
            u8::try_from((ofcal >> 16) & 0xFF).unwrap(),
        ];
        self.write_multiple_registers(Register::OFCAL0, &res, spi)
    }

    pub fn get_fscal<SPI>(&mut self, spi: &mut SPI) -> Result<u32>
    where
        SPI: FullDuplex<u8>,
    {
        let bytes = self.read_multiple_registers(Register::FSCAL0, 3, spi)?; // [FSCAL0, FSCAL1, FSCAL2]
        let res = (bytes[2] as u32) << 16 | (bytes[1] as u32) << 8 | (bytes[0] as u32);
        Ok(res)
    }

    pub fn set_fscal<SPI>(&mut self, fscal: u32, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        // Will not panic as & 0xFF ensures values are u8
        let res: [u8; 3] = [
            u8::try_from(fscal & 0xFF).unwrap(),
            u8::try_from((fscal >> 8) & 0xFF).unwrap(),
            u8::try_from((fscal >> 16) & 0xFF).unwrap(),
        ];
        self.write_multiple_registers(Register::FSCAL0, &res, spi)
    }

    pub fn get_idacmux<SPI>(&mut self, spi: &mut SPI) -> Result<IdacMuxRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::IDACMUX, spi)?;
        let data = IdacMuxRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_idacmux<SPI>(&mut self, reg: &IdacMuxRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::IDACMUX, reg.bits(), spi)
    }

    pub fn get_idacmag<SPI>(&mut self, spi: &mut SPI) -> Result<IdacMagRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::IDACMAG, spi)?;
        let data = IdacMagRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_idacmag<SPI>(&mut self, reg: &IdacMagRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::IDACMAG, reg.bits(), spi)
    }

    pub fn get_refmux<SPI>(&mut self, spi: &mut SPI) -> Result<RefMuxRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::REFMUX, spi)?;
        let data = RefMuxRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_refmux<SPI>(&mut self, reg: &RefMuxRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::REFMUX, reg.bits(), spi)
    }

    pub fn get_tdacp<SPI>(&mut self, spi: &mut SPI) -> Result<TdacpRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::TDACP, spi)?;
        let data = TdacpRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_tdacp<SPI>(&mut self, reg: &TdacpRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::TDACP, reg.bits(), spi)
    }

    pub fn get_tdacn<SPI>(&mut self, spi: &mut SPI) -> Result<TdacnRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::TDACN, spi)?;
        let data = TdacnRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_tdacn<SPI>(&mut self, reg: &TdacnRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::TDACN, reg.bits(), spi)
    }

    pub fn get_gpiocon<SPI>(&mut self, spi: &mut SPI) -> Result<GpioConRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::GPIOCON, spi)?;
        let data = GpioConRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_gpiocon<SPI>(&mut self, reg: &GpioConRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::GPIOCON, reg.bits(), spi)
    }

    pub fn get_gpiodir<SPI>(&mut self, spi: &mut SPI) -> Result<GpioDirRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::GPIODIR, spi)?;
        let data = GpioDirRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_gpiodir<SPI>(&mut self, reg: &GpioDirRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::GPIODIR, reg.bits(), spi)
    }

    pub fn get_gpiodat<SPI>(&mut self, spi: &mut SPI) -> Result<GpioDatRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::GPIODAT, spi)?;
        let data = GpioDatRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_gpiodat<SPI>(&mut self, reg: &GpioDatRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::GPIODAT, reg.bits(), spi)
    }

    pub fn get_adc2cfg<SPI>(&mut self, spi: &mut SPI) -> Result<Adc2CfgRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::ADC2CFG, spi)?;
        let data = Adc2CfgRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_adc2cfg<SPI>(&mut self, reg: &Adc2CfgRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::ADC2CFG, reg.bits(), spi)
    }

    pub fn get_adc2mux<SPI>(&mut self, spi: &mut SPI) -> Result<Adc2MuxRegister>
    where
        SPI: FullDuplex<u8>,
    {
        let bits = self.read_register(Register::ADC2MUX, spi)?;
        let data = Adc2MuxRegister::from_bits(bits);
        match data {
            Some(reg) => Ok(reg),
            None => Err(ADS126xError::InvalidInputData),
        }
    }

    pub fn set_adc2mux<SPI>(&mut self, reg: &Adc2MuxRegister, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        self.write_register(Register::ADC2MUX, reg.bits(), spi)
    }

    pub fn get_adc2ofc<SPI>(&mut self, spi: &mut SPI) -> Result<u16>
    where
        SPI: FullDuplex<u8>,
    {
        let bytes = self.read_multiple_registers(Register::ADC2OFC0, 2, spi)?; // [ADC2OFC0, ADC2OFC1]
        let res = (bytes[1] as u16) << 8 | (bytes[0] as u16);
        Ok(res)
    }

    pub fn set_adc2ofc<SPI>(&mut self, ofc2: u16, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        // Will not panic as & 0xFF ensures values are u8
        let res: [u8; 2] = [
            u8::try_from(ofc2 & 0xFF).unwrap(),
            u8::try_from((ofc2 >> 8) & 0xFF).unwrap(),
        ];
        self.write_multiple_registers(Register::ADC2OFC0, &res, spi)
    }

    pub fn get_adc2fsc<SPI>(&mut self, spi: &mut SPI) -> Result<u16>
    where
        SPI: FullDuplex<u8>,
    {
        let bytes = self.read_multiple_registers(Register::ADC2FSC0, 2, spi)?; // [ADC2FSC0, ADC2FSC1]
        let res = (bytes[1] as u16) << 8 | (bytes[0] as u16);
        Ok(res)
    }

    pub fn set_adc2fsc<SPI>(&mut self, fsc2: u32, spi: &mut SPI) -> Result<()>
    where
        SPI: FullDuplex<u8>,
    {
        // Will not panic as & 0xFF ensures values are u8
        let res: [u8; 2] = [
            u8::try_from(fsc2 & 0xFF).unwrap(),
            u8::try_from((fsc2 >> 8) & 0xFF).unwrap(),
        ];
        self.write_multiple_registers(Register::ADC2FSC0, &res, spi)
    }
}
