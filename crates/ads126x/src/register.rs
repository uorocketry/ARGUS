mod enums;

use bitflags::bitflags;
pub use enums::*;

#[repr(u8)]
pub enum Register {
    ID = 0x00,
    POWER = 0x01,
    INTERFACE = 0x02,
    MODE0 = 0x03,
    MODE1 = 0x04,
    MODE2 = 0x05,
    INPMUX = 0x06,
    OFCAL0 = 0x07,
    OFCAL1 = 0x08,
    OFCAL2 = 0x09,
    FSCAL0 = 0x0A,
    FSCAL1 = 0x0B,
    FSCAL2 = 0x0C,
    IDACMUX = 0x0D,
    IDACMAG = 0x0E,
    REFMUX = 0x0F,
    TDACP = 0x10,
    TDACN = 0x11,
    GPIOCON = 0x12,
    GPIODIR = 0x13,
    GPIODAT = 0x14,
    ADC2CFG = 0x15,
    ADC2MUX = 0x16,
    ADC2OFC0 = 0x17,
    ADC2OFC1 = 0x18,
    ADC2FSC0 = 0x19,
    ADC2FSC1 = 0x1A,
}

bitflags! {
    pub struct StatusRegister: u8 {
        const ADC2      = 0b1000_0000;
        const ADC1      = 0b0100_0000;
        const EXTCLK    = 0b0010_0000;
        const REF_ALM   = 0b0001_0000;
        const PGAL_ALM  = 0b0000_1000;
        const PGAH_ALM  = 0b0000_0100;
        const PGAD_ALM  = 0b0000_0010;
        const RESET     = 0b0000_0001;
    }
}

impl StatusRegister {
    pub fn get_adc1_new(&self) -> bool {
        self.contains(StatusRegister::ADC1)
    }
    pub fn get_adc2_new(&self) -> bool {
        self.contains(StatusRegister::ADC2)
    }
    pub fn get_extclk(&self) -> bool {
        self.contains(StatusRegister::EXTCLK)
    }
    pub fn get_ref_alarm(&self) -> bool {
        self.contains(StatusRegister::REF_ALM)
    }
    pub fn get_pga_low_alarm(&self) -> bool {
        self.contains(StatusRegister::PGAL_ALM)
    }
    pub fn get_pga_high_alarm(&self) -> bool {
        self.contains(StatusRegister::PGAH_ALM)
    }
    pub fn get_pga_diff_alarm(&self) -> bool {
        self.contains(StatusRegister::PGAD_ALM)
    }
    /// The user needs to clear this byte from the Power register.
    pub fn get_reset(&self) -> bool {
        self.contains(StatusRegister::RESET)
    }
}

bitflags! {
    pub struct IdRegister: u8 {
        const _ = !0; // Source may set any bits
    }
}

impl IdRegister {
    pub fn get_rev_id(&self) -> u8 {
        self.bits() & 0b0001_1111
    }

    pub fn get_dev_id(&self) -> DevId {
        match (self.bits() & 0b1110_0000) >> 5 {
            0b000 => DevId::ADS1262,
            0b001 => DevId::ADS1263,

            _ => panic!("Device ID must be 0b000 or 0b001."),
        }
    }
}

bitflags! {
    pub struct PowerRegister: u8 {
        const INTREF = 0b0000_0001;
        const VBIAS  = 0b0000_0010;
        const RESET  = 0b0001_0000;
    }
}

impl PowerRegister {
    pub fn default() -> Self {
        PowerRegister::from_bits_truncate(0b0001_0001)
    }

    pub fn set_vbias(&mut self, vbias: bool) {
        if vbias {
            self.insert(PowerRegister::VBIAS);
        } else {
            self.remove(PowerRegister::VBIAS);
        }
    }

    pub fn set_intref(&mut self, intref: bool) {
        if intref {
            self.insert(PowerRegister::INTREF);
        } else {
            self.remove(PowerRegister::INTREF);
        }
    }

    pub fn get_reset(&self) -> bool {
        self.contains(PowerRegister::RESET)
    }

    // must be cleared if set to detect further resets.
    pub fn clear_reset(&mut self) {
        self.remove(PowerRegister::RESET);
    }
}

bitflags! {
    /// WARNING: If CRC is 0b11 set by ADC, it will reflect as CRC enabled not reserved.
    /// CRC only accounts for 0b00 disabled and 0b01 enabled.
    pub struct InterfaceRegister: u8 {
        const CRC     = 0b0000_0001;
        const STATUS  = 0b0000_0100;
        const TIMEOUT = 0b0000_1000;
    }
}

impl InterfaceRegister {
    pub fn default() -> Self {
        InterfaceRegister::from_bits_truncate(0b0000_0101)
    }

    pub fn get_crc(&self) -> CrcMode {
        match self.bits() & 0b0000_0001 {
            0b00 => CrcMode::Disabled,
            0b01 => CrcMode::Checksum,
            0b10 => CrcMode::CRC,
            _ => unreachable!(),
        }
    }

    pub fn set_crc(&mut self, crc: CrcMode) {
        let bits = crc as u8;
        self.insert(InterfaceRegister::from_bits_retain(bits));
    }

    pub fn get_status(&self) -> bool {
        self.contains(InterfaceRegister::STATUS)
    }

    pub fn set_status(&mut self, status: bool) {
        if status {
            self.insert(InterfaceRegister::STATUS);
        } else {
            self.remove(InterfaceRegister::STATUS);
        }
    }

    pub fn get_timeout(&self) -> bool {
        self.contains(InterfaceRegister::TIMEOUT)
    }

    pub fn set_timeout(&mut self, timeout: bool) {
        if timeout {
            self.insert(InterfaceRegister::TIMEOUT);
        } else {
            self.remove(InterfaceRegister::TIMEOUT);
        }
    }
}

bitflags! {
    pub struct Mode0Register: u8 {
        const RUNMODE = 0b0100_0000;
        const REFREV  = 0b1000_0000;

        const _ = !0; // Source may set any bits
    }
}

impl Mode0Register {
    pub fn default() -> Self {
        Mode0Register::from_bits_truncate(0b0000_0000)
    }

    pub fn get_delay(&self) -> ConversionDelay {
        match self.bits() & 0b0000_1111 {
            0b0000 => ConversionDelay::DNone,
            0b0001 => ConversionDelay::D8_7us,
            0b0010 => ConversionDelay::D17us,
            0b0011 => ConversionDelay::D35us,
            0b0100 => ConversionDelay::D69us,
            0b0101 => ConversionDelay::D139us,
            0b0110 => ConversionDelay::D278us,
            0b0111 => ConversionDelay::D555us,
            0b1000 => ConversionDelay::D1_1ms,
            0b1001 => ConversionDelay::D2_2ms,
            0b1010 => ConversionDelay::D4_4ms,
            0b1011 => ConversionDelay::D8_8ms,

            0b1100..=0b1111 => panic!("Unknown conversion delay"),
            _ => unreachable!(),
        }
    }

    pub fn set_delay(&mut self, delay: ConversionDelay) {
        let bits = delay as u8;
        self.insert(Mode0Register::from_bits_retain(bits));
    }

    pub fn get_chop(&self) -> ChopMode {
        match (self.bits() & 0b0011_0000) >> 4 {
            0b00 => ChopMode::Disabled,
            0b01 => ChopMode::InChopEnabled,
            0b10 => ChopMode::IdacEnabled,
            0b11 => ChopMode::InChopAndIdacEnabled,

            _ => unreachable!(),
        }
    }

    pub fn set_chop(&mut self, chop: ChopMode) {
        let bits = chop as u8;
        self.insert(Mode0Register::from_bits_retain(bits << 4));
    }
}

bitflags! {
    pub struct Mode1Register: u8 {
        const SBPOL = 0b0000_1000;
        const SBADC = 0b0001_0000;

        const _ = !0; // Source may set any bits
    }
}

impl Mode1Register {
    pub fn default() -> Self {
        Mode1Register::from_bits_truncate(0b1000_0000)
    }

    pub fn get_sbmag(&self) -> SensorBiasMagnitude {
        match self.bits() & 0b0000_0111 {
            0b000 => SensorBiasMagnitude::BNone,
            0b001 => SensorBiasMagnitude::B0_5uA,
            0b010 => SensorBiasMagnitude::B2uA,
            0b011 => SensorBiasMagnitude::B10uA,
            0b100 => SensorBiasMagnitude::B50uA,
            0b101 => SensorBiasMagnitude::B200uA,
            0b110 => SensorBiasMagnitude::R10MOhm,

            0b111 => panic!("Reserved SBMAG"),
            _ => unreachable!(),
        }
    }

    pub fn set_sbmag(&mut self, sbmag: SensorBiasMagnitude) {
        let bits = sbmag as u8;
        self.insert(Mode1Register::from_bits_retain(bits));
    }

    pub fn get_filter(&self) -> DigitalFilter {
        match (self.bits() & 0b1110_0000) >> 5 {
            0b000 => DigitalFilter::Sinc1,
            0b001 => DigitalFilter::Sinc2,
            0b010 => DigitalFilter::Sinc3,
            0b011 => DigitalFilter::Sinc4,
            0b100 => DigitalFilter::FIR,

            0b101..=0b111 => panic!("Reserved filter"),
            _ => unreachable!(),
        }
    }

    pub fn set_filter(&mut self, filter: DigitalFilter) {
        let bits = filter as u8;
        self.insert(Mode1Register::from_bits_retain(bits << 5));
    }
}

bitflags! {
    pub struct Mode2Register: u8 {
        const BYPASS = 0b1000_0000;

        const _ = !0; // Source may set any bits
    }
}

impl Mode2Register {
    pub fn default() -> Self {
        Mode2Register::from_bits_truncate(0b0000_0100)
    }

    pub fn get_dr(&self) -> DataRate {
        match self.bits() & 0b0000_1111 {
            0b0000 => DataRate::SPS2_5,
            0b0001 => DataRate::SPS5,
            0b0010 => DataRate::SPS10,
            0b0011 => DataRate::SPS16_6,
            0b0100 => DataRate::SPS20,
            0b0101 => DataRate::SPS50,
            0b0110 => DataRate::SPS60,
            0b0111 => DataRate::SPS100,
            0b1000 => DataRate::SPS400,
            0b1001 => DataRate::SPS1200,
            0b1010 => DataRate::SPS2400,
            0b1011 => DataRate::SPS4800,
            0b1100 => DataRate::SPS7200,
            0b1101 => DataRate::SPS14400,
            0b1110 => DataRate::SPS19200,
            0b1111 => DataRate::SPS38400,

            _ => unreachable!(),
        }
    }

    pub fn set_dr(&mut self, rate: DataRate) {
        let bits = rate as u8;
        self.insert(Mode2Register::from_bits_retain(bits));
    }

    pub fn get_gain(&self) -> PGAGain {
        match (self.bits() & 0b0111_0000) >> 4 {
            0b000 => PGAGain::VV1,
            0b001 => PGAGain::VV2,
            0b010 => PGAGain::VV4,
            0b011 => PGAGain::VV8,
            0b100 => PGAGain::VV16,
            0b101 => PGAGain::VV32,

            0b110 | 0b111 => panic!("Reserved gain"),
            _ => unreachable!(),
        }
    }

    pub fn set_gain(&mut self, gain: PGAGain) {
        let bits = gain as u8;
        self.insert(Mode2Register::from_bits_retain(bits << 4));
    }
}

bitflags! {
    pub struct InpMuxRegister: u8 {
        const _ = !0; // Source may set any bits
    }
}

impl InpMuxRegister {
    pub fn default() -> Self {
        InpMuxRegister::from_bits_truncate(0b0000_0001)
    }

    pub fn get_muxn(&self) -> NegativeInpMux {
        match self.bits() & 0b0000_1111 {
            0b0000 => NegativeInpMux::AIN0,
            0b0001 => NegativeInpMux::AIN1,
            0b0010 => NegativeInpMux::AIN2,
            0b0011 => NegativeInpMux::AIN3,
            0b0100 => NegativeInpMux::AIN4,
            0b0101 => NegativeInpMux::AIN5,
            0b0110 => NegativeInpMux::AIN6,
            0b0111 => NegativeInpMux::AIN7,
            0b1000 => NegativeInpMux::AIN8,
            0b1001 => NegativeInpMux::AIN9,
            0b1010 => NegativeInpMux::AINCOM,
            0b1011 => NegativeInpMux::TempSensMonNeg,
            0b1100 => NegativeInpMux::AnlgPwrSupMonNeg,
            0b1101 => NegativeInpMux::DgtlPwrSubMonNeg,
            0b1110 => NegativeInpMux::TDACTestSignalNeg,
            0b1111 => NegativeInpMux::Float,

            _ => unreachable!(),
        }
    }

    pub fn set_muxn(&mut self, muxn: NegativeInpMux) {
        let bits = muxn as u8;
        self.insert(InpMuxRegister::from_bits_retain(bits));
    }

    pub fn get_muxp(&self) -> PositiveInpMux {
        match (self.bits() & 0b1111_0000) >> 4 {
            0b0000 => PositiveInpMux::AIN0,
            0b0001 => PositiveInpMux::AIN1,
            0b0010 => PositiveInpMux::AIN2,
            0b0011 => PositiveInpMux::AIN3,
            0b0100 => PositiveInpMux::AIN4,
            0b0101 => PositiveInpMux::AIN5,
            0b0110 => PositiveInpMux::AIN6,
            0b0111 => PositiveInpMux::AIN7,
            0b1000 => PositiveInpMux::AIN8,
            0b1001 => PositiveInpMux::AIN9,
            0b1010 => PositiveInpMux::AINCOM,
            0b1011 => PositiveInpMux::TempSensMonPos,
            0b1100 => PositiveInpMux::AnlgPwrSupMonPos,
            0b1101 => PositiveInpMux::DgtlPwrSubMonPos,
            0b1110 => PositiveInpMux::TDACTestSignalPos,
            0b1111 => PositiveInpMux::Float,

            _ => unreachable!(),
        }
    }

    pub fn set_muxp(&mut self, muxp: PositiveInpMux) {
        let bits = muxp as u8;
        self.insert(InpMuxRegister::from_bits_retain(bits << 4));
    }
}

bitflags! {
    pub struct IdacMuxRegister: u8 {
        const _ = !0; // Source may set any bits
    }
}

impl IdacMuxRegister {
    pub fn default() -> Self {
        IdacMuxRegister::from_bits_truncate(0b1011_1011)
    }

    pub fn get_mux1(&self) -> IdacOutMux {
        match self.bits() & 0b0000_1111 {
            0b0000 => IdacOutMux::AIN0,
            0b0001 => IdacOutMux::AIN1,
            0b0010 => IdacOutMux::AIN2,
            0b0011 => IdacOutMux::AIN3,
            0b0100 => IdacOutMux::AIN4,
            0b0101 => IdacOutMux::AIN5,
            0b0110 => IdacOutMux::AIN6,
            0b0111 => IdacOutMux::AIN7,
            0b1000 => IdacOutMux::AIN8,
            0b1001 => IdacOutMux::AIN9,
            0b1010 => IdacOutMux::AINCOM,
            0b1011 => IdacOutMux::NoConnection,

            0b1100..=0b1111 => panic!("Reserved IDAC Output Multiplexer"),
            _ => unreachable!(),
        }
    }

    pub fn set_mux1(&mut self, mux1: IdacOutMux) {
        let bits = mux1 as u8;
        self.insert(IdacMuxRegister::from_bits_retain(bits));
    }

    pub fn get_mux2(&self) -> IdacOutMux {
        match (self.bits() & 0b1111_0000) >> 4 {
            0b0000 => IdacOutMux::AIN0,
            0b0001 => IdacOutMux::AIN1,
            0b0010 => IdacOutMux::AIN2,
            0b0011 => IdacOutMux::AIN3,
            0b0100 => IdacOutMux::AIN4,
            0b0101 => IdacOutMux::AIN5,
            0b0110 => IdacOutMux::AIN6,
            0b0111 => IdacOutMux::AIN7,
            0b1000 => IdacOutMux::AIN8,
            0b1001 => IdacOutMux::AIN9,
            0b1010 => IdacOutMux::AINCOM,
            0b1011 => IdacOutMux::NoConnection,

            0b1100..=0b1111 => panic!("Reserved IDAC Output Multiplexer"),
            _ => unreachable!(),
        }
    }

    pub fn set_mux2(&mut self, mux2: IdacOutMux) {
        let bits = mux2 as u8;
        self.insert(IdacMuxRegister::from_bits_retain(bits << 4));
    }
}

bitflags! {
    pub struct IdacMagRegister: u8 {
        const _ = !0; // Source may set any bits
    }
}

impl IdacMagRegister {
    pub fn default() -> Self {
        IdacMagRegister::from_bits_truncate(0b0000_0000)
    }

    pub fn get_mag1(&self) -> IdacCurMag {
        match self.bits() & 0b0000_1111 {
            0b0000 => IdacCurMag::I50uA,
            0b0001 => IdacCurMag::I100uA,
            0b0010 => IdacCurMag::I250uA,
            0b0011 => IdacCurMag::I500uA,
            0b0100 => IdacCurMag::I750uA,
            0b0101 => IdacCurMag::I1000uA,
            0b0110 => IdacCurMag::I1500uA,
            0b0111 => IdacCurMag::I2000uA,
            0b1000 => IdacCurMag::I2500uA,
            0b1001 => IdacCurMag::I3000uA,

            0b1010..=0b1111 => panic!("Reserved IDAC Magnitude Multiplexer"),
            _ => unreachable!(),
        }
    }

    pub fn set_mag1(&mut self, mag1: IdacCurMag) {
        let bits = mag1 as u8;
        self.insert(IdacMagRegister::from_bits_retain(bits));
    }

    pub fn get_mag2(&self) -> IdacCurMag {
        match (self.bits() & 0b1111_0000) >> 4 {
            0b0000 => IdacCurMag::I50uA,
            0b0001 => IdacCurMag::I100uA,
            0b0010 => IdacCurMag::I250uA,
            0b0011 => IdacCurMag::I500uA,
            0b0100 => IdacCurMag::I750uA,
            0b0101 => IdacCurMag::I1000uA,
            0b0110 => IdacCurMag::I1500uA,
            0b0111 => IdacCurMag::I2000uA,
            0b1000 => IdacCurMag::I2500uA,
            0b1001 => IdacCurMag::I3000uA,

            0b1010..=0b1111 => panic!("Reserved IDAC Output Multiplexer"),
            _ => unreachable!(),
        }
    }

    pub fn set_mag2(&mut self, mag2: IdacCurMag) {
        let bits = mag2 as u8;
        self.insert(IdacMagRegister::from_bits_retain(bits << 4));
    }
}

bitflags! {
    pub struct RefMuxRegister: u8 {
        const _ = 0b0011_1111;
    }
}

impl RefMuxRegister {
    pub fn default() -> Self {
        RefMuxRegister::from_bits_truncate(0b0000_0000)
    }

    pub fn get_rmuxn(&self) -> RefNegativeInp {
        match self.bits() & 0b0000_0111 {
            0b000 => RefNegativeInp::Int2_5VRef,
            0b001 => RefNegativeInp::ExtAIN1,
            0b010 => RefNegativeInp::ExtAIN3,
            0b011 => RefNegativeInp::ExtAIN5,
            0b100 => RefNegativeInp::IntAnlgSup,

            0b101..=0b111 => panic!("Reserved Reference Negative Input"),
            _ => unreachable!(),
        }
    }

    pub fn set_rmuxn(&mut self, rmuxn: RefNegativeInp) {
        let bits = rmuxn as u8;
        self.insert(RefMuxRegister::from_bits_retain(bits));
    }

    pub fn get_rmuxp(&self) -> RefPositiveInp {
        match (self.bits() & 0b0011_1000) >> 3 {
            0b000 => RefPositiveInp::Int2_5VRef,
            0b001 => RefPositiveInp::ExtAIN0,
            0b010 => RefPositiveInp::ExtAIN2,
            0b011 => RefPositiveInp::ExtAIN4,
            0b100 => RefPositiveInp::IntAnlgSup,

            0b101..=0b111 => panic!("Reserved Reference Positive Input"),
            _ => unreachable!(),
        }
    }

    pub fn set_rmuxp(&mut self, rmuxp: RefPositiveInp) {
        let bits = rmuxp as u8;
        self.insert(RefMuxRegister::from_bits_retain(bits << 3));
    }
}

bitflags! {
    pub struct TdacpRegister: u8 {
        const OUTP = 0b1000_0000;

        const _ = 0b1001_1111;
    }
}

impl TdacpRegister {
    // there is no defined default values in the datasheet so just zeros.
    pub fn default() -> Self {
        TdacpRegister::from_bits_truncate(0b0000_0000)
    }

    pub fn get_magp(&self) -> TdacOutMag {
        match self.bits() & 0b0001_1111 {
            0b01001 => TdacOutMag::V4_5,
            0b01000 => TdacOutMag::V3_5,
            0b00111 => TdacOutMag::V3,
            0b00110 => TdacOutMag::V2_75,
            0b00101 => TdacOutMag::V2_625,
            0b00100 => TdacOutMag::V2_5625,
            0b00011 => TdacOutMag::V2_53125,
            0b00010 => TdacOutMag::V2_515625,
            0b00001 => TdacOutMag::V2_5078125,
            0b00000 => TdacOutMag::V2_5,
            0b10001 => TdacOutMag::V2_4921875,
            0b10010 => TdacOutMag::V2_484375,
            0b10011 => TdacOutMag::V2_46875,
            0b10100 => TdacOutMag::V2_4375,
            0b10101 => TdacOutMag::V2_375,
            0b10110 => TdacOutMag::V2_25,
            0b10111 => TdacOutMag::V2,
            0b11000 => TdacOutMag::V1_5,
            0b11001 => TdacOutMag::V0_5,

            0b01010..=0b10000 | 0b11010..=0b11111 => panic!("Reserved MAGP"),
            _ => unreachable!(),
        }
    }

    pub fn set_magp(&mut self, magp: TdacOutMag) {
        let bits = magp as u8;
        self.insert(TdacpRegister::from_bits_retain(bits));
    }
}

bitflags! {
    pub struct TdacnRegister: u8 {
        const OUTN = 0b1000_0000;

        const _ = 0b1001_1111;
    }
}

impl TdacnRegister {
    // there is no defined default values in the datasheet so just zeros.
    pub fn default() -> Self {
        TdacnRegister::from_bits_truncate(0b0000_0000)
    }

    pub fn get_magn(&self) -> TdacOutMag {
        match self.bits() & 0b0001_1111 {
            0b01001 => TdacOutMag::V4_5,
            0b01000 => TdacOutMag::V3_5,
            0b00111 => TdacOutMag::V3,
            0b00110 => TdacOutMag::V2_75,
            0b00101 => TdacOutMag::V2_625,
            0b00100 => TdacOutMag::V2_5625,
            0b00011 => TdacOutMag::V2_53125,
            0b00010 => TdacOutMag::V2_515625,
            0b00001 => TdacOutMag::V2_5078125,
            0b00000 => TdacOutMag::V2_5,
            0b10001 => TdacOutMag::V2_4921875,
            0b10010 => TdacOutMag::V2_484375,
            0b10011 => TdacOutMag::V2_46875,
            0b10100 => TdacOutMag::V2_4375,
            0b10101 => TdacOutMag::V2_375,
            0b10110 => TdacOutMag::V2_25,
            0b10111 => TdacOutMag::V2,
            0b11000 => TdacOutMag::V1_5,
            0b11001 => TdacOutMag::V0_5,

            0b01010..=0b10000 | 0b11010..=0b11111 => panic!("Reserved MAGN"),
            _ => unreachable!(),
        }
    }

    pub fn set_magn(&mut self, magn: TdacOutMag) {
        let bits = magn as u8;
        self.insert(TdacnRegister::from_bits_retain(bits));
    }
}

bitflags! {
    pub struct GpioConRegister: u8 {
        const CON0 = 0b0000_0001; // GPIO[0] -> AIN3
        const CON1 = 0b0000_0010; // GPIO[1] -> AIN4
        const CON2 = 0b0000_0100; // GPIO[2] -> AIN5
        const CON3 = 0b0000_1000; // GPIO[3] -> AIN6
        const CON4 = 0b0001_0000; // GPIO[4] -> AIN7
        const CON5 = 0b0010_0000; // GPIO[5] -> AIN8
        const CON6 = 0b0100_0000; // GPIO[6] -> AIN9
        const CON7 = 0b1000_0000; // GPIO[7] -> AINCOM
    }
}

impl GpioConRegister {
    pub fn default() -> Self {
        GpioConRegister::from_bits_truncate(0b0000_0000)
    }
}

bitflags! {
    /// Setting `DIR<x>` to:
    /// - 0 = `GPIO<x>` is output
    /// - 1 = `GPIO<x>` is input
    pub struct GpioDirRegister: u8 {
        const DIR0 = 0b0000_0001;
        const DIR1 = 0b0000_0010;
        const DIR2 = 0b0000_0100;
        const DIR3 = 0b0000_1000;
        const DIR4 = 0b0001_0000;
        const DIR5 = 0b0010_0000;
        const DIR6 = 0b0100_0000;
        const DIR7 = 0b1000_0000;
    }
}

impl GpioDirRegister {
    pub fn default() -> Self {
        GpioDirRegister::from_bits_truncate(0b0000_0000)
    }
}

bitflags! {
    /// If `GPIO<x>` is output, read returns 0b.
    /// If `GPIO<x>` is input, write sets `GPIO<x>` to high (if 1) or low (if 0).
    pub struct GpioDatRegister: u8 {
        const DAT0 = 0b0000_0001;
        const DAT1 = 0b0000_0010;
        const DAT2 = 0b0000_0100;
        const DAT3 = 0b0000_1000;
        const DAT4 = 0b0001_0000;
        const DAT5 = 0b0010_0000;
        const DAT6 = 0b0100_0000;
        const DAT7 = 0b1000_0000;
    }
}

impl GpioDatRegister {
    // there is no defined default values in the datasheet so just zeros.
    pub fn default() -> Self {
        GpioDatRegister::from_bits_truncate(0b0000_0000)
    }
}

bitflags! {
    pub struct Adc2CfgRegister: u8 {
        const _ = !0; // Source may set any bits
    }
}

impl Adc2CfgRegister {
    pub fn default() -> Self {
        Adc2CfgRegister::from_bits_truncate(0b0000_0000)
    }

    pub fn get_gain2(&self) -> Adc2Gain {
        match self.bits() & 0b0000_0111 {
            0b000 => Adc2Gain::VV1,
            0b001 => Adc2Gain::VV2,
            0b010 => Adc2Gain::VV4,
            0b011 => Adc2Gain::VV8,
            0b100 => Adc2Gain::VV16,
            0b101 => Adc2Gain::VV32,
            0b110 => Adc2Gain::VV64,
            0b111 => Adc2Gain::VV128,

            _ => unreachable!(),
        }
    }

    pub fn set_gain2(&mut self, gain2: Adc2Gain) {
        let bits = gain2 as u8;
        self.insert(Adc2CfgRegister::from_bits_retain(bits));
    }

    pub fn get_ref2(&self) -> Adc2RefInp {
        match (self.bits() & 0b0011_1000) >> 3 {
            0b000 => Adc2RefInp::Int2_5VRef,
            0b001 => Adc2RefInp::ExtAIN0_1,
            0b010 => Adc2RefInp::ExtAIN2_3,
            0b011 => Adc2RefInp::ExtAIN4_5,
            0b100 => Adc2RefInp::IntAnlgSup,

            0b101..=0b111 => panic!("Reserved ADC2 reference input"),
            _ => unreachable!(),
        }
    }

    pub fn set_ref2(&mut self, ref2: Adc2RefInp) {
        let bits = ref2 as u8;
        self.insert(Adc2CfgRegister::from_bits_retain(bits << 3));
    }

    pub fn get_dr2(&self) -> Adc2DataRate {
        match (self.bits() & 0b1100_0000) >> 6 {
            0b00 => Adc2DataRate::SPS10,
            0b01 => Adc2DataRate::SPS100,
            0b10 => Adc2DataRate::SPS400,
            0b11 => Adc2DataRate::SPS800,

            _ => unreachable!(),
        }
    }

    pub fn set_dr2(&mut self, dr2: Adc2DataRate) {
        let bits = dr2 as u8;
        self.insert(Adc2CfgRegister::from_bits_retain(bits << 6));
    }
}

bitflags! {
    pub struct Adc2MuxRegister: u8 {
        const _ = !0; // Source may set any bits
    }
}

impl Adc2MuxRegister {
    pub fn default() -> Self {
        Adc2MuxRegister::from_bits_truncate(0b0000_0001)
    }

    pub fn get_muxn2(&self) -> NegativeInpMux {
        match self.bits() & 0b0000_1111 {
            0b0000 => NegativeInpMux::AIN0,
            0b0001 => NegativeInpMux::AIN1,
            0b0010 => NegativeInpMux::AIN2,
            0b0011 => NegativeInpMux::AIN3,
            0b0100 => NegativeInpMux::AIN4,
            0b0101 => NegativeInpMux::AIN5,
            0b0110 => NegativeInpMux::AIN6,
            0b0111 => NegativeInpMux::AIN7,
            0b1000 => NegativeInpMux::AIN8,
            0b1001 => NegativeInpMux::AIN9,
            0b1010 => NegativeInpMux::AINCOM,
            0b1011 => NegativeInpMux::TempSensMonNeg,
            0b1100 => NegativeInpMux::AnlgPwrSupMonNeg,
            0b1101 => NegativeInpMux::DgtlPwrSubMonNeg,
            0b1110 => NegativeInpMux::TDACTestSignalNeg,
            0b1111 => NegativeInpMux::Float,

            _ => unreachable!(),
        }
    }

    pub fn set_muxn2(&mut self, muxn2: NegativeInpMux) {
        let bits = muxn2 as u8;
        self.insert(Adc2MuxRegister::from_bits_retain(bits));
    }

    pub fn get_muxp2(&self) -> PositiveInpMux {
        match (self.bits() & 0b1111_0000) >> 4 {
            0b0000 => PositiveInpMux::AIN0,
            0b0001 => PositiveInpMux::AIN1,
            0b0010 => PositiveInpMux::AIN2,
            0b0011 => PositiveInpMux::AIN3,
            0b0100 => PositiveInpMux::AIN4,
            0b0101 => PositiveInpMux::AIN5,
            0b0110 => PositiveInpMux::AIN6,
            0b0111 => PositiveInpMux::AIN7,
            0b1000 => PositiveInpMux::AIN8,
            0b1001 => PositiveInpMux::AIN9,
            0b1010 => PositiveInpMux::AINCOM,
            0b1011 => PositiveInpMux::TempSensMonPos,
            0b1100 => PositiveInpMux::AnlgPwrSupMonPos,
            0b1101 => PositiveInpMux::DgtlPwrSubMonPos,
            0b1110 => PositiveInpMux::TDACTestSignalPos,
            0b1111 => PositiveInpMux::Float,

            _ => unreachable!(),
        }
    }

    pub fn set_muxp2(&mut self, muxp2: PositiveInpMux) {
        let bits = muxp2 as u8;
        self.insert(Adc2MuxRegister::from_bits_retain(bits << 4));
    }
}
