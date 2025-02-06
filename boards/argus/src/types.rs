use ads126x::register::{NegativeInpMux, PositiveInpMux};
use messages::{
    node::Node::{self, PressureBoard, StrainBoard, TemperatureBoard},
    sensor::AdcSensor,
};

pub enum Feature {
    Temperature,
    Pressure,
    Strain,
}

pub const FEATURE: Feature = if cfg!(feature = "temperature") {
    Feature::Temperature
} else if cfg!(feature = "pressure") {
    Feature::Pressure
} else if cfg!(feature = "strain") {
    Feature::Strain
} else {
    Feature::Pressure // dummy return the code will not compile in this case.
};

pub const ADC2_RST_PIN_PORT: char = match FEATURE {
    Feature::Temperature => 'E',
    Feature::Pressure => 'D',
    Feature::Strain => 'B',
};

pub const ADC2_RST_PIN_ID: u8 = match FEATURE {
    Feature::Temperature => 0,
    Feature::Pressure => 1,
    Feature::Strain => 9,
};

pub static COM_ID: Node = match FEATURE {
    Feature::Temperature => TemperatureBoard,
    Feature::Pressure => PressureBoard,
    Feature::Strain => StrainBoard,
};
