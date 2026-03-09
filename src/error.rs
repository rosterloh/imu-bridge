use core::fmt::{self, Display, Formatter};

#[derive(Debug, defmt::Format)]
pub enum Error {
    EspRadioInit(esp_radio::InitializationError),
    EspRadioWifi(esp_radio::wifi::WifiError),
    ZError(zenoh_nostd::session::Error),
    I2c(esp_hal::i2c::master::Error),
    Format,
    Write,
    Capacity(heapless::CapacityError),
    SensorError(crate::sensor::SensorError),
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Error::EspRadioInit(err) => write!(f, "esp radio initialization failed: {err:?}"),
            Error::EspRadioWifi(err) => write!(f, "esp radio wifi error: {err:?}"),
            Error::ZError(err) => write!(f, "zenoh error: {err:?}"),
            Error::I2c(_) => f.write_str("I2c error TODO"),
            Error::Format => f.write_str("core::fmt::Error error"),
            Error::Write => f.write_str("core::fmt::Write error"),
            Error::Capacity(err) => write!(f, "string capacity error: {err:?}"),
            Error::SensorError(err) => write!(f, "sensor error: {err:?}"),
        }
    }
}

impl From<esp_radio::InitializationError> for Error {
    fn from(e: esp_radio::InitializationError) -> Self {
        Error::EspRadioInit(e)
    }
}

impl From<esp_radio::wifi::WifiError> for Error {
    fn from(e: esp_radio::wifi::WifiError) -> Self {
        Error::EspRadioWifi(e)
    }
}

impl From<zenoh_nostd::session::Error> for Error {
    fn from(e: zenoh_nostd::session::Error) -> Self {
        Error::ZError(e)
    }
}

impl From<heapless::CapacityError> for Error {
    fn from(e: heapless::CapacityError) -> Self {
        Error::Capacity(e)
    }
}

impl From<core::fmt::Error> for Error {
    fn from(_e: core::fmt::Error) -> Self {
        Error::Format
    }
}

impl From<crate::sensor::SensorError> for Error {
    fn from(e: crate::sensor::SensorError) -> Self {
        Error::SensorError(e)
    }
}
