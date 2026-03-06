#[derive(Debug, thiserror::Error, defmt::Format)]
pub enum Error {
    #[error("esp radio initialization failed: {0:?}")]
    EspRadioInit(esp_radio::InitializationError),

    #[error("esp radio wifi error: {0:?}")]
    EspRadioWifi(esp_radio::wifi::WifiError),

    #[error("zenoh error: {0:?}")]
    ZError(zenoh_nostd::session::Error),

    // #[error("zenoh protocol error: {0:?}")]
    // ZProtocol(zenoh_nostd::ZProtocolError),

    // #[error("zenoh keyexpr error: {0:?}")]
    // ZKeyExpr(zenoh_nostd::ZKeyExprError),
    #[error("I2c error TODO")]
    I2c(esp_hal::i2c::master::Error),

    #[error("core::fmt::Error error")]
    Format,

    #[error("core::fmt::Write error")]
    Write,

    #[error("string capacity error: {0:?}")]
    Capacity(heapless::CapacityError),
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

// impl From<zenoh_nostd::ZError> for Error {
//     fn from(e: zenoh_nostd::ZError) -> Self {
//         Error::ZError(e)
//     }
// }

// impl From<zenoh_nostd::ZProtocolError> for Error {
//     fn from(e: zenoh_nostd::ZProtocolError) -> Self {
//         Error::ZProtocol(e)
//     }
// }

// impl From<zenoh_nostd::ZKeyExprError> for Error {
//     fn from(e: zenoh_nostd::ZKeyExprError) -> Self {
//         Error::ZKeyExpr(e)
//     }
// }

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
