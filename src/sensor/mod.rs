pub mod bmi088;
pub mod full_scale;
pub mod icm42688p;
pub mod icm45686;
pub mod imu;
pub mod ism330dlc;

use defmt::error;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_hal::gpio::Output;

use crate::{
    board::{SharedI2c, SharedSpi},
    config::{ImuKind, ImuTransport},
    domain::sensor_reading::SensorReading,
};

pub type SensorChannel = Channel<CriticalSectionRawMutex, SensorReading, 4>;

pub enum Sensor {
    Ism(ism330dlc::Ism330dlc),
    Bmi(bmi088::Bmi088),
    Icm42688p(icm42688p::Icm42688p),
    Icm45686(icm45686::Icm45686),
}

pub use full_scale::{
    AccelFullScale, FullScaleSelection, GyroFullScale, OdrSelection, OutputDataRate, SensorSettings,
};

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Ism(ism330dlc::SensorError),
    Bmi(bmi088::SensorError),
    Icm42688p(icm42688p::SensorError),
    Icm45686(icm45686::SensorError),
    MissingGyroCs,
    UnsupportedTransport,
}

pub async fn init_selected(
    imu_kind: ImuKind,
    settings: SensorSettings,
    transport: ImuTransport,
    i2c_bus: &'static SharedI2c,
    spi_bus: &'static SharedSpi,
    spi_cs_accel: Output<'static>,
    spi_cs_gyro: Option<Output<'static>>,
) -> Result<Sensor, SensorError> {
    match imu_kind {
        ImuKind::Ism330dlc => match transport {
            ImuTransport::Spi => ism330dlc::init_spi(spi_bus, spi_cs_accel, settings)
                .await
                .map(Sensor::Ism)
                .map_err(SensorError::Ism),
            ImuTransport::I2c => ism330dlc::init_i2c(i2c_bus, settings)
                .await
                .map(Sensor::Ism)
                .map_err(SensorError::Ism),
        },
        ImuKind::Bmi088 => {
            if matches!(transport, ImuTransport::I2c) {
                return Err(SensorError::UnsupportedTransport);
            }
            let cs_gyro = spi_cs_gyro.ok_or(SensorError::MissingGyroCs)?;
            bmi088::init_spi(spi_bus, spi_cs_accel, cs_gyro, settings)
                .await
                .map(Sensor::Bmi)
                .map_err(SensorError::Bmi)
        }
        ImuKind::Icm45686 => match transport {
            ImuTransport::Spi => icm45686::init_spi(spi_bus, spi_cs_accel, settings)
                .await
                .map(Sensor::Icm45686)
                .map_err(SensorError::Icm45686),
            ImuTransport::I2c => icm45686::init_i2c(i2c_bus, settings)
                .await
                .map(Sensor::Icm45686)
                .map_err(SensorError::Icm45686),
        },
        ImuKind::Icm42688p => match transport {
            ImuTransport::Spi => icm42688p::init_spi(spi_bus, spi_cs_accel, settings)
                .await
                .map(Sensor::Icm42688p)
                .map_err(SensorError::Icm42688p),
            ImuTransport::I2c => icm42688p::init_i2c(i2c_bus, settings)
                .await
                .map(Sensor::Icm42688p)
                .map_err(SensorError::Icm42688p),
        },
    }
}

pub async fn read_ready(
    sensor: &mut Sensor,
    timestamp_us: u64,
) -> Result<SensorReading, SensorError> {
    match sensor {
        Sensor::Ism(sensor) => ism330dlc::read_ready(sensor, timestamp_us)
            .await
            .map_err(SensorError::Ism),
        Sensor::Bmi(sensor) => bmi088::read_ready(sensor, timestamp_us)
            .await
            .map_err(SensorError::Bmi),
        Sensor::Icm42688p(sensor) => icm42688p::read_ready(sensor, timestamp_us)
            .await
            .map_err(SensorError::Icm42688p),
        Sensor::Icm45686(sensor) => icm45686::read_ready(sensor, timestamp_us)
            .await
            .map_err(SensorError::Icm45686),
    }
}

impl Sensor {
    pub fn full_scale(&self) -> FullScaleSelection {
        self.settings().full_scale
    }

    pub fn odr(&self) -> OdrSelection {
        self.settings().odr
    }

    pub fn settings(&self) -> SensorSettings {
        match self {
            Sensor::Ism(sensor) => sensor.settings(),
            Sensor::Bmi(sensor) => sensor.settings(),
            Sensor::Icm42688p(sensor) => sensor.settings(),
            Sensor::Icm45686(sensor) => sensor.settings(),
        }
    }
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Ism(error) => ism330dlc::log_error(error),
        SensorError::Bmi(error) => bmi088::log_error(error),
        SensorError::Icm42688p(error) => icm42688p::log_error(error),
        SensorError::Icm45686(error) => icm45686::log_error(error),
        SensorError::MissingGyroCs => {
            error!("BMI088 requires a dedicated gyro CS pin");
        }
        SensorError::UnsupportedTransport => {
            error!("Selected IMU does not support the configured transport");
        }
    }
}
