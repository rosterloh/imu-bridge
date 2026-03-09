pub mod bmi088;
pub mod icm45686;
pub mod ism330dlc;
pub mod imu;

use defmt::error;
use esp_hal::gpio::Output;

use crate::{
    board::{SharedI2c, SharedSpi},
    config::ImuKind,
    domain::sensor_reading::SensorReading,
};

pub enum Sensor {
    Ism(ism330dlc::Ism330dlc),
    Bmi(bmi088::Bmi088),
    Icm45686(icm45686::Icm45686),
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Ism(ism330dlc::SensorError),
    Bmi(bmi088::SensorError),
    Icm45686(icm45686::SensorError),
    MissingGyroCs,
}

pub async fn init_selected(
    imu_kind: ImuKind,
    i2c_bus: &'static SharedI2c,
    spi_bus: &'static SharedSpi,
    spi_cs_accel: Output<'static>,
    spi_cs_gyro: Option<Output<'static>>,
) -> Result<Sensor, SensorError> {
    match imu_kind {
        ImuKind::Ism330dlc => ism330dlc::init(i2c_bus)
            .await
            .map(Sensor::Ism)
            .map_err(SensorError::Ism),
        ImuKind::Bmi088 => {
            let cs_gyro = spi_cs_gyro.ok_or(SensorError::MissingGyroCs)?;
            bmi088::init(spi_bus, spi_cs_accel, cs_gyro)
                .await
                .map(Sensor::Bmi)
                .map_err(SensorError::Bmi)
        }
        ImuKind::Icm45686 => icm45686::init(spi_bus, spi_cs_accel)
            .await
            .map(Sensor::Icm45686)
            .map_err(SensorError::Icm45686),
    }
}

pub async fn read_ready(sensor: &mut Sensor) -> Result<Option<SensorReading>, SensorError> {
    match sensor {
        Sensor::Ism(sensor) => ism330dlc::read_ready(sensor).await.map_err(SensorError::Ism),
        Sensor::Bmi(sensor) => bmi088::read_ready(sensor).await.map_err(SensorError::Bmi),
        Sensor::Icm45686(sensor) => icm45686::read_ready(sensor)
            .await
            .map_err(SensorError::Icm45686),
    }
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Ism(error) => ism330dlc::log_error(error),
        SensorError::Bmi(error) => bmi088::log_error(error),
        SensorError::Icm45686(error) => icm45686::log_error(error),
        SensorError::MissingGyroCs => {
            error!("BMI088 requires a dedicated gyro CS pin");
        }
    }
}
