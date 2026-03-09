pub mod bmi088;
pub mod imu;

use defmt::error;
use esp_hal::gpio::Output;

use crate::{
    board::{SharedI2c, SharedSpi},
    config::ImuKind,
    domain::sensor_reading::SensorReading,
};

pub enum Sensor {
    Ism(imu::Imu),
    Bmi(bmi088::Bmi088),
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Ism(imu::SensorError),
    Bmi(bmi088::SensorError),
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
        ImuKind::Ism330dhcx => imu::init(i2c_bus)
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
    }
}

pub async fn read_ready(sensor: &mut Sensor) -> Result<Option<SensorReading>, SensorError> {
    match sensor {
        Sensor::Ism(sensor) => imu::read_ready(sensor).await.map_err(SensorError::Ism),
        Sensor::Bmi(sensor) => bmi088::read_ready(sensor).await.map_err(SensorError::Bmi),
    }
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Ism(error) => imu::log_error(error),
        SensorError::Bmi(error) => bmi088::log_error(error),
        SensorError::MissingGyroCs => {
            error!("BMI088 requires a dedicated gyro CS pin");
        }
    }
}
