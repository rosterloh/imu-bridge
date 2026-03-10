use defmt::{error, warn};
use embassy_time::Timer;
use esp_hal::gpio::Output;

use crate::{
    board::{SharedI2c, SharedSpi},
    config,
    domain::sensor_reading::SensorReading,
};

use super::imu::{RegisterDevice, SpiBusConfig};

const REG_ACCEL_DATA_X1_UI: u8 = 0x00;
const REG_GYRO_DATA_X1_UI: u8 = 0x06;
const REG_TEMP_DATA1_UI: u8 = 0x0c;
const REG_PWR_MGMT0: u8 = 0x10;
const REG_ACCEL_CONFIG0: u8 = 0x1b;
const REG_GYRO_CONFIG0: u8 = 0x1c;
const REG_WHO_AM_I: u8 = 0x72;

const WHO_AM_I_EXPECTED: u8 = 0xe9;
const POWER_MODE_LOW_NOISE: u8 = 0x0f;
const ACCEL_CONFIG_2G_12_5HZ: u8 = 0x4c;
const GYRO_CONFIG_2000DPS_12_5HZ: u8 = 0x1c;
const I2C_ADDRESS_PRIMARY: u8 = 0x68;

const ACCEL_MG_PER_LSB_2G: f32 = 2000.0 / 32768.0;
const GYRO_MDPS_PER_LSB_2000DPS: f32 = 2_000_000.0 / 32768.0;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidDeviceId(u8),
}

pub struct Icm45686 {
    device: RegisterDevice,
}

pub async fn init_spi(
    spi: &'static SharedSpi,
    cs: Output<'static>,
) -> Result<Icm45686, SensorError> {
    init_with_device(RegisterDevice::new_spi(spi, cs, SpiBusConfig::standard())).await
}

pub async fn init_i2c(i2c: &'static SharedI2c) -> Result<Icm45686, SensorError> {
    init_with_device(RegisterDevice::new_i2c(i2c, I2C_ADDRESS_PRIMARY)).await
}

async fn init_with_device(device: RegisterDevice) -> Result<Icm45686, SensorError> {
    let mut sensor = Icm45686 { device };

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    let device_id = sensor
        .device
        .read_register(REG_WHO_AM_I)
        .await
        .map_err(|_| SensorError::Bus)?;
    if device_id != WHO_AM_I_EXPECTED {
        return Err(SensorError::InvalidDeviceId(device_id));
    }

    sensor
        .device
        .write_register(REG_PWR_MGMT0, POWER_MODE_LOW_NOISE)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .device
        .write_register(REG_ACCEL_CONFIG0, ACCEL_CONFIG_2G_12_5HZ)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .device
        .write_register(REG_GYRO_CONFIG0, GYRO_CONFIG_2000DPS_12_5HZ)
        .await
        .map_err(|_| SensorError::Bus)?;

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    Ok(sensor)
}

pub async fn read_ready(
    sensor: &mut Icm45686,
    timestamp_us: u64,
) -> Result<SensorReading, SensorError> {
    let accel = sensor
        .device
        .read_xyz_be(REG_ACCEL_DATA_X1_UI)
        .await
        .map_err(|_| SensorError::Bus)?;
    let gyro = sensor
        .device
        .read_xyz_be(REG_GYRO_DATA_X1_UI)
        .await
        .map_err(|_| SensorError::Bus)?;
    let temp_raw = sensor
        .device
        .read_i16_be(REG_TEMP_DATA1_UI)
        .await
        .map_err(|_| SensorError::Bus)? as f32;

    Ok(SensorReading {
        timestamp_us,
        acceleration_mg: accel.map(|value| value as f32 * ACCEL_MG_PER_LSB_2G),
        angular_rate_mdps: gyro.map(|value| value as f32 * GYRO_MDPS_PER_LSB_2000DPS),
        temperature_c: (temp_raw / 128.0) + 25.0,
    })
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Bus => error!("ICM-45686 bus error"),
        SensorError::InvalidDeviceId(id) => warn!("Invalid ICM-45686 ID {}", id),
    }
}
