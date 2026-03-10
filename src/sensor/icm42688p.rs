use defmt::{error, warn};
use embassy_time::Timer;
use esp_hal::gpio::Output;

use crate::{
    board::{SharedI2c, SharedSpi},
    config,
    domain::sensor_reading::SensorReading,
};

use super::imu::{RegisterDevice, SpiBusConfig};

const REG_DEVICE_CONFIG: u8 = 0x11;
const REG_TEMP_DATA1: u8 = 0x1d;
const REG_ACCEL_DATA_X1: u8 = 0x1f;
const REG_GYRO_DATA_X1: u8 = 0x25;
const REG_INT_STATUS: u8 = 0x2d;
const REG_PWR_MGMT0: u8 = 0x4e;
const REG_GYRO_CONFIG0: u8 = 0x4f;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_WHO_AM_I: u8 = 0x75;

const DEVICE_SOFT_RESET: u8 = 0x01;
const WHO_AM_I_EXPECTED: u8 = 0x47;
const PWR_MGMT0_ACCEL_GYRO_LOW_NOISE: u8 = 0x0f;
const GYRO_CONFIG0_2000DPS_12_5HZ: u8 = 0x0b;
const ACCEL_CONFIG0_2G_12_5HZ: u8 = 0x6b;
const INT_STATUS_DATA_RDY: u8 = 0x08;
const I2C_ADDRESS_PRIMARY: u8 = 0x68;

const ACCEL_MG_PER_LSB_2G: f32 = 2000.0 / 32768.0;
const GYRO_MDPS_PER_LSB_2000DPS: f32 = 2_000_000.0 / 32768.0;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidDeviceId(u8),
    DataNotReady,
}

pub struct Icm42688p {
    device: RegisterDevice,
}

pub async fn init_spi(
    spi: &'static SharedSpi,
    cs: Output<'static>,
) -> Result<Icm42688p, SensorError> {
    init_with_device(RegisterDevice::new_spi(spi, cs, SpiBusConfig::standard())).await
}

pub async fn init_i2c(i2c: &'static SharedI2c) -> Result<Icm42688p, SensorError> {
    init_with_device(RegisterDevice::new_i2c(i2c, I2C_ADDRESS_PRIMARY)).await
}

async fn init_with_device(device: RegisterDevice) -> Result<Icm42688p, SensorError> {
    let mut sensor = Icm42688p { device };

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    sensor
        .device
        .write_register(REG_DEVICE_CONFIG, DEVICE_SOFT_RESET)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(1).await;

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
        .write_register(REG_GYRO_CONFIG0, GYRO_CONFIG0_2000DPS_12_5HZ)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .device
        .write_register(REG_ACCEL_CONFIG0, ACCEL_CONFIG0_2G_12_5HZ)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .device
        .write_register(REG_PWR_MGMT0, PWR_MGMT0_ACCEL_GYRO_LOW_NOISE)
        .await
        .map_err(|_| SensorError::Bus)?;

    Timer::after_millis(1).await;

    Ok(sensor)
}

pub async fn read_ready(
    sensor: &mut Icm42688p,
    timestamp_us: u64,
) -> Result<SensorReading, SensorError> {
    let status = sensor
        .device
        .read_register(REG_INT_STATUS)
        .await
        .map_err(|_| SensorError::Bus)?;
    if status & INT_STATUS_DATA_RDY == 0 {
        return Err(SensorError::DataNotReady);
    }

    let accel = sensor
        .device
        .read_xyz_be(REG_ACCEL_DATA_X1)
        .await
        .map_err(|_| SensorError::Bus)?;
    let gyro = sensor
        .device
        .read_xyz_be(REG_GYRO_DATA_X1)
        .await
        .map_err(|_| SensorError::Bus)?;
    let temp_raw = sensor
        .device
        .read_i16_be(REG_TEMP_DATA1)
        .await
        .map_err(|_| SensorError::Bus)? as f32;

    Ok(SensorReading {
        timestamp_us,
        acceleration_mg: accel.map(|value| value as f32 * ACCEL_MG_PER_LSB_2G),
        angular_rate_mdps: gyro.map(|value| value as f32 * GYRO_MDPS_PER_LSB_2000DPS),
        temperature_c: (temp_raw / 132.48) + 25.0,
    })
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Bus => error!("ICM-42688-P bus error"),
        SensorError::InvalidDeviceId(id) => warn!("Invalid ICM-42688-P ID {}", id),
        SensorError::DataNotReady => warn!("ICM-42688-P data not ready"),
    }
}
