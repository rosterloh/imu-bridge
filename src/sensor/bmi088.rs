use defmt::{error, warn};
use embassy_time::Timer;
use esp_hal::gpio::Output;

use crate::{board::SharedSpi, domain::sensor_reading::SensorReading};

use super::imu::{RegisterDevice, SpiBusConfig};

const ACC_CHIP_ID: u8 = 0x1e;
const GYRO_CHIP_ID: u8 = 0x0f;

const ACC_REG_CHIP_ID: u8 = 0x00;
const ACC_REG_ACCEL_X_LSB: u8 = 0x12;
const ACC_REG_TEMP_MSB: u8 = 0x22;
const ACC_REG_TEMP_LSB: u8 = 0x23;
const ACC_REG_ACC_RANGE: u8 = 0x41;
const ACC_REG_PWR_CONF: u8 = 0x7c;
const ACC_REG_PWR_CTRL: u8 = 0x7d;
const ACC_REG_SOFT_RESET: u8 = 0x7e;

const GYRO_REG_CHIP_ID: u8 = 0x00;
const GYRO_REG_X_LSB: u8 = 0x02;
const GYRO_REG_RANGE: u8 = 0x0f;
const GYRO_REG_BANDWIDTH: u8 = 0x10;
const GYRO_REG_POWER_MODE: u8 = 0x11;
const GYRO_REG_SOFT_RESET: u8 = 0x14;

const SOFT_RESET_CMD: u8 = 0xb6;
const ACC_PWR_CONF_ACTIVE: u8 = 0x00;
const ACC_PWR_CTRL_ON: u8 = 0x04;
const GYRO_RANGE_2000_DPS: u8 = 0x00;
const GYRO_BANDWIDTH_2000HZ: u8 = 0x07;
const GYRO_POWER_NORMAL: u8 = 0x00;

const GYRO_MDPS_PER_LSB_2000DPS: f32 = 2_000_000.0 / 32768.0;

const ACCEL_SPI_CONFIG: SpiBusConfig = SpiBusConfig {
    read_mask: 0x80,
    write_mask: 0x7f,
    read_dummy_bytes: 1,
};

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidAccelDeviceId(u8),
    InvalidGyroDeviceId(u8),
}

pub struct Bmi088 {
    accel: RegisterDevice,
    gyro: RegisterDevice,
    accel_mg_per_lsb: f32,
}

pub async fn init_spi(
    spi: &'static SharedSpi,
    cs_accel: Output<'static>,
    cs_gyro: Output<'static>,
) -> Result<Bmi088, SensorError> {
    let mut sensor = Bmi088 {
        accel: RegisterDevice::new_spi(spi, cs_accel, ACCEL_SPI_CONFIG),
        gyro: RegisterDevice::new_spi(spi, cs_gyro, SpiBusConfig::standard()),
        accel_mg_per_lsb: 3000.0 / 32768.0,
    };

    let accel_id = sensor
        .accel
        .read_register(ACC_REG_CHIP_ID)
        .await
        .map_err(|_| SensorError::Bus)?;
    if accel_id != ACC_CHIP_ID {
        return Err(SensorError::InvalidAccelDeviceId(accel_id));
    }

    let gyro_id = sensor
        .gyro
        .read_register(GYRO_REG_CHIP_ID)
        .await
        .map_err(|_| SensorError::Bus)?;
    if gyro_id != GYRO_CHIP_ID {
        return Err(SensorError::InvalidGyroDeviceId(gyro_id));
    }

    sensor
        .accel
        .write_register(ACC_REG_SOFT_RESET, SOFT_RESET_CMD)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(10).await;

    sensor
        .accel
        .write_register(ACC_REG_PWR_CONF, ACC_PWR_CONF_ACTIVE)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(1).await;
    sensor
        .accel
        .write_register(ACC_REG_PWR_CTRL, ACC_PWR_CTRL_ON)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(50).await;

    sensor
        .gyro
        .write_register(GYRO_REG_SOFT_RESET, SOFT_RESET_CMD)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(100).await;
    sensor
        .gyro
        .write_register(GYRO_REG_RANGE, GYRO_RANGE_2000_DPS)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .gyro
        .write_register(GYRO_REG_BANDWIDTH, GYRO_BANDWIDTH_2000HZ)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .gyro
        .write_register(GYRO_REG_POWER_MODE, GYRO_POWER_NORMAL)
        .await
        .map_err(|_| SensorError::Bus)?;

    let accel_range = sensor
        .accel
        .read_register(ACC_REG_ACC_RANGE)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor.accel_mg_per_lsb = accel_range_mg_per_lsb(accel_range);

    Ok(sensor)
}

pub async fn read_ready(
    sensor: &mut Bmi088,
    timestamp_us: u64,
) -> Result<SensorReading, SensorError> {
    let accel = sensor
        .accel
        .read_xyz_le(ACC_REG_ACCEL_X_LSB)
        .await
        .map_err(|_| SensorError::Bus)?;
    let gyro = sensor
        .gyro
        .read_xyz_le(GYRO_REG_X_LSB)
        .await
        .map_err(|_| SensorError::Bus)?;
    let msb = sensor
        .accel
        .read_register(ACC_REG_TEMP_MSB)
        .await
        .map_err(|_| SensorError::Bus)?;
    let lsb = sensor
        .accel
        .read_register(ACC_REG_TEMP_LSB)
        .await
        .map_err(|_| SensorError::Bus)?;
    let temp_raw = (msb as i16) * 8 + (lsb as i16) / 32;

    Ok(SensorReading {
        timestamp_us,
        acceleration_mg: accel.map(|value| value as f32 * sensor.accel_mg_per_lsb),
        angular_rate_mdps: gyro.map(|value| value as f32 * GYRO_MDPS_PER_LSB_2000DPS),
        temperature_c: (temp_raw as f32) * 0.125 + 23.0,
    })
}

fn accel_range_mg_per_lsb(range_register: u8) -> f32 {
    let range_g = match range_register & 0x03 {
        0x00 => 3.0,
        0x01 => 6.0,
        0x02 => 12.0,
        _ => 24.0,
    };
    (range_g * 1000.0) / 32768.0
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Bus => error!("BMI088 SPI bus error"),
        SensorError::InvalidAccelDeviceId(id) => warn!("Invalid BMI088 accel ID {}", id),
        SensorError::InvalidGyroDeviceId(id) => warn!("Invalid BMI088 gyro ID {}", id),
    }
}
