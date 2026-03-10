use defmt::warn;
use embassy_time::Timer;
use esp_hal::gpio::Output;

use crate::{board::SharedSpi, config, domain::sensor_reading::SensorReading};

use super::imu::{RegisterDevice, log_bus_error};

const REG_WHO_AM_I: u8 = 0x0f;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL2_G: u8 = 0x11;
const REG_CTRL3_C: u8 = 0x12;
const REG_STATUS_REG: u8 = 0x1e;
const REG_OUT_TEMP_L: u8 = 0x20;
const REG_OUTX_L_G: u8 = 0x22;
const REG_OUTX_L_XL: u8 = 0x28;

const WHO_AM_I_EXPECTED: u8 = 0x6b;
const CTRL1_XL_12_5HZ_2G: u8 = 0x10;
// const CTRL1_XL_12_5HZ_16G: u8 = 0x20;
const CTRL2_G_12_5HZ_2000DPS: u8 = 0x1c;
const CTRL3_C_IF_INC_BDU: u8 = 0x44;
const CTRL3_C_SW_RESET: u8 = 0x01;

const STATUS_XLDA: u8 = 0x01;
const STATUS_GDA: u8 = 0x02;
const STATUS_TDA: u8 = 0x04;
const RESET_POLL_LIMIT: usize = 100;

const ACCEL_MG_PER_LSB_2G: f32 = 0.061;
// const ACCEL_MG_PER_LSB_4G: f32 = 0.122;
// const ACCEL_MG_PER_LSB_8G: f32 = 0.244;
// const ACCEL_MG_PER_LSB_16G: f32 = 0.488;
// const GYRO_MDPS_PER_LSB_125DPS: f32 = 4.375;
// const GYRO_MDPS_PER_LSB_250DPS: f32 = 8.75;
// const GYRO_MDPS_PER_LSB_500DPS: f32 = 17.5;
// const GYRO_MDPS_PER_LSB_1000DPS: f32 = 35.0;
const GYRO_MDPS_PER_LSB_2000DPS: f32 = 70.0;

pub struct Ism330dlc {
    device: RegisterDevice,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidDeviceId(u8),
}

pub async fn init(spi: &'static SharedSpi, cs: Output<'static>) -> Result<Ism330dlc, SensorError> {
    let mut sensor = Ism330dlc {
        device: RegisterDevice::new(spi, cs),
    };

    let device_id = sensor
        .device
        .read_register(REG_WHO_AM_I)
        .await
        .map_err(|_| SensorError::Bus)?;
    if device_id != WHO_AM_I_EXPECTED {
        return Err(SensorError::InvalidDeviceId(device_id));
    }

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    sensor
        .device
        .write_register(REG_CTRL3_C, CTRL3_C_SW_RESET)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(10).await;

    for _ in 0..RESET_POLL_LIMIT {
        let ctrl3 = sensor
            .device
            .read_register(REG_CTRL3_C)
            .await
            .map_err(|_| SensorError::Bus)?;
        if ctrl3 & CTRL3_C_SW_RESET == 0 {
            break;
        }
        Timer::after_millis(1).await;
    }

    let ctrl3 = sensor
        .device
        .read_register(REG_CTRL3_C)
        .await
        .map_err(|_| SensorError::Bus)?;
    if ctrl3 & CTRL3_C_SW_RESET != 0 {
        return Err(SensorError::Bus);
    }

    sensor
        .device
        .write_register(REG_CTRL3_C, CTRL3_C_IF_INC_BDU)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .device
        .write_register(REG_CTRL1_XL, CTRL1_XL_12_5HZ_2G)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .device
        .write_register(REG_CTRL2_G, CTRL2_G_12_5HZ_2000DPS)
        .await
        .map_err(|_| SensorError::Bus)?;

    Ok(sensor)
}

pub async fn read_ready(
    sensor: &mut Ism330dlc,
    timestamp_us: u64,
) -> Result<SensorReading, SensorError> {
    let status = sensor
        .device
        .read_register(REG_STATUS_REG)
        .await
        .map_err(|_| SensorError::Bus)?;

    let mut accel: [i16; 3] = [0; 3];
    if status & STATUS_XLDA != 0 {
        accel = sensor
            .device
            .read_xyz(REG_OUTX_L_XL)
            .await
            .map_err(|_| SensorError::Bus)?;
    }

    let mut gyro: [i16; 3] = [0; 3];
    if status & STATUS_GDA != 0 {
        gyro = sensor
            .device
            .read_xyz(REG_OUTX_L_G)
            .await
            .map_err(|_| SensorError::Bus)?;
    }

    let mut temp_raw: i16 = 0;
    if status & STATUS_TDA != 0 {
        temp_raw = sensor
            .device
            .read_i16(REG_OUT_TEMP_L)
            .await
            .map_err(|_| SensorError::Bus)?;
    }

    Ok(SensorReading {
        timestamp_us,
        acceleration_mg: accel.map(|v| v as f32 * ACCEL_MG_PER_LSB_2G),
        angular_rate_mdps: gyro.map(|v| v as f32 * GYRO_MDPS_PER_LSB_2000DPS),
        temperature_c: 25.0 + temp_raw as f32 / 256.0,
    })
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Bus => log_bus_error("ISM330DLC SPI"),
        SensorError::InvalidDeviceId(id) => warn!("Invalid ISM330DLC ID {}", id),
    }
}
