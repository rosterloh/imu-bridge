use defmt::warn;

use crate::{board::SharedI2c, config, domain::sensor_reading::SensorReading};

use super::imu::{RegisterDevice, log_bus_error};

const DEVICE_ADDRESS: u8 = 0x6a;

const REG_WHO_AM_I: u8 = 0x0f;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL2_G: u8 = 0x11;
const REG_CTRL3_C: u8 = 0x12;
const REG_STATUS_REG: u8 = 0x1e;
const REG_OUT_TEMP_L: u8 = 0x20;
const REG_OUTX_L_G: u8 = 0x22;
const REG_OUTX_L_XL: u8 = 0x28;

const WHO_AM_I_EXPECTED: u8 = 0x6a;
const CTRL1_XL_12_5HZ_2G: u8 = 0x10;
const CTRL2_G_12_5HZ_2000DPS: u8 = 0x1c;
const CTRL3_C_IF_INC_BDU: u8 = 0x44;
const CTRL3_C_SW_RESET: u8 = 0x01;

const STATUS_XLDA: u8 = 0x01;
const STATUS_GDA: u8 = 0x02;
const STATUS_TDA: u8 = 0x04;

const ACCEL_MG_PER_LSB_2G: f32 = 0.061;
const GYRO_MDPS_PER_LSB_2000DPS: f32 = 70.0;

pub struct Ism330dlc {
    device: RegisterDevice,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidDeviceId(u8),
}

pub async fn init(i2c_bus: &'static SharedI2c) -> Result<Ism330dlc, SensorError> {
    let mut sensor = Ism330dlc {
        device: RegisterDevice::new(i2c_bus, DEVICE_ADDRESS),
    };

    let device_id = sensor
        .device
        .read_register(REG_WHO_AM_I)
        .await
        .map_err(|_| SensorError::Bus)?;
    if device_id != WHO_AM_I_EXPECTED {
        return Err(SensorError::InvalidDeviceId(device_id));
    }

    embassy_time::Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    sensor
        .device
        .write_register(REG_CTRL3_C, CTRL3_C_SW_RESET)
        .await
        .map_err(|_| SensorError::Bus)?;

    loop {
        let ctrl3 = sensor
            .device
            .read_register(REG_CTRL3_C)
            .await
            .map_err(|_| SensorError::Bus)?;
        if ctrl3 & CTRL3_C_SW_RESET == 0 {
            break;
        }
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

pub async fn read_ready(sensor: &mut Ism330dlc) -> Result<Option<SensorReading>, SensorError> {
    let status = sensor
        .device
        .read_register(REG_STATUS_REG)
        .await
        .map_err(|_| SensorError::Bus)?;

    if status & STATUS_XLDA != 0 {
        let raw = sensor
            .device
            .read_xyz(REG_OUTX_L_XL)
            .await
            .map_err(|_| SensorError::Bus)?;
        return Ok(Some(SensorReading::Acceleration(raw.map(|v| {
            v as f32 * ACCEL_MG_PER_LSB_2G
        }))));
    }

    if status & STATUS_GDA != 0 {
        let raw = sensor
            .device
            .read_xyz(REG_OUTX_L_G)
            .await
            .map_err(|_| SensorError::Bus)?;
        return Ok(Some(SensorReading::AngularRate(raw.map(|v| {
            v as f32 * GYRO_MDPS_PER_LSB_2000DPS
        }))));
    }

    if status & STATUS_TDA != 0 {
        let raw = sensor
            .device
            .read_i16(REG_OUT_TEMP_L)
            .await
            .map_err(|_| SensorError::Bus)?;
        return Ok(Some(SensorReading::Temperature(25.0 + raw as f32 / 256.0)));
    }

    Ok(None)
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Bus => log_bus_error("ISM330DLC I2C"),
        SensorError::InvalidDeviceId(id) => warn!("Invalid ISM330DLC ID {}", id),
    }
}
