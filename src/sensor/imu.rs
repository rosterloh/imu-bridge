use defmt::{error, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_time::{Delay, Timer};
use ism330dhcx_rs::asynchronous::{
    I2CAddress, ISM330DHCX_ID, Ism330dhcx, PROPERTY_ENABLE, from_fs2g_to_mg,
    from_fs2000dps_to_mdps, from_lsb_to_celsius, register::MainBank,
};
use st_mems_bus::asynchronous::i2c::I2cBus as StI2cBus;

use crate::{board::SharedI2c, config, domain::sensor_reading::SensorReading};

type SharedI2cDevice = I2cDevice<
    'static,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    esp_hal::i2c::master::I2c<'static, esp_hal::Async>,
>;

pub type Imu = Ism330dhcx<StI2cBus<SharedI2cDevice>, Delay, MainBank>;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidDeviceId(u8),
}

pub async fn init(i2c_bus: &'static SharedI2c) -> Result<Imu, SensorError> {
    let delay = Delay;
    let device = I2cDevice::new(i2c_bus);
    let mut sensor = Ism330dhcx::new_i2c(device, I2CAddress::I2cAddL, delay);

    let device_id = sensor.device_id_get().await.map_err(|_| SensorError::Bus)?;
    if device_id != ISM330DHCX_ID {
        return Err(SensorError::InvalidDeviceId(device_id));
    }

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    sensor
        .reset_set(PROPERTY_ENABLE)
        .await
        .map_err(|_| SensorError::Bus)?;

    loop {
        let reset_done = sensor.reset_get().await.map_err(|_| SensorError::Bus)?;
        if reset_done == 0 {
            break;
        }
    }

    sensor
        .device_conf_set(PROPERTY_ENABLE)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .block_data_update_set(PROPERTY_ENABLE)
        .await
        .map_err(|_| SensorError::Bus)?;

    sensor
        .xl_data_rate_set(config::ACCEL_ODR)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .gy_data_rate_set(config::GYRO_ODR)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .xl_full_scale_set(config::ACCEL_FULL_SCALE)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .gy_full_scale_set(config::GYRO_FULL_SCALE)
        .await
        .map_err(|_| SensorError::Bus)?;

    sensor
        .xl_hp_path_on_out_set(config::ACCEL_FILTER_PATH)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .xl_filter_lp2_set(PROPERTY_ENABLE)
        .await
        .map_err(|_| SensorError::Bus)?;

    Ok(sensor)
}

pub async fn read_ready(sensor: &mut Imu) -> Result<Option<SensorReading>, SensorError> {
    if sensor
        .xl_flag_data_ready_get()
        .await
        .map_err(|_| SensorError::Bus)?
        == 1
    {
        let raw = sensor
            .acceleration_raw_get()
            .await
            .map_err(|_| SensorError::Bus)?;
        return Ok(Some(SensorReading::Acceleration(raw.map(from_fs2g_to_mg))));
    }

    if sensor
        .gy_flag_data_ready_get()
        .await
        .map_err(|_| SensorError::Bus)?
        == 1
    {
        let raw = sensor
            .angular_rate_raw_get()
            .await
            .map_err(|_| SensorError::Bus)?;
        return Ok(Some(SensorReading::AngularRate(
            raw.map(from_fs2000dps_to_mdps),
        )));
    }

    if sensor
        .temp_flag_data_ready_get()
        .await
        .map_err(|_| SensorError::Bus)?
        == 1
    {
        let raw = sensor
            .temperature_raw_get()
            .await
            .map_err(|_| SensorError::Bus)?;
        return Ok(Some(SensorReading::Temperature(from_lsb_to_celsius(raw))));
    }

    Ok(None)
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Bus => {
            error!("Sensor bus error");
        }
        SensorError::InvalidDeviceId(id) => {
            warn!("Invalid sensor ID {}", id);
        }
    }
}
