use defmt::{error, warn};
use embassy_time::Timer;
use embedded_hal_async::spi::SpiBus;
use esp_hal::gpio::Output;

use crate::{board::SharedSpi, domain::sensor_reading::SensorReading};

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

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidAccelDeviceId(u8),
    InvalidGyroDeviceId(u8),
}

pub struct Bmi088 {
    spi: &'static SharedSpi,
    cs_accel: Output<'static>,
    cs_gyro: Output<'static>,
    accel_mg_per_lsb: f32,
    cycle_index: u8,
}

pub async fn init(
    spi: &'static SharedSpi,
    mut cs_accel: Output<'static>,
    mut cs_gyro: Output<'static>,
) -> Result<Bmi088, SensorError> {
    cs_accel.set_high();
    cs_gyro.set_high();

    let mut sensor = Bmi088 {
        spi,
        cs_accel,
        cs_gyro,
        accel_mg_per_lsb: 3000.0 / 32768.0,
        cycle_index: 0,
    };

    let accel_id = sensor.read_accel_register(ACC_REG_CHIP_ID).await?;
    if accel_id != ACC_CHIP_ID {
        return Err(SensorError::InvalidAccelDeviceId(accel_id));
    }

    let gyro_id = sensor.read_gyro_register(GYRO_REG_CHIP_ID).await?;
    if gyro_id != GYRO_CHIP_ID {
        return Err(SensorError::InvalidGyroDeviceId(gyro_id));
    }

    sensor
        .write_accel_register(ACC_REG_SOFT_RESET, SOFT_RESET_CMD)
        .await?;
    Timer::after_millis(10).await;

    sensor
        .write_accel_register(ACC_REG_PWR_CONF, ACC_PWR_CONF_ACTIVE)
        .await?;
    Timer::after_millis(1).await;
    sensor
        .write_accel_register(ACC_REG_PWR_CTRL, ACC_PWR_CTRL_ON)
        .await?;
    Timer::after_millis(50).await;

    sensor
        .write_gyro_register(GYRO_REG_SOFT_RESET, SOFT_RESET_CMD)
        .await?;
    Timer::after_millis(100).await;
    sensor
        .write_gyro_register(GYRO_REG_RANGE, GYRO_RANGE_2000_DPS)
        .await?;
    sensor
        .write_gyro_register(GYRO_REG_BANDWIDTH, GYRO_BANDWIDTH_2000HZ)
        .await?;
    sensor
        .write_gyro_register(GYRO_REG_POWER_MODE, GYRO_POWER_NORMAL)
        .await?;

    let accel_range = sensor.read_accel_register(ACC_REG_ACC_RANGE).await?;
    sensor.accel_mg_per_lsb = accel_range_mg_per_lsb(accel_range);

    Ok(sensor)
}

pub async fn read_ready(sensor: &mut Bmi088) -> Result<Option<SensorReading>, SensorError> {
    let reading = match sensor.cycle_index {
        0 => {
            let mut buf = [0u8; 6];
            sensor
                .read_accel_burst(ACC_REG_ACCEL_X_LSB, &mut buf)
                .await?;
            Some(SensorReading::Acceleration([
                read_le_i16(&buf[0..2]) as f32 * sensor.accel_mg_per_lsb,
                read_le_i16(&buf[2..4]) as f32 * sensor.accel_mg_per_lsb,
                read_le_i16(&buf[4..6]) as f32 * sensor.accel_mg_per_lsb,
            ]))
        }
        1 => {
            let mut buf = [0u8; 6];
            sensor.read_gyro_burst(GYRO_REG_X_LSB, &mut buf).await?;
            Some(SensorReading::AngularRate([
                read_le_i16(&buf[0..2]) as f32 * GYRO_MDPS_PER_LSB_2000DPS,
                read_le_i16(&buf[2..4]) as f32 * GYRO_MDPS_PER_LSB_2000DPS,
                read_le_i16(&buf[4..6]) as f32 * GYRO_MDPS_PER_LSB_2000DPS,
            ]))
        }
        2 => {
            let msb = sensor.read_accel_register(ACC_REG_TEMP_MSB).await?;
            let lsb = sensor.read_accel_register(ACC_REG_TEMP_LSB).await?;
            let temp_raw = (msb as i16) * 8 + (lsb as i16) / 32;
            Some(SensorReading::Temperature((temp_raw as f32) * 0.125 + 23.0))
        }
        _ => None,
    };

    sensor.cycle_index = (sensor.cycle_index + 1) % 4;
    Ok(reading)
}

impl Bmi088 {
    async fn read_accel_register(&mut self, register: u8) -> Result<u8, SensorError> {
        let mut buf = [register | 0x80, 0x00, 0x00];
        transfer(self.spi, &mut self.cs_accel, &mut buf).await?;
        Ok(buf[2])
    }

    async fn read_gyro_register(&mut self, register: u8) -> Result<u8, SensorError> {
        let mut buf = [register | 0x80, 0x00];
        transfer(self.spi, &mut self.cs_gyro, &mut buf).await?;
        Ok(buf[1])
    }

    async fn read_accel_burst(
        &mut self,
        register: u8,
        out: &mut [u8; 6],
    ) -> Result<(), SensorError> {
        let mut buf = [0u8; 8];
        buf[0] = register | 0x80;
        transfer(self.spi, &mut self.cs_accel, &mut buf).await?;
        out.copy_from_slice(&buf[2..]);
        Ok(())
    }

    async fn read_gyro_burst(
        &mut self,
        register: u8,
        out: &mut [u8; 6],
    ) -> Result<(), SensorError> {
        let mut buf = [0u8; 7];
        buf[0] = register | 0x80;
        transfer(self.spi, &mut self.cs_gyro, &mut buf).await?;
        out.copy_from_slice(&buf[1..]);
        Ok(())
    }

    async fn write_accel_register(&mut self, register: u8, value: u8) -> Result<(), SensorError> {
        write(self.spi, &mut self.cs_accel, &[register & 0x7f, value]).await
    }

    async fn write_gyro_register(&mut self, register: u8, value: u8) -> Result<(), SensorError> {
        write(self.spi, &mut self.cs_gyro, &[register & 0x7f, value]).await
    }
}

async fn transfer(
    spi_bus: &'static SharedSpi,
    cs: &mut Output<'static>,
    buffer: &mut [u8],
) -> Result<(), SensorError> {
    let mut spi = spi_bus.lock().await;
    cs.set_low();
    let result = SpiBus::transfer_in_place(&mut *spi, buffer).await;
    let flush_result = SpiBus::flush(&mut *spi).await;
    cs.set_high();
    result.map_err(|_| SensorError::Bus)?;
    flush_result.map_err(|_| SensorError::Bus)?;
    Ok(())
}

async fn write(
    spi_bus: &'static SharedSpi,
    cs: &mut Output<'static>,
    buffer: &[u8],
) -> Result<(), SensorError> {
    let mut spi = spi_bus.lock().await;
    cs.set_low();
    let result = SpiBus::write(&mut *spi, buffer).await;
    let flush_result = SpiBus::flush(&mut *spi).await;
    cs.set_high();
    result.map_err(|_| SensorError::Bus)?;
    flush_result.map_err(|_| SensorError::Bus)?;
    Ok(())
}

fn read_le_i16(bytes: &[u8]) -> i16 {
    i16::from_le_bytes([bytes[0], bytes[1]])
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
