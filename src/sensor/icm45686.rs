use defmt::{error, warn};
use embassy_time::Timer;
use embedded_hal_async::spi::SpiBus;
use esp_hal::gpio::Output;

use crate::sensor::SensorChannel;
use crate::{board::SharedSpi, config, domain::sensor_reading::SensorReading};

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

const ACCEL_MG_PER_LSB_2G: f32 = 2000.0 / 32768.0;
const GYRO_MDPS_PER_LSB_2000DPS: f32 = 2_000_000.0 / 32768.0;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidDeviceId(u8),
}

pub struct Icm45686 {
    spi: &'static SharedSpi,
    cs: Output<'static>,
    cycle_index: u8,
}

pub async fn init(
    spi: &'static SharedSpi,
    mut cs: Output<'static>,
) -> Result<Icm45686, SensorError> {
    cs.set_high();

    let mut sensor = Icm45686 {
        spi,
        cs,
        cycle_index: 0,
    };

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    let device_id = sensor.read_register(REG_WHO_AM_I).await?;
    if device_id != WHO_AM_I_EXPECTED {
        return Err(SensorError::InvalidDeviceId(device_id));
    }

    sensor
        .write_register(REG_PWR_MGMT0, POWER_MODE_LOW_NOISE)
        .await?;
    sensor
        .write_register(REG_ACCEL_CONFIG0, ACCEL_CONFIG_2G_12_5HZ)
        .await?;
    sensor
        .write_register(REG_GYRO_CONFIG0, GYRO_CONFIG_2000DPS_12_5HZ)
        .await?;

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    Ok(sensor)
}

pub async fn read_ready(
    sensor: &mut Icm45686,
    sensor_channel: &'static SensorChannel,
) -> Result<(), SensorError> {
    match sensor.cycle_index {
        0 => {
            let mut buf = [0u8; 6];
            sensor
                .read_registers(REG_ACCEL_DATA_X1_UI, &mut buf)
                .await?;
            sensor_channel
                .send(SensorReading::Acceleration([
                    read_be_i16(&buf[0..2]) as f32 * ACCEL_MG_PER_LSB_2G,
                    read_be_i16(&buf[2..4]) as f32 * ACCEL_MG_PER_LSB_2G,
                    read_be_i16(&buf[4..6]) as f32 * ACCEL_MG_PER_LSB_2G,
                ]))
                .await;
        }
        1 => {
            let mut buf = [0u8; 6];
            sensor.read_registers(REG_GYRO_DATA_X1_UI, &mut buf).await?;
            sensor_channel
                .send(SensorReading::AngularRate([
                    read_be_i16(&buf[0..2]) as f32 * GYRO_MDPS_PER_LSB_2000DPS,
                    read_be_i16(&buf[2..4]) as f32 * GYRO_MDPS_PER_LSB_2000DPS,
                    read_be_i16(&buf[4..6]) as f32 * GYRO_MDPS_PER_LSB_2000DPS,
                ]))
                .await;
        }
        _ => {
            let mut buf = [0u8; 2];
            sensor.read_registers(REG_TEMP_DATA1_UI, &mut buf).await?;
            let temp_raw = read_be_i16(&buf) as f32;
            sensor_channel
                .send(SensorReading::Temperature((temp_raw / 128.0) + 25.0))
                .await;
        }
    };

    sensor.cycle_index = (sensor.cycle_index + 1) % 3;
    Ok(())
}

impl Icm45686 {
    async fn read_register(&mut self, address: u8) -> Result<u8, SensorError> {
        let mut buf = [address | 0x80, 0];
        self.transfer(&mut buf).await?;
        Ok(buf[1])
    }

    async fn read_registers(
        &mut self,
        start_address: u8,
        data: &mut [u8],
    ) -> Result<(), SensorError> {
        let mut buf = [0u8; 7];
        let frame = &mut buf[..data.len() + 1];
        frame[0] = start_address | 0x80;
        self.transfer(frame).await?;
        data.copy_from_slice(&frame[1..]);
        Ok(())
    }

    async fn write_register(&mut self, address: u8, value: u8) -> Result<(), SensorError> {
        let buf = [address & 0x7f, value];
        let mut spi = self.spi.lock().await;
        self.cs.set_low();
        let result = SpiBus::write(&mut *spi, &buf).await;
        let flush_result = SpiBus::flush(&mut *spi).await;
        self.cs.set_high();
        result.map_err(|_| SensorError::Bus)?;
        flush_result.map_err(|_| SensorError::Bus)?;
        Ok(())
    }

    async fn transfer(&mut self, buffer: &mut [u8]) -> Result<(), SensorError> {
        let mut spi = self.spi.lock().await;
        self.cs.set_low();
        let result = SpiBus::transfer_in_place(&mut *spi, buffer).await;
        let flush_result = SpiBus::flush(&mut *spi).await;
        self.cs.set_high();
        result.map_err(|_| SensorError::Bus)?;
        flush_result.map_err(|_| SensorError::Bus)?;
        Ok(())
    }
}

fn read_be_i16(bytes: &[u8]) -> i16 {
    i16::from_be_bytes([bytes[0], bytes[1]])
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Bus => error!("ICM-45686 SPI bus error"),
        SensorError::InvalidDeviceId(id) => warn!("Invalid ICM-45686 ID {}", id),
    }
}
