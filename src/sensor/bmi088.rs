use bmi088::{Builder, Error as BmiError};
use defmt::error;
use embassy_time::Delay;
use embedded_hal::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use esp_hal::gpio::Output;
use esp_hal::spi::master::Spi;

use crate::{board::SharedSpi, domain::sensor_reading::SensorReading};

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    Unresponsive,
}

#[derive(Debug)]
pub struct SpiWrapError;

impl embedded_hal::spi::Error for SpiWrapError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

struct BorrowedSpiDevice<'a> {
    spi: &'a mut Spi<'static, esp_hal::Blocking>,
    cs: &'a mut Output<'static>,
}

impl ErrorType for BorrowedSpiDevice<'_> {
    type Error = SpiWrapError;
}

impl SpiDevice<u8> for BorrowedSpiDevice<'_> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        self.cs.set_low();
        let result = (|| {
            for op in operations {
                match op {
                    Operation::Read(buf) => SpiBus::read(self.spi, buf).map_err(|_| SpiWrapError)?,
                    Operation::Write(buf) => {
                        SpiBus::write(self.spi, buf).map_err(|_| SpiWrapError)?
                    }
                    Operation::Transfer(read, write) => {
                        SpiBus::transfer(self.spi, read, write).map_err(|_| SpiWrapError)?
                    }
                    Operation::TransferInPlace(buf) => {
                        SpiBus::transfer_in_place(self.spi, buf).map_err(|_| SpiWrapError)?
                    }
                    Operation::DelayNs(_) => {}
                }
            }
            SpiBus::flush(self.spi).map_err(|_| SpiWrapError)
        })();
        self.cs.set_high();
        result
    }
}

pub struct Bmi088 {
    spi: &'static SharedSpi,
    cs_accel: Output<'static>,
    cs_gyro: Output<'static>,
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
        cycle_index: 0,
    };

    let mut delay = Delay;

    {
        let mut spi = sensor.spi.lock().await;
        let accel_spi = BorrowedSpiDevice {
            spi: &mut spi,
            cs: &mut sensor.cs_accel,
        };
        let mut accel = Builder::new_accel_spi(accel_spi);
        accel.setup(&mut delay).map_err(map_bmi_error)?;
    }

    {
        let mut spi = sensor.spi.lock().await;
        let gyro_spi = BorrowedSpiDevice {
            spi: &mut spi,
            cs: &mut sensor.cs_gyro,
        };
        let mut gyro = Builder::new_gyro_spi(gyro_spi);
        gyro.setup(&mut delay).map_err(map_bmi_error)?;
    }

    Ok(sensor)
}

pub async fn read_ready(sensor: &mut Bmi088) -> Result<Option<SensorReading>, SensorError> {
    let reading = match sensor.cycle_index {
        0 => {
            let mut spi = sensor.spi.lock().await;
            let accel_spi = BorrowedSpiDevice {
                spi: &mut spi,
                cs: &mut sensor.cs_accel,
            };
            let mut accel = Builder::new_accel_spi(accel_spi);
            let raw = accel.get_accel().map_err(map_bmi_error)?;
            Some(SensorReading::Acceleration([
                raw[0] as f32 * 1000.0 / 10920.0,
                raw[1] as f32 * 1000.0 / 10920.0,
                raw[2] as f32 * 1000.0 / 10920.0,
            ]))
        }
        1 => {
            let mut spi = sensor.spi.lock().await;
            let gyro_spi = BorrowedSpiDevice {
                spi: &mut spi,
                cs: &mut sensor.cs_gyro,
            };
            let mut gyro = Builder::new_gyro_spi(gyro_spi);
            let raw = gyro.get_gyro().map_err(map_bmi_error)?;
            Some(SensorReading::AngularRate([
                raw[0] as f32 * 1000.0 / 16.4,
                raw[1] as f32 * 1000.0 / 16.4,
                raw[2] as f32 * 1000.0 / 16.4,
            ]))
        }
        _ => None,
    };

    sensor.cycle_index = (sensor.cycle_index + 1) % 3;
    Ok(reading)
}

fn map_bmi_error(error: BmiError<SpiWrapError, SpiWrapError>) -> SensorError {
    match error {
        BmiError::Unresponsive => SensorError::Unresponsive,
        BmiError::Comm(_) | BmiError::Pin(_) => SensorError::Bus,
    }
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Bus => error!("BMI088 SPI bus error"),
        SensorError::Unresponsive => error!("BMI088 not responding"),
    }
}
