use defmt::error;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embedded_hal_async::i2c::I2c;

use crate::board::SharedI2c;

pub type SharedI2cDevice = I2cDevice<
    'static,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    esp_hal::i2c::master::I2c<'static, esp_hal::Async>,
>;

pub struct RegisterDevice {
    device: SharedI2cDevice,
    address: u8,
}

impl RegisterDevice {
    pub fn new(i2c_bus: &'static SharedI2c, address: u8) -> Self {
        Self {
            device: I2cDevice::new(i2c_bus),
            address,
        }
    }

    pub async fn read_register(&mut self, register: u8) -> Result<u8, ()> {
        let mut buf = [0u8; 1];
        self.device
            .write_read(self.address, &[register], &mut buf)
            .await
            .map_err(|_| ())?;
        Ok(buf[0])
    }

    pub async fn write_register(&mut self, register: u8, value: u8) -> Result<(), ()> {
        self.device
            .write(self.address, &[register, value])
            .await
            .map_err(|_| ())
    }

    pub async fn read_i16(&mut self, start_register: u8) -> Result<i16, ()> {
        let mut buf = [0u8; 2];
        self.device
            .write_read(self.address, &[start_register], &mut buf)
            .await
            .map_err(|_| ())?;
        Ok(i16::from_le_bytes(buf))
    }

    pub async fn read_xyz(&mut self, start_register: u8) -> Result<[i16; 3], ()> {
        let mut buf = [0u8; 6];
        self.device
            .write_read(self.address, &[start_register], &mut buf)
            .await
            .map_err(|_| ())?;

        Ok([
            i16::from_le_bytes([buf[0], buf[1]]),
            i16::from_le_bytes([buf[2], buf[3]]),
            i16::from_le_bytes([buf[4], buf[5]]),
        ])
    }
}

pub fn log_bus_error(label: &str) {
    error!("{} bus error", label);
}
