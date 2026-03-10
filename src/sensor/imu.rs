use defmt::error;
use embedded_hal_async::{i2c::I2c, spi::SpiBus};
use esp_hal::gpio::Output;

use crate::board::{SharedI2c, SharedSpi};

#[derive(Clone, Copy)]
pub struct SpiBusConfig {
    pub read_mask: u8,
    pub write_mask: u8,
    pub read_dummy_bytes: usize,
}

impl SpiBusConfig {
    pub const fn standard() -> Self {
        Self {
            read_mask: 0x80,
            write_mask: 0x7f,
            read_dummy_bytes: 0,
        }
    }
}

pub enum RegisterDevice {
    Spi(SpiRegisterDevice),
    I2c(I2cRegisterDevice),
}

pub struct SpiRegisterDevice {
    spi: &'static SharedSpi,
    cs: Output<'static>,
    config: SpiBusConfig,
}

pub struct I2cRegisterDevice {
    i2c: &'static SharedI2c,
    address: u8,
}

impl RegisterDevice {
    pub fn new_spi(spi: &'static SharedSpi, mut cs: Output<'static>, config: SpiBusConfig) -> Self {
        cs.set_high();
        Self::Spi(SpiRegisterDevice { spi, cs, config })
    }

    pub fn new_i2c(i2c: &'static SharedI2c, address: u8) -> Self {
        Self::I2c(I2cRegisterDevice { i2c, address })
    }

    pub async fn read_register(&mut self, register: u8) -> Result<u8, ()> {
        let mut value = [0u8; 1];
        self.read_registers(register, &mut value).await?;
        Ok(value[0])
    }

    pub async fn write_register(&mut self, register: u8, value: u8) -> Result<(), ()> {
        self.write_registers(register, &[value]).await
    }

    pub async fn write_registers(&mut self, register: u8, values: &[u8]) -> Result<(), ()> {
        match self {
            RegisterDevice::Spi(device) => device.write_registers(register, values).await,
            RegisterDevice::I2c(device) => device.write_registers(register, values).await,
        }
    }

    pub async fn read_registers(&mut self, register: u8, buffer: &mut [u8]) -> Result<(), ()> {
        match self {
            RegisterDevice::Spi(device) => device.read_registers(register, buffer).await,
            RegisterDevice::I2c(device) => device.read_registers(register, buffer).await,
        }
    }

    pub async fn read_i16_le(&mut self, start_register: u8) -> Result<i16, ()> {
        let mut bytes = [0u8; 2];
        self.read_registers(start_register, &mut bytes).await?;
        Ok(i16::from_le_bytes(bytes))
    }

    pub async fn read_i16_be(&mut self, start_register: u8) -> Result<i16, ()> {
        let mut bytes = [0u8; 2];
        self.read_registers(start_register, &mut bytes).await?;
        Ok(i16::from_be_bytes(bytes))
    }

    pub async fn read_xyz_le(&mut self, start_register: u8) -> Result<[i16; 3], ()> {
        let mut bytes = [0u8; 6];
        self.read_registers(start_register, &mut bytes).await?;
        Ok(read_xyz_le(&bytes))
    }

    pub async fn read_xyz_be(&mut self, start_register: u8) -> Result<[i16; 3], ()> {
        let mut bytes = [0u8; 6];
        self.read_registers(start_register, &mut bytes).await?;
        Ok(read_xyz_be(&bytes))
    }
}

impl SpiRegisterDevice {
    async fn read_registers(&mut self, register: u8, buffer: &mut [u8]) -> Result<(), ()> {
        let mut frame = [0u8; 16];
        let frame_len = 1 + self.config.read_dummy_bytes + buffer.len();
        let data_start = 1 + self.config.read_dummy_bytes;
        let spi_frame = frame.get_mut(..frame_len).ok_or(())?;
        spi_frame[0] = register | self.config.read_mask;
        self.transfer(spi_frame).await?;
        buffer.copy_from_slice(&spi_frame[data_start..]);
        Ok(())
    }

    async fn write_registers(&mut self, register: u8, values: &[u8]) -> Result<(), ()> {
        let mut frame = [0u8; 16];
        let frame_len = 1 + values.len();
        let spi_frame = frame.get_mut(..frame_len).ok_or(())?;
        spi_frame[0] = register & self.config.write_mask;
        spi_frame[1..].copy_from_slice(values);
        self.write(spi_frame).await
    }

    async fn transfer(&mut self, buffer: &mut [u8]) -> Result<(), ()> {
        let mut spi = self.spi.lock().await;
        self.cs.set_low();
        let result = SpiBus::transfer_in_place(&mut *spi, buffer).await;
        let flush_result = SpiBus::flush(&mut *spi).await;
        self.cs.set_high();
        result.map_err(|_| ())?;
        flush_result.map_err(|_| ())?;
        Ok(())
    }

    async fn write(&mut self, buffer: &[u8]) -> Result<(), ()> {
        let mut spi = self.spi.lock().await;
        self.cs.set_low();
        let result = SpiBus::write(&mut *spi, buffer).await;
        let flush_result = SpiBus::flush(&mut *spi).await;
        self.cs.set_high();
        result.map_err(|_| ())?;
        flush_result.map_err(|_| ())?;
        Ok(())
    }
}

impl I2cRegisterDevice {
    async fn read_registers(&mut self, register: u8, buffer: &mut [u8]) -> Result<(), ()> {
        let mut i2c = self.i2c.lock().await;
        I2c::write_read(&mut *i2c, self.address, &[register], buffer)
            .await
            .map_err(|_| ())
    }

    async fn write_registers(&mut self, register: u8, values: &[u8]) -> Result<(), ()> {
        let mut frame = [0u8; 16];
        let frame_len = 1 + values.len();
        let i2c_frame = frame.get_mut(..frame_len).ok_or(())?;
        i2c_frame[0] = register;
        i2c_frame[1..].copy_from_slice(values);

        let mut i2c = self.i2c.lock().await;
        I2c::write(&mut *i2c, self.address, i2c_frame)
            .await
            .map_err(|_| ())
    }
}

pub fn read_xyz_le(bytes: &[u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([bytes[0], bytes[1]]),
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}

pub fn read_xyz_be(bytes: &[u8; 6]) -> [i16; 3] {
    [
        i16::from_be_bytes([bytes[0], bytes[1]]),
        i16::from_be_bytes([bytes[2], bytes[3]]),
        i16::from_be_bytes([bytes[4], bytes[5]]),
    ]
}

pub fn log_bus_error(label: &str) {
    error!("{} bus error", label);
}
