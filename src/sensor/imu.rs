use defmt::error;
use embedded_hal_async::spi::SpiBus;
use esp_hal::gpio::Output;

use crate::board::SharedSpi;

pub struct RegisterDevice {
    spi: &'static SharedSpi,
    cs: Output<'static>,
}

impl RegisterDevice {
    pub fn new(spi: &'static SharedSpi, mut cs: Output<'static>) -> Self {
        cs.set_high();
        Self { spi, cs }
    }

    pub async fn read_register(&mut self, register: u8) -> Result<u8, ()> {
        let mut buf = [register | 0x80, 0];
        self.transfer(&mut buf).await?;
        Ok(buf[1])
    }

    pub async fn write_register(&mut self, register: u8, value: u8) -> Result<(), ()> {
        let buf = [register & 0x7f, value];
        let mut spi = self.spi.lock().await;
        self.cs.set_low();
        let result = SpiBus::write(&mut *spi, &buf).await;
        let flush_result = SpiBus::flush(&mut *spi).await;
        self.cs.set_high();
        result.map_err(|_| ())?;
        flush_result.map_err(|_| ())?;
        Ok(())
    }

    pub async fn read_i16(&mut self, start_register: u8) -> Result<i16, ()> {
        let mut buf = [0u8; 3];
        buf[0] = start_register | 0x80;
        self.transfer(&mut buf).await?;
        Ok(i16::from_le_bytes([buf[1], buf[2]]))
    }

    pub async fn read_xyz(&mut self, start_register: u8) -> Result<[i16; 3], ()> {
        let mut buf = [0u8; 7];
        buf[0] = start_register | 0x80;
        self.transfer(&mut buf).await?;

        Ok([
            i16::from_le_bytes([buf[1], buf[2]]),
            i16::from_le_bytes([buf[3], buf[4]]),
            i16::from_le_bytes([buf[5], buf[6]]),
        ])
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
}

pub fn log_bus_error(label: &str) {
    error!("{} bus error", label);
}
