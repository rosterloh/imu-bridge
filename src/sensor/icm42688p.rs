use defmt::{error, warn};
use embassy_time::Timer;
use esp_hal::gpio::Output;

use crate::{
    board::{SharedI2c, SharedSpi},
    config,
    domain::sensor_reading::SensorReading,
};

use super::{
    full_scale::{
        AccelFullScale, AccelRangeSetting, FullScaleSelection, GyroFullScale, GyroRangeSetting,
        OdrSelection, OdrSetting, OutputDataRate, SensorSettings,
        accel_table_is_sorted_and_nonzero, gyro_table_is_sorted_and_nonzero, log_selected_settings,
        odr_table_is_sorted_and_nonzero, select_accel_range, select_gyro_range, select_odr,
    },
    imu::{RegisterDevice, SpiBusConfig},
};

const REG_DEVICE_CONFIG: u8 = 0x11;
const REG_TEMP_DATA1: u8 = 0x1d;
const REG_ACCEL_DATA_X1: u8 = 0x1f;
const REG_GYRO_DATA_X1: u8 = 0x25;
const REG_INT_STATUS: u8 = 0x2d;
const REG_PWR_MGMT0: u8 = 0x4e;
const REG_GYRO_CONFIG0: u8 = 0x4f;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_WHO_AM_I: u8 = 0x75;

const DEVICE_SOFT_RESET: u8 = 0x01;
const WHO_AM_I_EXPECTED: u8 = 0x47;
const PWR_MGMT0_ACCEL_GYRO_LOW_NOISE: u8 = 0x0f;
const INT_STATUS_DATA_RDY: u8 = 0x08;
const I2C_ADDRESS_PRIMARY: u8 = 0x68;

const ACCEL_RANGES: [AccelRangeSetting; 4] = [
    AccelRangeSetting::new(AccelFullScale::from_g(2), 0x60),
    AccelRangeSetting::new(AccelFullScale::from_g(4), 0x40),
    AccelRangeSetting::new(AccelFullScale::from_g(8), 0x20),
    AccelRangeSetting::new(AccelFullScale::from_g(16), 0x00),
];
const GYRO_RANGES: [GyroRangeSetting; 8] = [
    GyroRangeSetting::new(GyroFullScale::from_thousandths_dps(15_625), 0xe0),
    GyroRangeSetting::new(GyroFullScale::from_thousandths_dps(31_250), 0xc0),
    GyroRangeSetting::new(GyroFullScale::from_thousandths_dps(62_500), 0xa0),
    GyroRangeSetting::new(GyroFullScale::from_dps(125), 0x80),
    GyroRangeSetting::new(GyroFullScale::from_dps(250), 0x60),
    GyroRangeSetting::new(GyroFullScale::from_dps(500), 0x40),
    GyroRangeSetting::new(GyroFullScale::from_dps(1000), 0x20),
    GyroRangeSetting::new(GyroFullScale::from_dps(2000), 0x00),
];
const ACCEL_ODRS: [OdrSetting; 11] = [
    OdrSetting::new(OutputDataRate::from_millihz(1562), 0x0F),
    OdrSetting::new(OutputDataRate::from_millihz(3125), 0x0E),
    OdrSetting::new(OutputDataRate::from_millihz(6250), 0x0D),
    OdrSetting::new(OutputDataRate::from_millihz(12_500), 0x0C),
    OdrSetting::new(OutputDataRate::from_hz(25), 0x0B),
    OdrSetting::new(OutputDataRate::from_hz(50), 0x0A),
    OdrSetting::new(OutputDataRate::from_hz(100), 0x09),
    OdrSetting::new(OutputDataRate::from_hz(200), 0x08),
    OdrSetting::new(OutputDataRate::from_hz(400), 0x07),
    OdrSetting::new(OutputDataRate::from_hz(800), 0x06),
    OdrSetting::new(OutputDataRate::from_hz(1600), 0x05),
];
const GYRO_ODRS: [OdrSetting; 11] = [
    OdrSetting::new(OutputDataRate::from_millihz(1562), 0x0F),
    OdrSetting::new(OutputDataRate::from_millihz(3125), 0x0E),
    OdrSetting::new(OutputDataRate::from_millihz(6250), 0x0D),
    OdrSetting::new(OutputDataRate::from_millihz(12_500), 0x0C),
    OdrSetting::new(OutputDataRate::from_hz(25), 0x0B),
    OdrSetting::new(OutputDataRate::from_hz(50), 0x0A),
    OdrSetting::new(OutputDataRate::from_hz(100), 0x09),
    OdrSetting::new(OutputDataRate::from_hz(200), 0x08),
    OdrSetting::new(OutputDataRate::from_hz(400), 0x07),
    OdrSetting::new(OutputDataRate::from_hz(800), 0x06),
    OdrSetting::new(OutputDataRate::from_hz(1600), 0x05),
];
const _: () = assert!(accel_table_is_sorted_and_nonzero(&ACCEL_RANGES));
const _: () = assert!(gyro_table_is_sorted_and_nonzero(&GYRO_RANGES));
const _: () = assert!(odr_table_is_sorted_and_nonzero(&ACCEL_ODRS));
const _: () = assert!(odr_table_is_sorted_and_nonzero(&GYRO_ODRS));

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidDeviceId(u8),
    DataNotReady,
}

pub struct Icm42688p {
    device: RegisterDevice,
    accel_range: AccelRangeSetting,
    gyro_range: GyroRangeSetting,
    accel_odr: OdrSetting,
    gyro_odr: OdrSetting,
}

pub async fn init_spi(
    spi: &'static SharedSpi,
    cs: Output<'static>,
    settings: SensorSettings,
) -> Result<Icm42688p, SensorError> {
    init_with_device(
        RegisterDevice::new_spi(spi, cs, SpiBusConfig::standard()),
        settings,
    )
    .await
}

pub async fn init_i2c(
    i2c: &'static SharedI2c,
    settings: SensorSettings,
) -> Result<Icm42688p, SensorError> {
    init_with_device(RegisterDevice::new_i2c(i2c, I2C_ADDRESS_PRIMARY), settings).await
}

async fn init_with_device(
    device: RegisterDevice,
    requested_settings: SensorSettings,
) -> Result<Icm42688p, SensorError> {
    let mut sensor = Icm42688p {
        device,
        accel_range: select_accel_range(requested_settings.full_scale.accel, &ACCEL_RANGES),
        gyro_range: select_gyro_range(requested_settings.full_scale.gyro, &GYRO_RANGES),
        accel_odr: select_odr(requested_settings.odr.accel, &ACCEL_ODRS),
        gyro_odr: select_odr(requested_settings.odr.gyro, &GYRO_ODRS),
    };

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    sensor
        .device
        .write_register(REG_DEVICE_CONFIG, DEVICE_SOFT_RESET)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(1).await;

    let device_id = sensor
        .device
        .read_register(REG_WHO_AM_I)
        .await
        .map_err(|_| SensorError::Bus)?;
    if device_id != WHO_AM_I_EXPECTED {
        return Err(SensorError::InvalidDeviceId(device_id));
    }

    sensor
        .device
        .write_register(
            REG_GYRO_CONFIG0,
            sensor.gyro_range.register_value | sensor.gyro_odr.register_value,
        )
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .device
        .write_register(
            REG_ACCEL_CONFIG0,
            sensor.accel_range.register_value | sensor.accel_odr.register_value,
        )
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .device
        .write_register(REG_PWR_MGMT0, PWR_MGMT0_ACCEL_GYRO_LOW_NOISE)
        .await
        .map_err(|_| SensorError::Bus)?;

    Timer::after_millis(1).await;
    log_selected_settings("ICM-42688-P", requested_settings, sensor.settings());

    Ok(sensor)
}

pub async fn read_ready(
    sensor: &mut Icm42688p,
    timestamp_us: u64,
) -> Result<SensorReading, SensorError> {
    let status = sensor
        .device
        .read_register(REG_INT_STATUS)
        .await
        .map_err(|_| SensorError::Bus)?;
    if status & INT_STATUS_DATA_RDY == 0 {
        return Err(SensorError::DataNotReady);
    }

    let accel = sensor
        .device
        .read_xyz_be(REG_ACCEL_DATA_X1)
        .await
        .map_err(|_| SensorError::Bus)?;
    let gyro = sensor
        .device
        .read_xyz_be(REG_GYRO_DATA_X1)
        .await
        .map_err(|_| SensorError::Bus)?;
    let temp_raw = sensor
        .device
        .read_i16_be(REG_TEMP_DATA1)
        .await
        .map_err(|_| SensorError::Bus)? as f32;

    Ok(SensorReading {
        timestamp_us,
        acceleration_mg: accel.map(|value| value as f32 * sensor.accel_range.mg_per_lsb()),
        angular_rate_mdps: gyro.map(|value| value as f32 * sensor.gyro_range.mdps_per_lsb()),
        temperature_c: (temp_raw / 132.48) + 25.0,
    })
}

impl Icm42688p {
    pub fn settings(&self) -> SensorSettings {
        SensorSettings {
            full_scale: FullScaleSelection {
                accel: self.accel_range.full_scale,
                gyro: self.gyro_range.full_scale,
            },
            odr: OdrSelection {
                accel: self.accel_odr.odr,
                gyro: self.gyro_odr.odr,
            },
        }
    }

    pub fn full_scale(&self) -> FullScaleSelection {
        self.settings().full_scale
    }

    pub fn odr(&self) -> OdrSelection {
        self.settings().odr
    }
}

pub fn log_error(error: SensorError) {
    match error {
        SensorError::Bus => error!("ICM-42688-P bus error"),
        SensorError::InvalidDeviceId(id) => warn!("Invalid ICM-42688-P ID {}", id),
        SensorError::DataNotReady => warn!("ICM-42688-P data not ready"),
    }
}
