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

const REG_ACCEL_DATA_X1_UI: u8 = 0x00;
const REG_GYRO_DATA_X1_UI: u8 = 0x06;
const REG_TEMP_DATA1_UI: u8 = 0x0c;
const REG_PWR_MGMT0: u8 = 0x10;
const REG_ACCEL_CONFIG0: u8 = 0x1b;
const REG_GYRO_CONFIG0: u8 = 0x1c;
const REG_WHO_AM_I: u8 = 0x72;

const WHO_AM_I_EXPECTED: u8 = 0xe9;
const POWER_MODE_LOW_NOISE: u8 = 0x0f;
const I2C_ADDRESS_PRIMARY: u8 = 0x68;

const ACCEL_RANGES: [AccelRangeSetting; 5] = [
    AccelRangeSetting::new(AccelFullScale::from_g(2), 0x40),
    AccelRangeSetting::new(AccelFullScale::from_g(4), 0x30),
    AccelRangeSetting::new(AccelFullScale::from_g(8), 0x20),
    AccelRangeSetting::new(AccelFullScale::from_g(16), 0x10),
    AccelRangeSetting::new(AccelFullScale::from_g(32), 0x00),
];
const GYRO_RANGES: [GyroRangeSetting; 9] = [
    GyroRangeSetting::new(GyroFullScale::from_thousandths_dps(15_625), 0x80),
    GyroRangeSetting::new(GyroFullScale::from_thousandths_dps(31_250), 0x70),
    GyroRangeSetting::new(GyroFullScale::from_thousandths_dps(62_500), 0x60),
    GyroRangeSetting::new(GyroFullScale::from_dps(125), 0x50),
    GyroRangeSetting::new(GyroFullScale::from_dps(250), 0x40),
    GyroRangeSetting::new(GyroFullScale::from_dps(500), 0x30),
    GyroRangeSetting::new(GyroFullScale::from_dps(1000), 0x20),
    GyroRangeSetting::new(GyroFullScale::from_dps(2000), 0x10),
    GyroRangeSetting::new(GyroFullScale::from_dps(4000), 0x00),
];
const ACCEL_ODRS: [OdrSetting; 13] = [
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
    OdrSetting::new(OutputDataRate::from_hz(3200), 0x04),
    OdrSetting::new(OutputDataRate::from_hz(6400), 0x03),
];
const GYRO_ODRS: [OdrSetting; 13] = [
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
    OdrSetting::new(OutputDataRate::from_hz(3200), 0x04),
    OdrSetting::new(OutputDataRate::from_hz(6400), 0x03),
];
const _: () = assert!(accel_table_is_sorted_and_nonzero(&ACCEL_RANGES));
const _: () = assert!(gyro_table_is_sorted_and_nonzero(&GYRO_RANGES));
const _: () = assert!(odr_table_is_sorted_and_nonzero(&ACCEL_ODRS));
const _: () = assert!(odr_table_is_sorted_and_nonzero(&GYRO_ODRS));

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidDeviceId(u8),
}

pub struct Icm45686 {
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
) -> Result<Icm45686, SensorError> {
    init_with_device(
        RegisterDevice::new_spi(spi, cs, SpiBusConfig::standard()),
        settings,
    )
    .await
}

pub async fn init_i2c(
    i2c: &'static SharedI2c,
    settings: SensorSettings,
) -> Result<Icm45686, SensorError> {
    init_with_device(RegisterDevice::new_i2c(i2c, I2C_ADDRESS_PRIMARY), settings).await
}

async fn init_with_device(
    device: RegisterDevice,
    requested_settings: SensorSettings,
) -> Result<Icm45686, SensorError> {
    let mut sensor = Icm45686 {
        device,
        accel_range: select_accel_range(requested_settings.full_scale.accel, &ACCEL_RANGES),
        gyro_range: select_gyro_range(requested_settings.full_scale.gyro, &GYRO_RANGES),
        accel_odr: select_odr(requested_settings.odr.accel, &ACCEL_ODRS),
        gyro_odr: select_odr(requested_settings.odr.gyro, &GYRO_ODRS),
    };

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

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
        .write_register(REG_PWR_MGMT0, POWER_MODE_LOW_NOISE)
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
        .write_register(
            REG_GYRO_CONFIG0,
            sensor.gyro_range.register_value | sensor.gyro_odr.register_value,
        )
        .await
        .map_err(|_| SensorError::Bus)?;

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;
    log_selected_settings("ICM-45686", requested_settings, sensor.settings());

    Ok(sensor)
}

pub async fn read_ready(
    sensor: &mut Icm45686,
    timestamp_us: u64,
) -> Result<SensorReading, SensorError> {
    let accel = sensor
        .device
        .read_xyz_be(REG_ACCEL_DATA_X1_UI)
        .await
        .map_err(|_| SensorError::Bus)?;
    let gyro = sensor
        .device
        .read_xyz_be(REG_GYRO_DATA_X1_UI)
        .await
        .map_err(|_| SensorError::Bus)?;
    let temp_raw = sensor
        .device
        .read_i16_be(REG_TEMP_DATA1_UI)
        .await
        .map_err(|_| SensorError::Bus)? as f32;

    Ok(SensorReading {
        timestamp_us,
        acceleration_mg: accel.map(|value| value as f32 * sensor.accel_range.mg_per_lsb()),
        angular_rate_mdps: gyro.map(|value| value as f32 * sensor.gyro_range.mdps_per_lsb()),
        temperature_c: (temp_raw / 128.0) + 25.0,
    })
}

impl Icm45686 {
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
        SensorError::Bus => error!("ICM-45686 bus error"),
        SensorError::InvalidDeviceId(id) => warn!("Invalid ICM-45686 ID {}", id),
    }
}
