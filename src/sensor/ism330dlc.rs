use defmt::{debug, info, warn};
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
    imu::{RegisterDevice, SpiBusConfig, log_bus_error},
};

const REG_WHO_AM_I: u8 = 0x0f;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL2_G: u8 = 0x11;
const REG_CTRL3_C: u8 = 0x12;
const REG_STATUS_REG: u8 = 0x1e;
const REG_OUT_TEMP_L: u8 = 0x20;
const REG_OUTX_L_G: u8 = 0x22;
const REG_OUTX_L_XL: u8 = 0x28;

const WHO_AM_I_EXPECTED: u8 = 0x6b;
const CTRL3_C_IF_INC_BDU: u8 = 0x44;
const CTRL3_C_SW_RESET: u8 = 0x01;

const STATUS_XLDA: u8 = 0x01;
const STATUS_GDA: u8 = 0x02;
const STATUS_TDA: u8 = 0x04;
const RESET_POLL_LIMIT: usize = 100;
const I2C_ADDRESS_PRIMARY: u8 = 0x6a;

const ACCEL_RANGES: [AccelRangeSetting; 4] = [
    AccelRangeSetting::new(AccelFullScale::from_g(2), 0x10),
    AccelRangeSetting::new(AccelFullScale::from_g(4), 0x18),
    AccelRangeSetting::new(AccelFullScale::from_g(8), 0x1c),
    AccelRangeSetting::new(AccelFullScale::from_g(16), 0x14),
];
const GYRO_RANGES: [GyroRangeSetting; 5] = [
    GyroRangeSetting::new(GyroFullScale::from_dps(125), 0x12),
    GyroRangeSetting::new(GyroFullScale::from_dps(250), 0x10),
    GyroRangeSetting::new(GyroFullScale::from_dps(500), 0x14),
    GyroRangeSetting::new(GyroFullScale::from_dps(1000), 0x18),
    GyroRangeSetting::new(GyroFullScale::from_dps(2000), 0x1c),
];
const ACCEL_ODRS: [OdrSetting; 10] = [
    OdrSetting::new(OutputDataRate::from_millihz(12_500), 0x10),
    OdrSetting::new(OutputDataRate::from_hz(26), 0x20),
    OdrSetting::new(OutputDataRate::from_hz(52), 0x30),
    OdrSetting::new(OutputDataRate::from_hz(104), 0x40),
    OdrSetting::new(OutputDataRate::from_hz(208), 0x50),
    OdrSetting::new(OutputDataRate::from_hz(416), 0x60),
    OdrSetting::new(OutputDataRate::from_hz(833), 0x70),
    OdrSetting::new(OutputDataRate::from_hz(1660), 0x80),
    OdrSetting::new(OutputDataRate::from_hz(3330), 0x90),
    OdrSetting::new(OutputDataRate::from_hz(6660), 0xA0),
];
const GYRO_ODRS: [OdrSetting; 10] = [
    OdrSetting::new(OutputDataRate::from_millihz(12_500), 0x10),
    OdrSetting::new(OutputDataRate::from_hz(26), 0x20),
    OdrSetting::new(OutputDataRate::from_hz(52), 0x30),
    OdrSetting::new(OutputDataRate::from_hz(104), 0x40),
    OdrSetting::new(OutputDataRate::from_hz(208), 0x50),
    OdrSetting::new(OutputDataRate::from_hz(416), 0x60),
    OdrSetting::new(OutputDataRate::from_hz(833), 0x70),
    OdrSetting::new(OutputDataRate::from_hz(1660), 0x80),
    OdrSetting::new(OutputDataRate::from_hz(3330), 0x90),
    OdrSetting::new(OutputDataRate::from_hz(6660), 0xA0),
];
const _: () = assert!(accel_table_is_sorted_and_nonzero(&ACCEL_RANGES));
const _: () = assert!(gyro_table_is_sorted_and_nonzero(&GYRO_RANGES));
const _: () = assert!(odr_table_is_sorted_and_nonzero(&ACCEL_ODRS));
const _: () = assert!(odr_table_is_sorted_and_nonzero(&GYRO_ODRS));

pub struct Ism330dlc {
    device: RegisterDevice,
    accel_range: AccelRangeSetting,
    gyro_range: GyroRangeSetting,
    accel_odr: OdrSetting,
    gyro_odr: OdrSetting,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidDeviceId(u8),
}

pub async fn init_spi(
    spi: &'static SharedSpi,
    cs: Output<'static>,
    settings: SensorSettings,
) -> Result<Ism330dlc, SensorError> {
    init_with_device(
        RegisterDevice::new_spi(spi, cs, SpiBusConfig::standard()),
        settings,
    )
    .await
}

pub async fn init_i2c(
    i2c: &'static SharedI2c,
    settings: SensorSettings,
) -> Result<Ism330dlc, SensorError> {
    init_with_device(RegisterDevice::new_i2c(i2c, I2C_ADDRESS_PRIMARY), settings).await
}

async fn init_with_device(
    device: RegisterDevice,
    requested_settings: SensorSettings,
) -> Result<Ism330dlc, SensorError> {
    let mut sensor = Ism330dlc {
        device,
        accel_range: ACCEL_RANGES[0],
        gyro_range: GYRO_RANGES[0],
        accel_odr: ACCEL_ODRS[0],
        gyro_odr: GYRO_ODRS[0],
    };

    info!("Initialising ISM330DLC...");

    let device_id = sensor
        .device
        .read_register(REG_WHO_AM_I)
        .await
        .map_err(|_| SensorError::Bus)?;
    if device_id != WHO_AM_I_EXPECTED {
        return Err(SensorError::InvalidDeviceId(device_id));
    }

    debug!("Device found. Configuring...");

    Timer::after_millis(config::SENSOR_BOOT_DELAY_MS).await;

    sensor
        .device
        .write_register(REG_CTRL3_C, CTRL3_C_SW_RESET)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(10).await;

    for _ in 0..RESET_POLL_LIMIT {
        let ctrl3 = sensor
            .device
            .read_register(REG_CTRL3_C)
            .await
            .map_err(|_| SensorError::Bus)?;
        if ctrl3 & CTRL3_C_SW_RESET == 0 {
            break;
        }
        Timer::after_millis(1).await;
    }

    let ctrl3 = sensor
        .device
        .read_register(REG_CTRL3_C)
        .await
        .map_err(|_| SensorError::Bus)?;
    if ctrl3 & CTRL3_C_SW_RESET != 0 {
        return Err(SensorError::Bus);
    }

    sensor
        .device
        .write_register(REG_CTRL3_C, CTRL3_C_IF_INC_BDU)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor.accel_range = select_accel_range(requested_settings.full_scale.accel, &ACCEL_RANGES);
    sensor.gyro_range = select_gyro_range(requested_settings.full_scale.gyro, &GYRO_RANGES);
    sensor.accel_odr = select_odr(requested_settings.odr.accel, &ACCEL_ODRS);
    sensor.gyro_odr = select_odr(requested_settings.odr.gyro, &GYRO_ODRS);
    sensor
        .device
        .write_register(
            REG_CTRL1_XL,
            sensor.accel_odr.register_value | (sensor.accel_range.register_value & 0x0c),
        )
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .device
        .write_register(
            REG_CTRL2_G,
            sensor.gyro_odr.register_value | (sensor.gyro_range.register_value & 0x0e),
        )
        .await
        .map_err(|_| SensorError::Bus)?;
    log_selected_settings("ISM330DLC", requested_settings, sensor.settings());

    Ok(sensor)
}

pub async fn read_ready(
    sensor: &mut Ism330dlc,
    timestamp_us: u64,
) -> Result<SensorReading, SensorError> {
    let status = sensor
        .device
        .read_register(REG_STATUS_REG)
        .await
        .map_err(|_| SensorError::Bus)?;

    let mut accel: [i16; 3] = [0; 3];
    if status & STATUS_XLDA != 0 {
        accel = sensor
            .device
            .read_xyz_le(REG_OUTX_L_XL)
            .await
            .map_err(|_| SensorError::Bus)?;
    }

    let mut gyro: [i16; 3] = [0; 3];
    if status & STATUS_GDA != 0 {
        gyro = sensor
            .device
            .read_xyz_le(REG_OUTX_L_G)
            .await
            .map_err(|_| SensorError::Bus)?;
    }

    let mut temp_raw: i16 = 0;
    if status & STATUS_TDA != 0 {
        temp_raw = sensor
            .device
            .read_i16_le(REG_OUT_TEMP_L)
            .await
            .map_err(|_| SensorError::Bus)?;
    }

    Ok(SensorReading {
        timestamp_us,
        acceleration_mg: accel.map(|v| v as f32 * sensor.accel_range.mg_per_lsb()),
        angular_rate_mdps: gyro.map(|v| v as f32 * sensor.gyro_range.mdps_per_lsb()),
        temperature_c: 25.0 + temp_raw as f32 / 256.0,
    })
}

impl Ism330dlc {
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
        SensorError::Bus => log_bus_error("ISM330DLC bus error"),
        SensorError::InvalidDeviceId(id) => warn!("Invalid ISM330DLC ID {}", id),
    }
}
