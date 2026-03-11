use defmt::{error, warn};
use embassy_time::Timer;
use esp_hal::gpio::Output;

use crate::{board::SharedSpi, domain::sensor_reading::SensorReading};

use super::{
    full_scale::{
        AccelFullScale, AccelRangeSetting, FullScaleSelection, GyroFullScale, GyroRangeSetting,
        OdrSelection, OdrSetting, OutputDataRate, SensorSettings,
        accel_table_is_sorted_and_nonzero, gyro_table_is_sorted_and_nonzero, log_selected_settings,
        odr_table_is_sorted_and_nonzero, select_accel_range, select_gyro_range, select_odr,
    },
    imu::{RegisterDevice, SpiBusConfig},
};

const ACC_CHIP_ID: u8 = 0x1e;
const GYRO_CHIP_ID: u8 = 0x0f;

const ACC_REG_CHIP_ID: u8 = 0x00;
const ACC_REG_ACCEL_X_LSB: u8 = 0x12;
const ACC_REG_TEMP_MSB: u8 = 0x22;
const ACC_REG_TEMP_LSB: u8 = 0x23;
const ACC_REG_CONF: u8 = 0x40;
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
const ACC_BW_NORMAL: u8 = 0xA0;
const GYRO_POWER_NORMAL: u8 = 0x00;
const ACCEL_RANGES: [AccelRangeSetting; 4] = [
    AccelRangeSetting::new(AccelFullScale::from_g(3), 0x00),
    AccelRangeSetting::new(AccelFullScale::from_g(6), 0x01),
    AccelRangeSetting::new(AccelFullScale::from_g(12), 0x02),
    AccelRangeSetting::new(AccelFullScale::from_g(24), 0x03),
];
const GYRO_RANGES: [GyroRangeSetting; 5] = [
    GyroRangeSetting::new(GyroFullScale::from_dps(125), 0x04),
    GyroRangeSetting::new(GyroFullScale::from_dps(250), 0x03),
    GyroRangeSetting::new(GyroFullScale::from_dps(500), 0x02),
    GyroRangeSetting::new(GyroFullScale::from_dps(1000), 0x01),
    GyroRangeSetting::new(GyroFullScale::from_dps(2000), 0x00),
];
const ACCEL_ODRS: [OdrSetting; 8] = [
    OdrSetting::new(OutputDataRate::from_millihz(12_500), 0x05),
    OdrSetting::new(OutputDataRate::from_hz(25), 0x06),
    OdrSetting::new(OutputDataRate::from_hz(50), 0x07),
    OdrSetting::new(OutputDataRate::from_hz(100), 0x08),
    OdrSetting::new(OutputDataRate::from_hz(200), 0x09),
    OdrSetting::new(OutputDataRate::from_hz(400), 0x0A),
    OdrSetting::new(OutputDataRate::from_hz(800), 0x0B),
    OdrSetting::new(OutputDataRate::from_hz(1600), 0x0C),
];
const GYRO_ODRS: [OdrSetting; 5] = [
    OdrSetting::new(OutputDataRate::from_hz(100), 0x07),
    OdrSetting::new(OutputDataRate::from_hz(200), 0x06),
    OdrSetting::new(OutputDataRate::from_hz(400), 0x03),
    OdrSetting::new(OutputDataRate::from_hz(1000), 0x02),
    OdrSetting::new(OutputDataRate::from_hz(2000), 0x00),
];
const _: () = assert!(accel_table_is_sorted_and_nonzero(&ACCEL_RANGES));
const _: () = assert!(gyro_table_is_sorted_and_nonzero(&GYRO_RANGES));
const _: () = assert!(odr_table_is_sorted_and_nonzero(&ACCEL_ODRS));
const _: () = assert!(odr_table_is_sorted_and_nonzero(&GYRO_ODRS));

const ACCEL_SPI_CONFIG: SpiBusConfig = SpiBusConfig {
    read_mask: 0x80,
    write_mask: 0x7f,
    read_dummy_bytes: 1,
};

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum SensorError {
    Bus,
    InvalidAccelDeviceId(u8),
    InvalidGyroDeviceId(u8),
}

pub struct Bmi088 {
    accel: RegisterDevice,
    gyro: RegisterDevice,
    accel_range: AccelRangeSetting,
    gyro_range: GyroRangeSetting,
    accel_odr: OdrSetting,
    gyro_odr: OdrSetting,
}

pub async fn init_spi(
    spi: &'static SharedSpi,
    cs_accel: Output<'static>,
    cs_gyro: Output<'static>,
    settings: SensorSettings,
) -> Result<Bmi088, SensorError> {
    let mut sensor = Bmi088 {
        accel: RegisterDevice::new_spi(spi, cs_accel, ACCEL_SPI_CONFIG),
        gyro: RegisterDevice::new_spi(spi, cs_gyro, SpiBusConfig::standard()),
        accel_range: select_accel_range(settings.full_scale.accel, &ACCEL_RANGES),
        gyro_range: select_gyro_range(settings.full_scale.gyro, &GYRO_RANGES),
        accel_odr: select_odr(settings.odr.accel, &ACCEL_ODRS),
        gyro_odr: select_odr(settings.odr.gyro, &GYRO_ODRS),
    };

    let accel_id = sensor
        .accel
        .read_register(ACC_REG_CHIP_ID)
        .await
        .map_err(|_| SensorError::Bus)?;
    if accel_id != ACC_CHIP_ID {
        return Err(SensorError::InvalidAccelDeviceId(accel_id));
    }

    let gyro_id = sensor
        .gyro
        .read_register(GYRO_REG_CHIP_ID)
        .await
        .map_err(|_| SensorError::Bus)?;
    if gyro_id != GYRO_CHIP_ID {
        return Err(SensorError::InvalidGyroDeviceId(gyro_id));
    }

    sensor
        .accel
        .write_register(ACC_REG_SOFT_RESET, SOFT_RESET_CMD)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(10).await;

    sensor
        .accel
        .write_register(ACC_REG_PWR_CONF, ACC_PWR_CONF_ACTIVE)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(1).await;
    sensor
        .accel
        .write_register(ACC_REG_PWR_CTRL, ACC_PWR_CTRL_ON)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(50).await;
    sensor
        .accel
        .write_register(
            ACC_REG_CONF,
            ACC_BW_NORMAL | sensor.accel_odr.register_value,
        )
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .accel
        .write_register(ACC_REG_ACC_RANGE, sensor.accel_range.register_value)
        .await
        .map_err(|_| SensorError::Bus)?;

    sensor
        .gyro
        .write_register(GYRO_REG_SOFT_RESET, SOFT_RESET_CMD)
        .await
        .map_err(|_| SensorError::Bus)?;
    Timer::after_millis(100).await;
    sensor
        .gyro
        .write_register(GYRO_REG_RANGE, sensor.gyro_range.register_value)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .gyro
        .write_register(GYRO_REG_BANDWIDTH, sensor.gyro_odr.register_value)
        .await
        .map_err(|_| SensorError::Bus)?;
    sensor
        .gyro
        .write_register(GYRO_REG_POWER_MODE, GYRO_POWER_NORMAL)
        .await
        .map_err(|_| SensorError::Bus)?;

    log_selected_settings("BMI088", settings, sensor.settings());

    Ok(sensor)
}

pub async fn read_ready(
    sensor: &mut Bmi088,
    timestamp_us: u64,
) -> Result<SensorReading, SensorError> {
    let accel = sensor
        .accel
        .read_xyz_le(ACC_REG_ACCEL_X_LSB)
        .await
        .map_err(|_| SensorError::Bus)?;
    let gyro = sensor
        .gyro
        .read_xyz_le(GYRO_REG_X_LSB)
        .await
        .map_err(|_| SensorError::Bus)?;
    let msb = sensor
        .accel
        .read_register(ACC_REG_TEMP_MSB)
        .await
        .map_err(|_| SensorError::Bus)?;
    let lsb = sensor
        .accel
        .read_register(ACC_REG_TEMP_LSB)
        .await
        .map_err(|_| SensorError::Bus)?;
    let temp_raw = (msb as i16) * 8 + (lsb as i16) / 32;

    Ok(SensorReading {
        timestamp_us,
        acceleration_mg: accel.map(|value| value as f32 * sensor.accel_range.mg_per_lsb()),
        angular_rate_mdps: gyro.map(|value| value as f32 * sensor.gyro_range.mdps_per_lsb()),
        temperature_c: (temp_raw as f32) * 0.125 + 23.0,
    })
}

impl Bmi088 {
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
        SensorError::Bus => error!("BMI088 SPI bus error"),
        SensorError::InvalidAccelDeviceId(id) => warn!("Invalid BMI088 accel ID {}", id),
        SensorError::InvalidGyroDeviceId(id) => warn!("Invalid BMI088 gyro ID {}", id),
    }
}
