use ism330dhcx_rs::asynchronous::register::main::{FsGy, FsXl, HpSlopeXlEn, OdrGy, OdrXl};

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImuKind {
    Ism330dhcx,
    Bmi088,
}

pub const IMU_KIND: ImuKind = ImuKind::Ism330dhcx;

pub const SENSOR_BOOT_DELAY_MS: u64 = 25;
pub const SENSOR_POLL_INTERVAL_MS: u64 = 100;

pub const ACCEL_ODR: OdrXl = OdrXl::_12_5hz;
pub const GYRO_ODR: OdrGy = OdrGy::_12_5hz;

pub const ACCEL_FULL_SCALE: FsXl = FsXl::_2g;
pub const GYRO_FULL_SCALE: FsGy = FsGy::_2000dps;

pub const ACCEL_FILTER_PATH: HpSlopeXlEn = HpSlopeXlEn::LpOdrDiv100;

pub const SENSOR_MESSAGE_CAPACITY: usize = 48;
pub const ZENOH_ROUTER_ENDPOINT: &str = "tcp/192.168.1.10:7447";
pub const ZENOH_KEYEXPR: &str = "imu/reading";
pub const ZENOH_PAYLOAD_CAPACITY: usize = 13;

pub fn zenoh_router_endpoint() -> &'static str {
    option_env!("ZENOH_ROUTER_ENDPOINT").unwrap_or(ZENOH_ROUTER_ENDPOINT)
}

// BMI088 defaults (SPI)
pub const BMI088_SPI_CLOCK_KHZ_MIN: u32 = 1000;
pub const BMI088_SPI_CLOCK_KHZ_MAX: u32 = 5000;
