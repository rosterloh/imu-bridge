use crate::sensor::{
    AccelFullScale, FullScaleSelection, GyroFullScale, OdrSelection, OutputDataRate, SensorSettings,
};

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImuKind {
    Ism330dlc,
    Bmi088,
    Icm45686,
    Icm42688p,
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImuTransport {
    Spi,
    I2c,
}

pub const IMU_KIND: ImuKind = ImuKind::Icm45686;
pub const IMU_TRANSPORT: ImuTransport = ImuTransport::I2c;
pub const SENSOR_SETTINGS: SensorSettings = SensorSettings {
    full_scale: FullScaleSelection {
        accel: AccelFullScale::from_g(16),
        gyro: GyroFullScale::from_dps(2000),
    },
    odr: OdrSelection {
        accel: OutputDataRate::from_hz(25),
        gyro: OutputDataRate::from_hz(25),
    },
};

pub const SENSOR_BOOT_DELAY_MS: u64 = 25;
pub const SENSOR_POLL_INTERVAL_MS: u64 = 100;

pub const SENSOR_MESSAGE_CAPACITY: usize = 128;
pub const NTP_SERVER_IP: [u8; 4] = [162, 159, 200, 1]; // time.cloudflare.com
pub const NTP_SERVER_PORT: u16 = 123;
pub const NTP_TIMEOUT_MS: u64 = 3000;
pub const ZENOH_ROUTER_ENDPOINT: &str = "tcp/192.168.1.10:7447";
pub const ZENOH_KEYEXPR: &str = "imu/reading";
pub const ZENOH_PAYLOAD_CAPACITY: usize = 13;

pub fn zenoh_router_endpoint() -> &'static str {
    option_env!("ZENOH_ROUTER_ENDPOINT").unwrap_or(ZENOH_ROUTER_ENDPOINT)
}
