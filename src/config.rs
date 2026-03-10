#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImuKind {
    Ism330dlc,
    Bmi088,
    Icm45686,
    Icm42688p,
}

pub const IMU_KIND: ImuKind = ImuKind::Ism330dlc;

pub const SENSOR_BOOT_DELAY_MS: u64 = 25;
pub const SENSOR_POLL_INTERVAL_MS: u64 = 100;

pub const SENSOR_MESSAGE_CAPACITY: usize = 48;
pub const NTP_SERVER_IP: [u8; 4] = [162, 159, 200, 1]; // time.cloudflare.com
pub const NTP_SERVER_PORT: u16 = 123;
pub const NTP_TIMEOUT_MS: u64 = 3000;
pub const ZENOH_ROUTER_ENDPOINT: &str = "tcp/192.168.1.10:7447";
pub const ZENOH_KEYEXPR: &str = "imu/reading";
pub const ZENOH_PAYLOAD_CAPACITY: usize = 13;

pub fn zenoh_router_endpoint() -> &'static str {
    option_env!("ZENOH_ROUTER_ENDPOINT").unwrap_or(ZENOH_ROUTER_ENDPOINT)
}
