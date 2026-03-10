use defmt::{error, info, warn};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Ipv4Address, Runner, Stack};
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::rtc_cntl::Rtc;
use esp_radio::wifi::{
    ClientConfig, ModeConfig, ScanConfig, WifiController, WifiDevice, WifiEvent, WifiStaState,
};
use static_cell::StaticCell;
use zenoh_embassy::EmbassyLinkManager;
use zenoh_nostd::platform::Endpoint;
use zenoh_nostd::session::{
    FixedCapacityGetCallbacks, FixedCapacityQueryableCallbacks, FixedCapacitySubCallbacks,
    Resources, Session, TransportLinkManager, ZSessionConfig, zenoh::storage::RawOrBox,
};

use crate::config;

const SSID: Option<&str> = option_env!("SSID");
const PASSWORD: Option<&str> = option_env!("PASSWORD");
const RECONNECT_DELAY_MS: u64 = 5000;
const RADIO_RETRY_DELAY_MS: u64 = 1000;
const NTP_PACKET_LEN: usize = 48;
const NTP_UNIX_EPOCH_OFFSET_SECS: u64 = 2_208_988_800;

pub struct ZenohConfig {
    transports: TransportLinkManager<EmbassyLinkManager<'static, 512, 3>>,
}
impl ZenohConfig {
    pub fn new(stack: Stack<'static>) -> Self {
        Self {
            transports: TransportLinkManager::from(EmbassyLinkManager::new(stack)),
        }
    }
}

impl ZSessionConfig for ZenohConfig {
    type LinkManager = EmbassyLinkManager<'static, 512, 3>;
    type Buff = [u8; 512];
    type SubCallbacks<'res> = FixedCapacitySubCallbacks<'res, 8, RawOrBox<56>, RawOrBox<600>>;
    type GetCallbacks<'res> = FixedCapacityGetCallbacks<'res, 8, RawOrBox<1>, RawOrBox<32>>;
    type QueryableCallbacks<'res> =
        FixedCapacityQueryableCallbacks<'res, Self, 8, RawOrBox<32>, RawOrBox<952>>;

    fn buff(&self) -> Self::Buff {
        [0u8; 512]
    }

    fn transports(&self) -> &TransportLinkManager<Self::LinkManager> {
        &self.transports
    }
}

pub async fn zenoh_connect_static(
    stack: Stack<'static>,
    endpoint: Endpoint<'_>,
) -> Option<&'static Session<'static, ZenohConfig>> {
    static CONFIG: StaticCell<ZenohConfig> = StaticCell::new();
    static RESOURCES: StaticCell<Resources<'static, ZenohConfig>> = StaticCell::new();
    static SESSION: StaticCell<Session<'static, ZenohConfig>> = StaticCell::new();
    let config = CONFIG.init(ZenohConfig::new(stack));
    let resources = RESOURCES.init(Resources::default());
    let transport = match config.transports().connect(endpoint, config.buff()).await {
        Ok(transport) => transport,
        Err(_) => return None,
    };
    Some(SESSION.init(Session::new(resources.init(transport))))
}

pub async fn sync_rtc_from_ntp(stack: Stack<'static>, rtc: &Rtc<'_>) {
    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buffer = [0u8; NTP_PACKET_LEN];
    let mut tx_buffer = [0u8; NTP_PACKET_LEN];
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    if let Err(error) = socket.bind(0) {
        warn!("NTP bind failed: {:?}", error);
        return;
    }

    let mut request = [0u8; NTP_PACKET_LEN];
    request[0] = 0x1b;

    let server = (
        Ipv4Address::new(
            config::NTP_SERVER_IP[0],
            config::NTP_SERVER_IP[1],
            config::NTP_SERVER_IP[2],
            config::NTP_SERVER_IP[3],
        ),
        config::NTP_SERVER_PORT,
    );

    if let Err(error) = socket.send_to(&request, server).await {
        warn!("NTP send failed: {:?}", error);
        return;
    }

    let mut response = [0u8; NTP_PACKET_LEN];
    let recv_result = with_timeout(
        Duration::from_millis(config::NTP_TIMEOUT_MS),
        socket.recv_from(&mut response),
    )
    .await;

    let Ok(Ok((received, _remote))) = recv_result else {
        warn!("NTP receive failed or timed out");
        return;
    };

    if received < NTP_PACKET_LEN {
        warn!("NTP response too short: {}", received);
        return;
    }

    let seconds = u32::from_be_bytes([response[40], response[41], response[42], response[43]]);
    let fraction = u32::from_be_bytes([response[44], response[45], response[46], response[47]]);
    let seconds = seconds as u64;
    if seconds < NTP_UNIX_EPOCH_OFFSET_SECS {
        warn!("NTP response predates Unix epoch");
        return;
    }

    let unix_seconds = seconds - NTP_UNIX_EPOCH_OFFSET_SECS;
    let micros = ((fraction as u64) * 1_000_000) >> 32;
    let unix_time_us = unix_seconds
        .saturating_mul(1_000_000)
        .saturating_add(micros);

    rtc.set_current_time_us(unix_time_us);
    info!("RTC synced from NTP: {} us since Unix epoch", unix_time_us);
}

#[embassy_executor::task]
pub async fn connection(mut controller: WifiController<'static>) {
    info!("Start connection task");

    let ssid = SSID.unwrap_or("ssid");
    let password = PASSWORD.unwrap_or("password");

    info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        info!("Wi-Fi (re)init");
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(RECONNECT_DELAY_MS)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(ssid.into())
                    .with_password(password.into()),
            );
            if let Err(e) = controller.set_config(&client_config) {
                error!("Failed to configure radio stack: {:?}, retrying...", e);
                Timer::after(Duration::from_millis(RADIO_RETRY_DELAY_MS)).await;
                continue;
            }
            info!("Starting Wi-Fi");
            if let Err(e) = controller.start_async().await {
                error!("Failed to start radio stack: {:?}, retrying...", e);
                Timer::after(Duration::from_millis(RADIO_RETRY_DELAY_MS)).await;
                continue;
            }
            info!("Wi-Fi started!");

            info!("Scan");
            let scan_config = ScanConfig::default().with_max(10);
            let result = controller
                .scan_with_config_async(scan_config)
                .await
                .unwrap();
            for ap in &result {
                info!("{:?}", ap);
            }
            if !result.iter().any(|ap| ap.ssid == ssid) {
                error!("SSID {} not yet available", ssid);
            }
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wi-Fi connected!"),
            Err(e) => {
                error!("Failed to connect to Wi-Fi: {:?}", e);
                Timer::after(Duration::from_millis(RECONNECT_DELAY_MS)).await
            }
        }
    }
}

#[embassy_executor::task]
pub async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
