use defmt::{error, info};
use embassy_net::{Runner, Stack};
use embassy_time::{Duration, Timer};
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

const SSID: Option<&str> = option_env!("SSID");
const PASSWORD: Option<&str> = option_env!("PASSWORD");
const RECONNECT_DELAY_MS: u64 = 5000;
const RADIO_RETRY_DELAY_MS: u64 = 1000;

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
