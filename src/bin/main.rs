#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::num::NonZeroU32;
use core::sync::atomic::{AtomicBool, Ordering};
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::StackResources;
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    channel::Channel,
};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::Peripherals;
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::timer::timg::{MwdtStage, TimerGroup};
use esp_radio::Controller;
use getrandom::register_custom_getrandom;
use panic_rtt_target as _;
use static_cell::StaticCell;
use zenoh_nostd::session::Endpoint;
use zenoh_nostd::session::{Session, zenoh};

extern crate alloc;

use imu_bridge::error::Error;
use imu_bridge::network::{
    ZenohConfig, connection, net_task, sync_rtc_from_ntp, zenoh_connect_static,
};
use imu_bridge::sensor;
use imu_bridge::{board, config};

esp_bootloader_esp_idf::esp_app_desc!();

static RNG: Mutex<CriticalSectionRawMutex, Option<Rng>> = Mutex::new(None);
static WIFI_RADIO: StaticCell<Controller> = StaticCell::new();
static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
static RTC: StaticCell<Rtc> = StaticCell::new();
static SENSOR_CHANNEL: sensor::SensorChannel = Channel::new();
static ZENOH_CONNECTED: AtomicBool = AtomicBool::new(false);

register_custom_getrandom!(getrandom_custom);
const MY_CUSTOM_ERROR_CODE: u32 = getrandom::Error::CUSTOM_START + 42;
pub fn getrandom_custom(bytes: &mut [u8]) -> Result<(), getrandom::Error> {
    unsafe {
        RNG.lock_mut(|rng_opt| {
            let code = NonZeroU32::new(MY_CUSTOM_ERROR_CODE).unwrap();
            let rng = rng_opt.as_mut().ok_or(getrandom::Error::from(code))?;
            rng.read(bytes);
            Ok(())
        })
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    // esp_alloc::heap_allocator!(size: 36 * 1024);

    if let Err(e) = real_main(peripherals, spawner).await {
        error!("Init error: {:?}", e);
    }

    Timer::after(Duration::from_secs(5)).await;
    esp_hal::system::software_reset()
}

async fn real_main<'a>(peripherals: Peripherals, spawner: Spawner) -> Result<(), Error> {
    let Peripherals {
        TIMG0: timg0_peripheral,
        WIFI: wifi,
        I2C0: i2c0,
        LPWR: lpwr,
        SPI2: spi2,
        GPIO11: gpio11,
        GPIO12: gpio12,
        GPIO13: gpio13,
        GPIO21: gpio21,
        GPIO38: gpio38,
        GPIO45: gpio45,
        GPIO46: gpio46,
        GPIO47: gpio47,
        GPIO48: gpio48,
        GPIO0: gpio0,
        ..
    } = peripherals;

    let timg0 = TimerGroup::new(timg0_peripheral);
    let rtc = RTC.init(Rtc::new(lpwr));
    // let sw_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0); // sw_interrupt.software_interrupt0

    let esp_radio_ctrl = &*WIFI_RADIO.init(esp_radio::init()?);
    let (wifi_controller, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, wifi, Default::default())?;

    let board::BoardContext {
        i2c_bus,
        spi_bus,
        spi_cs_accel,
        spi_cs_gyro,
        led0,
    } = board::init(
        i2c0, spi2, gpio11, gpio12, gpio21, gpio13, gpio38, gpio47, gpio48, gpio46, gpio0, gpio45,
    );

    let wifi_interface = interfaces.sta;

    let config = embassy_net::Config::dhcpv4(Default::default());
    // TODO
    // if let Some(hostname) = option_env!("DEVICE") {
    // config.ipv4.hostname = Some(hostname);
    // }

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    unsafe {
        RNG.lock_mut(|rng_opt| {
            *rng_opt = Some(rng);
        });
    }

    info!("Configure network stack");
    let resources = STACK_RESOURCES.init(StackResources::<3>::new());
    let (stack, runner) = embassy_net::new(wifi_interface, config, resources, seed);

    spawner.spawn(connection(wifi_controller)).ok();
    spawner.spawn(net_task(runner)).ok();
    spawner.spawn(zenoh_task(stack, rtc, led0)).ok();
    spawner
        .spawn(sensor_task(
            config::IMU_KIND,
            i2c_bus,
            spi_bus,
            spi_cs_accel,
            spi_cs_gyro,
            rtc,
        ))
        .ok();

    info!("Start watchdog");
    let mut wdt = timg0.wdt;
    wdt.set_timeout(
        MwdtStage::Stage0,
        esp_hal::time::Duration::from_secs(6 * 60),
    );
    wdt.enable();

    loop {
        wdt.feed();
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn sensor_task(
    imu_kind: config::ImuKind,
    i2c_bus: &'static board::SharedI2c,
    spi_bus: &'static board::SharedSpi,
    spi_cs_accel: esp_hal::gpio::Output<'static>,
    spi_cs_gyro: Option<esp_hal::gpio::Output<'static>>,
    rtc: &'static Rtc<'static>,
) {
    let mut sensor =
        match sensor::init_selected(imu_kind, i2c_bus, spi_bus, spi_cs_accel, spi_cs_gyro).await {
            Ok(sensor) => sensor,
            Err(error) => {
                sensor::log_error(error);
                panic!("sensor init failed: {:?}", error);
            }
        };

    info!("IMU initialised, waiting for zenoh session");
    let mut was_connected = false;

    loop {
        let connected = ZENOH_CONNECTED.load(Ordering::Acquire);
        if !connected {
            if was_connected {
                info!("Zenoh disconnected, pause IMU sampling");
                was_connected = false;
            }
            Timer::after_millis(250).await;
            continue;
        }

        if !was_connected {
            info!("Zenoh session connected, start publishing sensor readings");
            was_connected = true;
        }

        let timestamp_us = rtc.current_time_us();
        match sensor::read_ready(&mut sensor, timestamp_us).await {
            Ok(reading) => {
                SENSOR_CHANNEL.send(reading).await;
            }
            Err(error) => {
                sensor::log_error(error);
            }
        }

        Timer::after_millis(config::SENSOR_POLL_INTERVAL_MS).await;
    }
}

#[embassy_executor::task]
async fn zenoh_task(
    stack: embassy_net::Stack<'static>,
    rtc: &'static Rtc<'static>,
    mut led0: esp_hal::gpio::Output<'static>,
) {
    let mut rtc_synced = false;

    loop {
        wait_for_network_ready(stack, rtc, &mut rtc_synced).await;

        let endpoint = match Endpoint::try_from(config::zenoh_router_endpoint()) {
            Ok(ep) => ep,
            Err(e) => {
                error!("Invalid endpoint: {:?}", e);
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        };

        info!("Connecting zenoh...");
        let session = match zenoh_connect_static(stack, endpoint).await {
            Some(session) => session,
            None => {
                warn!("Zenoh connect failed");
                Timer::after(Duration::from_secs(2)).await;
                continue;
            }
        };

        info!("Zenoh connected");
        led0.set_low();
        ZENOH_CONNECTED.store(true, Ordering::Release);

        let reconnect_result = select(session.run(), publish_sensor_loop(session, stack)).await;

        match reconnect_result {
            Either::First(session_result) => {
                if let Err(e) = session_result {
                    warn!("Zenoh session closed: {:?}", e);
                } else {
                    warn!("Zenoh session ended");
                }
            }
            Either::Second(loop_result) => {
                if let Err(e) = loop_result {
                    warn!("Zenoh publish loop stopped: {:?}", e);
                }
            }
        }

        ZENOH_CONNECTED.store(false, Ordering::Release);
        led0.set_high();
        warn!("Resetting to recover Wi-Fi/zenoh connection");
        Timer::after(Duration::from_millis(250)).await;
        esp_hal::system::software_reset();
    }
}

async fn wait_for_network_ready(
    stack: embassy_net::Stack<'static>,
    rtc: &'static Rtc<'static>,
    rtc_synced: &mut bool,
) {
    if !stack.is_link_up() {
        info!("Waiting for link...");
    }
    while !stack.is_link_up() {
        Timer::after(Duration::from_millis(500)).await;
    }
    info!("link up!");

    if stack.config_v4().is_none() {
        info!("Waiting to get IP address...");
    }
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    if !*rtc_synced {
        sync_rtc_from_ntp(stack, rtc).await;
        *rtc_synced = true;
    }
}

async fn publish_sensor_loop(
    session: &Session<'_, ZenohConfig>,
    stack: embassy_net::Stack<'static>,
) -> Result<(), Error> {
    let keyexpr = zenoh::keyexpr::new(config::ZENOH_KEYEXPR).map_err(|_| Error::Write)?;

    loop {
        if !stack.is_link_up() || stack.config_v4().is_none() {
            warn!("Network lost, reconnecting zenoh");
            return Ok(());
        }

        let val = match embassy_time::with_timeout(
            Duration::from_millis(500),
            SENSOR_CHANNEL.receive(),
        )
        .await
        {
            Ok(val) => val,
            Err(_) => continue,
        };

        let mut msg = heapless::String::<{ config::SENSOR_MESSAGE_CAPACITY }>::new();
        val.format_into(&mut msg)?;
        info!("Received sensor reading: {}", msg);

        session
            .put(keyexpr, msg.as_bytes())
            .finish()
            .await
            .map_err(|e| Error::ZError(e.into()))?;
    }
}
