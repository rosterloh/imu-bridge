#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::num::NonZeroU32;
// use core::sync::atomic::{AtomicBool, Ordering};
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    channel::Channel,
    signal::Signal,
};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::Peripherals;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::{MwdtStage, TimerGroup};
use esp_radio::Controller;
use getrandom::register_custom_getrandom;
use panic_rtt_target as _;
use static_cell::StaticCell;
use zenoh_nostd::session::Endpoint;
use zenoh_nostd::session::{Session, zenoh};

extern crate alloc;

use imu_bridge::domain::sensor_reading::SensorReading;
use imu_bridge::error::Error;
use imu_bridge::network::{ZenohConfig, connection, net_task, zenoh_connect_static};
use imu_bridge::sensor;
use imu_bridge::{board, config};

esp_bootloader_esp_idf::esp_app_desc!();

static RNG: Mutex<CriticalSectionRawMutex, Option<Rng>> = Mutex::new(None);
static WIFI_RADIO: StaticCell<Controller> = StaticCell::new();
static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorReading, 4> = Channel::new();
static ZENOH_CONNECTED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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
        led0: _led0,
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
    spawner
        .spawn(sensor_task(
            config::IMU_KIND,
            i2c_bus,
            spi_bus,
            spi_cs_accel,
            spi_cs_gyro,
        ))
        .ok();

    info!("Waiting for link...");
    loop {
        if stack.is_link_up() {
            info!("link up!");
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Configure zenoh");
    let endpoint = match Endpoint::try_from(config::zenoh_router_endpoint()) {
        Ok(ep) => ep,
        Err(e) => {
            error!("Invalid endpoint: {:?}", e);
            return Err(Error::ZError(e.into()));
        }
    };
    let session = zenoh_connect_static(stack, endpoint).await;
    if session.is_none() {
        warn!("Zenoh connect failed");
    } else {
        ZENOH_CONNECTED.signal(());
    }

    info!("Start watchdog");
    let mut wdt = timg0.wdt;
    wdt.set_timeout(
        MwdtStage::Stage0,
        esp_hal::time::Duration::from_secs(6 * 60),
    );
    wdt.enable();

    loop {
        // Timer::after(Duration::from_secs(1)).await;
        wdt.feed();
        if let Err(e) = main_loop(session).await {
            error!("main loop error: {:?}", e);
        }
    }
}

#[embassy_executor::task]
async fn sensor_task(
    imu_kind: config::ImuKind,
    i2c_bus: &'static board::SharedI2c,
    spi_bus: &'static board::SharedSpi,
    spi_cs_accel: esp_hal::gpio::Output<'static>,
    spi_cs_gyro: Option<esp_hal::gpio::Output<'static>>,
) {
    let mut sensor =
        match sensor::init_selected(imu_kind, i2c_bus, spi_bus, spi_cs_accel, spi_cs_gyro).await {
            Ok(sensor) => sensor,
            Err(error) => {
                sensor::log_error(error);
                return;
            }
        };

    info!("IMU initialised, waiting for zenoh session");
    ZENOH_CONNECTED.wait().await;
    info!("Zenoh session connected, start publishing sensor readings");

    loop {
        match sensor::read_ready(&mut sensor).await {
            Ok(Some(reading)) => {
                SENSOR_CHANNEL.send(reading).await;
            }
            Ok(None) => {}
            Err(error) => {
                sensor::log_error(error);
            }
        }

        Timer::after_millis(config::SENSOR_POLL_INTERVAL_MS).await;
    }
}

async fn main_loop(session: Option<&'static Session<'static, ZenohConfig>>) -> Result<(), Error> {
    let val = SENSOR_CHANNEL.receive().await;
    let Some(session) = session else {
        warn!("Skipping publish because zenoh session is not connected");
        return Ok(());
    };

    let mut msg = heapless::String::<{ config::SENSOR_MESSAGE_CAPACITY }>::new();
    val.format_into(&mut msg)?;

    let keyexpr = zenoh::keyexpr::new(config::ZENOH_KEYEXPR).map_err(|_| Error::Write)?;
    session
        .put(keyexpr, msg.as_bytes())
        .finish()
        .await
        .map_err(|e| Error::ZError(e.into()))?;

    Ok(())
}
