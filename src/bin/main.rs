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
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex}; // signal::Signal
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::Peripherals;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::Controller;
use getrandom::register_custom_getrandom;
use panic_rtt_target as _;
use static_cell::StaticCell;
use zenoh_nostd::session::Endpoint;

extern crate alloc;

use imu_bridge::error::Error;
use imu_bridge::network::{connection, net_task, zenoh_connect_static};

esp_bootloader_esp_idf::esp_app_desc!();

const ENDPOINT: Option<&str> = option_env!("ENDPOINT");

static RNG: Mutex<CriticalSectionRawMutex, Option<Rng>> = Mutex::new(None);
static WIFI_RADIO: StaticCell<Controller> = StaticCell::new();
static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

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
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    // let sw_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0); // sw_interrupt.software_interrupt0

    let _led_red = Output::new(peripherals.GPIO46, Level::High, OutputConfig::default());
    let mut green_led = Output::new(peripherals.GPIO0, Level::High, OutputConfig::default());
    let _led_blue = Output::new(peripherals.GPIO45, Level::High, OutputConfig::default());

    let esp_radio_ctrl = &*WIFI_RADIO.init(esp_radio::init()?);
    let (wifi_controller, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, peripherals.WIFI, Default::default())?;

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

    // static LED_CTRL: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
    // let led_ctrl_signal = &*LED_CTRL.init(Signal::new());
    // let _ = spawner.spawn(enable_disable_led(led_ctrl_signal));

    info!("Configure network stack");
    let resources = STACK_RESOURCES.init(StackResources::<3>::new());
    let (stack, runner) = embassy_net::new(wifi_interface, config, resources, seed);

    spawner.spawn(connection(wifi_controller)).ok();
    spawner.spawn(net_task(runner)).ok();

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

    green_led.set_low();

    info!("Configure zenoh");
    let endpoint = match Endpoint::try_from(ENDPOINT.unwrap_or("tcp/127.0.0.1:7447")) {
        Ok(ep) => ep,
        Err(e) => {
            error!("Invalid endpoint: {:?}", e);
            return Err(Error::ZError(e.into()));
        }
    };
    let session = zenoh_connect_static(stack, endpoint).await;
    if session.is_none() {
        warn!("Zenoh connect failed");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
