#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::num::NonZeroU32;
use defmt::error;
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::Peripherals;
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::timer::timg::{MwdtStage, TimerGroup, TimerGroupInstance, Wdt};
use esp_radio::Controller;
use getrandom::register_custom_getrandom;
use panic_rtt_target as _;
use static_cell::StaticCell;

extern crate alloc;

use imu_bridge::app::tasks;
use imu_bridge::error::Error;
use imu_bridge::network::{connection, net_task};
use imu_bridge::{board, config};

esp_bootloader_esp_idf::esp_app_desc!();

static RNG: Mutex<CriticalSectionRawMutex, Option<Rng>> = Mutex::new(None);
static WIFI_RADIO: StaticCell<Controller> = StaticCell::new();
static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
static RTC: StaticCell<Rtc> = StaticCell::new();

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

    if let Err(e) = real_main(peripherals, spawner).await {
        error!("Init error: {:?}", e);
    }

    Timer::after(Duration::from_secs(5)).await;
    esp_hal::system::software_reset()
}

async fn real_main(peripherals: Peripherals, spawner: Spawner) -> Result<(), Error> {
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
    let TimerGroup { timer0, wdt, .. } = timg0;
    let rtc = RTC.init(Rtc::new(lpwr));
    esp_rtos::start(timer0);

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

    let config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    unsafe {
        RNG.lock_mut(|rng_opt| {
            *rng_opt = Some(rng);
        });
    }

    let resources = STACK_RESOURCES.init(StackResources::<3>::new());
    let (stack, runner) = embassy_net::new(interfaces.sta, config, resources, seed);

    spawner.spawn(connection(wifi_controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();
    spawner.spawn(tasks::zenoh_task(stack, rtc, led0)).unwrap();
    spawner
        .spawn(tasks::sensor_task(
            config::IMU_KIND,
            i2c_bus,
            spi_bus,
            spi_cs_accel,
            spi_cs_gyro,
            rtc,
        ))
        .unwrap();

    run_watchdog(wdt).await
}

async fn run_watchdog<T>(mut wdt: Wdt<T>) -> Result<(), Error>
where
    T: TimerGroupInstance + 'static,
{
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
