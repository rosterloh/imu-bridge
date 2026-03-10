use defmt::{error, info, warn};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::gpio::Output;
use esp_hal::rtc_cntl::Rtc;
use zenoh_nostd::session::Endpoint;
use zenoh_nostd::session::{Session, zenoh};

use crate::app::{CONNECTION_STATE, SENSOR_CHANNEL};
use crate::board;
use crate::config;
use crate::error::Error;
use crate::network::{ZenohConfig, sync_rtc_from_ntp, zenoh_connect_static};
use crate::sensor;
use crate::telemetry;

#[embassy_executor::task]
pub async fn sensor_task(
    imu_kind: config::ImuKind,
    i2c_bus: &'static board::SharedI2c,
    spi_bus: &'static board::SharedSpi,
    spi_cs_accel: Output<'static>,
    spi_cs_gyro: Option<Output<'static>>,
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
        let connected = CONNECTION_STATE.zenoh_ready();
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
pub async fn zenoh_task(
    stack: embassy_net::Stack<'static>,
    rtc: &'static Rtc<'static>,
    mut led0: Output<'static>,
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
        CONNECTION_STATE.set_zenoh_ready(true);

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

        CONNECTION_STATE.set_zenoh_ready(false);
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

        let val = match with_timeout(Duration::from_millis(500), SENSOR_CHANNEL.receive()).await {
            Ok(val) => val,
            Err(_) => continue,
        };

        let mut msg = heapless::String::<{ config::SENSOR_MESSAGE_CAPACITY }>::new();
        telemetry::text::encode_reading(&val, &mut msg)?;
        info!("Received sensor reading: {}", msg);

        session
            .put(keyexpr, msg.as_bytes())
            .finish()
            .await
            .map_err(|e| Error::ZError(e.into()))?;
    }
}
