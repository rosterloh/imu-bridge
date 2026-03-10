pub mod state;
pub mod tasks;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

use crate::domain::sensor_reading::SensorReading;
use state::ConnectionState;

pub type SensorChannel = Channel<CriticalSectionRawMutex, SensorReading, 4>;

pub static CONNECTION_STATE: ConnectionState = ConnectionState::new();
pub static SENSOR_CHANNEL: SensorChannel = Channel::new();
