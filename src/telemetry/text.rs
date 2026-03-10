use core::fmt::{Result, Write};

use crate::domain::sensor_reading::SensorReading;

pub fn encode_reading<const N: usize>(
    reading: &SensorReading,
    msg: &mut heapless::String<N>,
) -> Result {
    write!(
        msg,
        "ts_us={} accel_mg={:4.2},{:4.2},{:4.2} gyro_mdps={:4.2},{:4.2},{:4.2} temp_c={:4.2}",
        reading.timestamp_us,
        reading.acceleration_mg[0],
        reading.acceleration_mg[1],
        reading.acceleration_mg[2],
        reading.angular_rate_mdps[0],
        reading.angular_rate_mdps[1],
        reading.angular_rate_mdps[2],
        reading.temperature_c
    )
}
