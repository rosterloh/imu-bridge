#[derive(Clone, Copy, Debug)]
pub struct SensorReading {
    pub timestamp_us: u64,
    pub acceleration_mg: [f32; 3],
    pub angular_rate_mdps: [f32; 3],
    pub temperature_c: f32,
}

impl defmt::Format for SensorReading {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "SensorReading {{ ts_us: {=u64}, accel_mg: [{=f32}, {=f32}, {=f32}], gyro_mdps: [{=f32}, {=f32}, {=f32}], temp_c: {=f32} }}",
            self.timestamp_us,
            self.acceleration_mg[0],
            self.acceleration_mg[1],
            self.acceleration_mg[2],
            self.angular_rate_mdps[0],
            self.angular_rate_mdps[1],
            self.angular_rate_mdps[2],
            self.temperature_c
        );
    }
}
