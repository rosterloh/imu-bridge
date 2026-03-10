use core::fmt::{Result, Write};

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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BinaryEncodeError {
    BufferTooSmall,
}

impl SensorReading {
    pub fn format_into<const N: usize>(&self, msg: &mut heapless::String<N>) -> Result {
        write!(
            msg,
            "ts_us={} accel_mg={:4.2},{:4.2},{:4.2} gyro_mdps={:4.2},{:4.2},{:4.2} temp_c={:4.2}",
            self.timestamp_us,
            self.acceleration_mg[0],
            self.acceleration_mg[1],
            self.acceleration_mg[2],
            self.angular_rate_mdps[0],
            self.angular_rate_mdps[1],
            self.angular_rate_mdps[2],
            self.temperature_c
        )
    }

    pub fn encode_binary_into<const N: usize>(
        &self,
        payload: &mut heapless::Vec<u8, N>,
    ) -> core::result::Result<(), BinaryEncodeError> {
        payload.clear();
        push_u64(payload, self.timestamp_us)?;
        for value in self.acceleration_mg {
            push_f32(payload, value)?;
        }
        for value in self.angular_rate_mdps {
            push_f32(payload, value)?;
        }
        push_f32(payload, self.temperature_c)?;
        Ok(())
    }
}

fn push_u64<const N: usize>(
    payload: &mut heapless::Vec<u8, N>,
    value: u64,
) -> core::result::Result<(), BinaryEncodeError> {
    for byte in value.to_le_bytes() {
        payload
            .push(byte)
            .map_err(|_| BinaryEncodeError::BufferTooSmall)?;
    }
    Ok(())
}

fn push_f32<const N: usize>(
    payload: &mut heapless::Vec<u8, N>,
    value: f32,
) -> core::result::Result<(), BinaryEncodeError> {
    for byte in value.to_le_bytes() {
        payload
            .push(byte)
            .map_err(|_| BinaryEncodeError::BufferTooSmall)?;
    }
    Ok(())
}
