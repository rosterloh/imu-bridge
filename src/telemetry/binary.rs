use crate::domain::sensor_reading::SensorReading;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EncodeError {
    BufferTooSmall,
}

pub fn encode_reading_into<const N: usize>(
    reading: &SensorReading,
    payload: &mut heapless::Vec<u8, N>,
) -> core::result::Result<(), EncodeError> {
    payload.clear();
    push_u64(payload, reading.timestamp_us)?;
    for value in reading.acceleration_mg {
        push_f32(payload, value)?;
    }
    for value in reading.angular_rate_mdps {
        push_f32(payload, value)?;
    }
    push_f32(payload, reading.temperature_c)?;
    Ok(())
}

fn push_u64<const N: usize>(
    payload: &mut heapless::Vec<u8, N>,
    value: u64,
) -> core::result::Result<(), EncodeError> {
    for byte in value.to_le_bytes() {
        payload
            .push(byte)
            .map_err(|_| EncodeError::BufferTooSmall)?;
    }
    Ok(())
}

fn push_f32<const N: usize>(
    payload: &mut heapless::Vec<u8, N>,
    value: f32,
) -> core::result::Result<(), EncodeError> {
    for byte in value.to_le_bytes() {
        payload
            .push(byte)
            .map_err(|_| EncodeError::BufferTooSmall)?;
    }
    Ok(())
}
