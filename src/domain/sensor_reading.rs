use core::fmt::{Result, Write};

#[derive(Clone, Copy, Debug)]
pub enum SensorReading {
    Temperature(f32),
    Acceleration([f32; 3]),
    AngularRate([f32; 3]),
}

impl defmt::Format for SensorReading {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            SensorReading::Temperature(temp) => defmt::write!(fmt, "Temperature: {=f32} C", temp),
            SensorReading::Acceleration(accel) => defmt::write!(
                fmt,
                "Acceleration: [{=f32}, {=f32}, {=f32}] mg",
                accel[0],
                accel[1],
                accel[2]
            ),
            SensorReading::AngularRate(gyro) => defmt::write!(
                fmt,
                "Angular Rate: [{=f32}, {=f32}, {=f32}] mdps",
                gyro[0],
                gyro[1],
                gyro[2]
            ),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BinaryEncodeError {
    BufferTooSmall,
}

impl SensorReading {
    pub fn format_into<const N: usize>(&self, msg: &mut heapless::String<N>) -> Result {
        match self {
            SensorReading::Temperature(temp) => write!(msg, "Temperature: {:6.2} C", temp),
            SensorReading::Acceleration(accel) => write!(
                msg,
                "Acceleration: {:4.2}, {:4.2}, {:4.2} mg",
                accel[0], accel[1], accel[2]
            ),
            SensorReading::AngularRate(gyro) => write!(
                msg,
                "Angular Rate: {:4.2}, {:4.2}, {:4.2} mdps",
                gyro[0], gyro[1], gyro[2]
            ),
        }
    }
    pub fn encode_binary_into<const N: usize>(
        &self,
        payload: &mut heapless::Vec<u8, N>,
    ) -> core::result::Result<(), BinaryEncodeError> {
        payload.clear();
        match self {
            SensorReading::Temperature(temp) => {
                payload
                    .push(0)
                    .map_err(|_| BinaryEncodeError::BufferTooSmall)?;
                push_f32(payload, *temp)?;
            }
            SensorReading::Acceleration(accel) => {
                payload
                    .push(1)
                    .map_err(|_| BinaryEncodeError::BufferTooSmall)?;
                for value in accel {
                    push_f32(payload, *value)?;
                }
            }
            SensorReading::AngularRate(gyro) => {
                payload
                    .push(2)
                    .map_err(|_| BinaryEncodeError::BufferTooSmall)?;
                for value in gyro {
                    push_f32(payload, *value)?;
                }
            }
        }
        Ok(())
    }
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
