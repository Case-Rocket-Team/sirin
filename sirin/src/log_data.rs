use crate::state::{EcefPos, State};
use byteorder::{ByteOrder, LittleEndian};
use uunit::WithUnits;
use zerocopy::{IntoBytes, transmute_mut, transmute};

pub trait SerializationSize {
    /// Number of bytes that this should be when serialized.
    fn serialization_size(self: &Self) -> usize;
}

pub trait Serialize: SerializationSize {
    /// Serialize `self`` into `buf`. Return the number of bytes written
    fn serialize(&self, buf: &mut [u8]) -> Result<usize, SerializationError>;
}

pub trait Deserialize: SerializationSize {
    /// Deserialize from `buf` and return the deserialized struct.
    fn deserialize(buf: &[u8]) -> Result<Self, DeserializationError> where Self: Sized;
}

// Fill out as needed.
pub enum SerializationError {
    NotEnoughBytes
}

pub enum DeserializationError {
    NotEnoughBytes,
    InvalidState
}

pub enum LogData {
    Null,
    State(State)
}

#[repr(u8)]
pub enum LogDataType {
    Null = 0x00,
    State = 0x01
}

impl SerializationSize for LogData {
    fn serialization_size(self: &Self) -> usize {
        match self {
            Self::Null => 1,
            Self::State(state) => state.serialization_size()
        }
    }
}

impl Serialize for LogData {
    fn serialize(&self, buf: &mut [u8]) -> Result<usize, SerializationError> {
        match self {
            Self::Null => {
                if buf.len() >= 1 {
                    buf[0] = LogDataType::Null as u8;
                    Ok(1)
                } else {
                    Err(SerializationError::NotEnoughBytes)
                }
            },
            Self::State(state) => {
                let len = state.serialization_size() + 1;
                if buf.len() >= len {
                    buf[0] = LogDataType::State as u8;
                    state.serialize(&mut buf[1..])
                } else {
                    Err(SerializationError::NotEnoughBytes)
                }
            }
        }
    }
}

impl SerializationSize for State {
    /// This should be 80 
    fn serialization_size(self: &Self) -> usize {
        80
    }
}

impl Serialize for State {
    fn serialize(&self, buf: &mut [u8]) -> Result<usize, SerializationError> {
        let size = self.serialization_size();
        if buf.len() >= size {
            LittleEndian::write_f64(&mut buf[0..], self.pos.x.value);
            
            buf[8 ..].copy_from_slice(&self.pos.y.value.to_le_bytes());
            buf[16..].copy_from_slice(&self.pos.z.value.to_le_bytes());

            buf[24..].copy_from_slice(&self.vel.x.value.to_le_bytes());
            buf[32..].copy_from_slice(&self.vel.y.value.to_le_bytes());
            buf[40..].copy_from_slice(&self.vel.z.value.to_le_bytes());

            buf[48..].copy_from_slice(&self.accel.x.value.to_le_bytes());
            buf[56..].copy_from_slice(&self.accel.y.value.to_le_bytes());
            buf[64..].copy_from_slice(&self.accel.z.value.to_le_bytes());

            buf[72..].copy_from_slice(&self.altitude.value.to_le_bytes());

            Ok(size)
        } else {
            Err(SerializationError::NotEnoughBytes)
        }
    }
}

impl Deserialize for LogData {
    fn deserialize(buf: &[u8]) -> Result<Self, DeserializationError> {
        if buf[0] == LogDataType::Null as u8{
            Ok(LogData::Null)
        } else if buf[0] == LogDataType::State as u8 {
            Ok(LogData::State(State::deserialize(&buf[1..])?))
        } else {
            Err(DeserializationError::InvalidState)
        }
    }
}

impl Deserialize for State {
    fn deserialize(buf: &[u8]) -> Result<Self, DeserializationError> {
        Ok(State {
            pos: EcefPos {
                x: LittleEndian::read_f64(&buf[0..8]).with_units(),
            }
        })
    }
}