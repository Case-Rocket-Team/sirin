use crate::state::{EcefPos, State, Accel, Vel};
use byteorder::{ByteOrder, LittleEndian};
use uunit::WithUnits;
use zerocopy::{IntoBytes, transmute_mut, transmute};

pub trait SerializationSize {
    /// Number of bytes that this should be when serialized.
    fn serialization_size(self: &Self) -> usize;
}

pub trait Serialize: SerializationSize {
    /// Serialize `self`` into `buf`. Return the number of bytes written
    fn serialize(&self, buf: &mut [u8]) -> Result<(), SerializationError>;
}

pub trait Deserialize: SerializationSize {
    /// Deserialize from `buf` and return the deserialized struct and bytes read
    fn deserialize(buf: &[u8]) -> Result<Self, DeserializationError> where Self: Sized;
}

// Fill out as needed.
#[non_exhaustive]
#[derive(Debug, Clone)]
pub enum SerializationError {
    NotEnoughBytes
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeserializationError {
    NotEnoughBytes,
    InvalidInputData
}

#[derive(Debug, Clone)]
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
            Self::State(state) => 1 + state.serialization_size()
        }
    }
}

impl Serialize for LogData {
    fn serialize(&self, buf: &mut [u8]) -> Result<(), SerializationError> {
        match self {
            Self::Null => {
                if buf.len() >= 1 {
                    buf[0] = LogDataType::Null as u8;
                    Ok(())
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
    fn serialize(&self, buf: &mut [u8]) -> Result<(), SerializationError> {
        let size = self.serialization_size();
        if buf.len() >= size {
            LittleEndian::write_f64(&mut buf[0..], self.pos.x.value);
            LittleEndian::write_f64(&mut buf[8..], self.pos.y.value);
            LittleEndian::write_f64(&mut buf[16..], self.pos.z.value);
            
            LittleEndian::write_f64(&mut buf[24..], self.vel.x.value);
            LittleEndian::write_f64(&mut buf[32..], self.vel.y.value);
            LittleEndian::write_f64(&mut buf[40..], self.vel.z.value);
            
            LittleEndian::write_f64(&mut buf[48..], self.accel.x.value);
            LittleEndian::write_f64(&mut buf[56..], self.accel.y.value);
            LittleEndian::write_f64(&mut buf[64..], self.accel.z.value);
            
            LittleEndian::write_f64(&mut buf[72..], self.altitude.value);
            Ok(())
        } else {
            Err(SerializationError::NotEnoughBytes)
        }
    }
}

impl Deserialize for LogData {
    fn deserialize(buf: &[u8]) -> Result<Self, DeserializationError> {
        let Some(&disc) = buf.get(0) else {
            return Err(DeserializationError::NotEnoughBytes)
        };

        if disc == LogDataType::Null as u8 {
            Ok(LogData::Null)
        } else if disc == LogDataType::State as u8 {
            Ok(LogData::State(State::deserialize(&buf[1..])?))
        } else {
            Err(DeserializationError::InvalidInputData)
        }
    }
}

impl Deserialize for State {
    fn deserialize(buf: &[u8]) -> Result<Self, DeserializationError> {
        if buf.len() < 80 {
            return Err(DeserializationError::NotEnoughBytes)
        }

        Ok(State {
            pos: EcefPos {
                x: LittleEndian::read_f64(&buf[0..8]).with_units(),
                y: LittleEndian::read_f64(&buf[8..16]).with_units(),
                z: LittleEndian::read_f64(&buf[16..24]).with_units()
            },
            vel: Vel {
                x: LittleEndian::read_f64(&buf[24..32]).with_units(),
                y: LittleEndian::read_f64(&buf[32..40]).with_units(),
                z: LittleEndian::read_f64(&buf[40..48]).with_units(),
            },
            accel: Accel {
                x: LittleEndian::read_f64(&buf[48..56]).with_units(),
                y: LittleEndian::read_f64(&buf[56..64]).with_units(),
                z: LittleEndian::read_f64(&buf[64..72]).with_units()
            },
            altitude: LittleEndian::read_f64(&buf[72..80]).with_units()
        })
    }
}