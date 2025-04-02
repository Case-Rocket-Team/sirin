use crate::state::State;
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
        size_of::<State>()
    }
}

impl Serialize for State {
    fn serialize(&self, buf: &mut [u8]) -> Result<usize, SerializationError> {
        let size = self.serialization_size();
        if buf.len() >= size {
            let mut temp: [u8; 80];
            temp = transmute!(self);
            buf[0..80].copy_from_slice(&temp);
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
        } else if buf[1] == LogDataType::State as u8 {
            let data = 
            Ok(LogData::State((state)))
        } else {
            Err(DeserializationError::InvalidState)
        }
    }
}

impl Deserialize for State {
    fn deserialize(buf: &[u8]) -> Result<Self, DeserializationError> {
        todo!()
    }
}