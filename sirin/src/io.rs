use crate::state::{EcefPos, State, Accel, Vel};
use byteorder::{ByteOrder, LittleEndian};
use sirin_macros::Serialize;
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
    OutOfSpace,
    NotImplemented
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeserializationError {
    OutOfSpace,
    NotImplemented,
    InvalidPacketId
}

macro_rules! numeric_impl {
    ($($ty: ty => $size:expr),*) => {
        $(
            impl SerializationSize for $ty {
                fn serialization_size(&self) -> usize {
                    $size
                }
            }

            impl Serialize for $ty {
                fn serialize(&self, buf: &mut [u8]) -> Result<(), SerializationError> {
                    if buf.len() < self.serialization_size() {
                        return Err(SerializationError::OutOfSpace);
                    }

                    buf.copy_from_slice(&self.to_le_bytes());
                    Ok(())
                }
            }
        )*
    };
}

numeric_impl!(
    u8 => 1,
    u16 => 2,
    u32 => 4,
    u64 => 8,
    u128 => 16,
    i8 => 1,
    i16 => 2,
    i32 => 4,
    i64 => 8,
    i128 => 16,
    f32 => 4,
    f64 => 8
);

#[derive(Debug, Clone, Serialize)]
pub enum OutPacket {
    Null,
    State(State)
}

#[repr(u8)]
pub enum OutPacketType {
    Null = 0x00,
    State = 0x01
}

impl SerializationSize for OutPacket {
    fn serialization_size(self: &Self) -> usize {
        match self {
            Self::Null => 1,
            Self::State(state) => 1 + state.serialization_size()
        }
    }
}

impl Deserialize for OutPacket {
    fn deserialize(buf: &[u8]) -> Result<Self, DeserializationError> {
        let Some(&disc) = buf.get(0) else {
            return Err(DeserializationError::OutOfSpace)
        };

        if disc == OutPacketType::Null as u8 {
            Ok(OutPacket::Null)
        } else if disc == OutPacketType::State as u8 {
            Ok(OutPacket::State(State::deserialize(&buf[1..])?))
        } else {
            Err(DeserializationError::InvalidPacketId)
        }
    }
}

impl Deserialize for State {
    fn deserialize(buf: &[u8]) -> Result<Self, DeserializationError> {
        if buf.len() < 80 {
            return Err(DeserializationError::OutOfSpace)
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