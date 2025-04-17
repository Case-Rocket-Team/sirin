use crate::state::{EcefPos, State, Accel, Vel};
use byteorder::{ByteOrder, LittleEndian};
use sirin_macros::{SongSize, ToSong};
use uunit::{Dimension, Quantity, WithUnits};
use zerocopy::{IntoBytes, transmute_mut, transmute};

pub trait SongSize {
    /// Number of bytes that this should be when serialized.
    fn song_size(self: &Self) -> usize;
}

pub trait ToSong: SongSize {
    /// Serialize `self`` into `buf`. Return the number of bytes written
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError>;
}

pub trait FromSong: SongSize {
    /// Deserialize from `buf` and return the deserialized struct and bytes read
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized;
}

pub trait Song: ToSong + FromSong {}

// Fill out as needed.
#[non_exhaustive]
#[derive(Debug, Clone)]
pub enum ToSongError {
    BufferOverflow,
    NotImplemented
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FromSongError {
    BufferOverflow,
    NotImplemented,
    InvalidPacketId
}

impl <T: SongSize, D: Dimension> SongSize for Quantity<T, D> {
    fn song_size(self: &Self) -> usize {
        self.value.song_size()
    }
}

impl <T: ToSong, D: Dimension> ToSong for Quantity<T, D> {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
        self.value.to_song(buf)
    }
}

impl <T: FromSong, D: Dimension> FromSong for Quantity<T, D> {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized {
        Ok(Self::new(<T as FromSong>::from_song(buf)?))
    }
}

macro_rules! numeric_impl {
    ($($ty: ty => $size:expr),*) => {
        $(
            impl SongSize for $ty {
                fn song_size(&self) -> usize {
                    $size
                }
            }

            impl ToSong for $ty {
                fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
                    if buf.len() < self.song_size() {
                        return Err(ToSongError::BufferOverflow);
                    }

                    buf.copy_from_slice(&self.to_le_bytes());
                    Ok(())
                }
            }

            impl FromSong for $ty {
                fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
                    let mut arr = [0u8; $size];
                    arr.copy_from_slice(buf);
                    Ok(<$ty>::from_le_bytes(arr))
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

#[derive(Debug, Clone, SongSize, ToSong)]
#[song(discriminant(OutPacketType = u8))]
pub enum OutPacket {
    Null,
    State(State)
}

impl FromSong for OutPacket {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
        let Some(&disc) = buf.get(0) else {
            return Err(FromSongError::BufferOverflow)
        };

        if disc == OutPacketType::Null as u8 {
            Ok(OutPacket::Null)
        } else if disc == OutPacketType::State as u8 {
            Ok(OutPacket::State(State::from_song(&buf[1..])?))
        } else {
            Err(FromSongError::InvalidPacketId)
        }
    }
}

impl FromSong for State {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
        if buf.len() < 80 {
            return Err(FromSongError::BufferOverflow)
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