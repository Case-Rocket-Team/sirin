use core::mem::MaybeUninit;

use crate::state::{EcefPos, State, Accel, Vel};
use byteorder::{ByteOrder, LittleEndian};
use embassy_futures::join::join3;
use sirin_macros::{SongSize, ToSong, FromSong};
use uunit::{Dimension, Quantity, WithUnits};
use zerocopy::{IntoBytes, transmute_mut, transmute};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::PubSubChannel};

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
                    if buf.len() < $size {
                        return Err(FromSongError::BufferOverflow);
                    }
                    let mut arr = [0u8; $size];
                    arr.copy_from_slice(&buf[0..$size]);
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

impl <T: SongSize, const N: usize> SongSize for [T; N] {
    fn song_size(self: &Self) -> usize {
        let mut i = 0;
        for item in self {
            i += item.song_size();
        }
        i
    }
}

impl <T: SongSize + ToSong, const N: usize> ToSong for [T; N] {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
        let mut i = 0;
        for item in self {
            item.to_song(&mut buf[i..])?;
            i += item.song_size();
        }
        Ok(())
    }
}

impl <T: SongSize + FromSong, const N: usize> FromSong for [T; N] {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized {
        let mut i = 0;
        let mut arr = [const { MaybeUninit::uninit() }; N];

        for j in 0..N {
            let item = T::from_song(&buf[i..])?;
            i += item.song_size();
            arr[j] = MaybeUninit::new(item);
        }

        unsafe {
            // https://github.com/rust-lang/rust/issues/61956
            let ptr = &mut arr as *mut _ as *mut [T; N];
            let res = ptr.read();
            core::mem::forget(arr);
            Ok(res)
        }
    }
}

pub struct OutPacketChannel<const N: usize, const L: usize> {
    
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[song(discriminant(OutPacketType = u8))]
pub enum OutPacket {
    Null,
    State(State),
    FlashSectorDump(FlashPageDump)
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]

pub struct FlashPageDump {
    addr: u32,
    data: [u8; 256]
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[song(discriminant(InPacketType = u8))]
pub enum InPacket {
    Null,
    DumpFlash
}

//pub static EVENT_CHANNEL: PubSubChannel<CriticalSectionRawMutex, Event, 100, 4, 4>;