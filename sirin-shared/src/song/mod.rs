
use core::mem::MaybeUninit;

use uunit::{Dimension, Quantity};

pub mod option;

pub trait SongSize {
    /// Number of bytes that this should be when serialized.
    fn song_size(self: &Self) -> usize;
}

pub trait ConstSongSize {
    const SONG_SIZE: usize;
}

pub trait ToSong: SongSize {
    /// Serialize `self`` into `buf`. Return the number of bytes written
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError>;
}

pub trait FromSong: SongSize {
    /// Deserialize from `buf` and return the deserialized struct and bytes read
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized;
}

pub trait SongDiscriminant {
    type Discriminant: Copy;

    fn song_discriminant(&self) -> Self::Discriminant;
}

// Use some tricks to automatically implement ConstSongSize.
pub(crate) trait ConstSongSizeImpl {
    const SIZE: usize;
}

impl ConstSongSizeImpl for () {
    const SIZE: usize = 0;
}

impl <T: ConstSongSizeImpl> ConstSongSizeImpl for (T,) {
    const SIZE: usize = T::SIZE;
}

impl <A: ConstSongSizeImpl, B: ConstSongSizeImpl> ConstSongSizeImpl for (A, B) {
    const SIZE: usize = A::SIZE + B::SIZE;
}

pub struct ConstSongSizeValue<const SIZE: usize>;

impl <const SIZE: usize> ConstSongSizeImpl for ConstSongSizeValue<SIZE> {
    const SIZE: usize = SIZE;
}

#[allow(private_bounds)]
pub struct MultiplyConstSongSizeImpl<A: ConstSongSizeImpl, B: ConstSongSizeImpl>(A, B);
impl <A: ConstSongSizeImpl, B: ConstSongSizeImpl> ConstSongSizeImpl for MultiplyConstSongSizeImpl<A, B> {
    const SIZE: usize = A::SIZE * B::SIZE;
}

pub struct ConstSongSizeImplFromConstSongSize<T>(T);
impl <T: ConstSongSize> ConstSongSizeImpl for ConstSongSizeImplFromConstSongSize<T> {
    const SIZE: usize = T::SONG_SIZE;
}

pub trait HasSongSize {
    type Size;
}

impl <T: HasSongSize> ConstSongSize for T
where T::Size: ConstSongSizeImpl {
    const SONG_SIZE: usize = T::Size::SIZE;
}
// Now inside the macro we can write an impl for HasSongSize and the above will do the rest.

// Fill out as needed.
#[non_exhaustive]
#[derive(Debug, Clone)]
pub enum ToSongError {
    BufferOverflow,
    NotImplemented,
    StringTooLong
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FromSongError {
    BufferOverflow,
    NotImplemented,
    InvalidPacketId,
    Utf8Error,
    InvalidValue
}

impl SongSize for &str {
    fn song_size(self: &Self) -> usize {
        2 + self.len()
    }
}

impl ToSong for &str {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
        if buf[2..(2 + self.len())].len() != self.len() {
            return Err(ToSongError::BufferOverflow)
        }
        self.len().to_song(buf)?;
        buf[2..(2 + self.len())].copy_from_slice(self.as_bytes());
        Ok(())
    }
}

// This is not implemented correctly
/*
impl <const N: usize> SongSize for heapless::String<N> {
    fn song_size(self: &Self) -> usize {
        N
    }
}

impl <const N: usize> ToSong for heapless::String<N> {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
        if buf.len() < N {
            return Err(ToSongError::BufferOverflow)
        }

        buf.copy_from_slice(self.as_bytes());
        Ok(())
    }
}

impl <const N: usize> FromSong for heapless::String<N> {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized {
        if buf.len() < N {
            return Err(FromSongError::BufferOverflow)
        }

        let vec = heapless::Vec::from_slice(&buf[0..N]).unwrap();
        Ok(Self::from_utf8(vec).map_err(|_| FromSongError::Utf8Error)?)
    }
}*/

// TODO -- How are we going to handle reading this out?
/*impl <'a> FromSong for &'a str {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized {
        let len = u16::from_song(buf)?;
        if len as usize + 2 > buf.len() {
            return Err(FromSongError::BufferOverflow)
        }
        let str = core::str::from_utf8(&buf[2..(len as usize)]).map_err(|e| FromSongError::Utf8Error)?;
        
        Ok(str)
    }
}*/

impl HasSongSize for bool {
    type Size = ConstSongSizeValue<1>;
}

impl SongSize for bool {
    fn song_size(self: &Self) -> usize {
        1
    }
}

impl ToSong for bool {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
        if buf.len() < 1 {
            return Err(ToSongError::BufferOverflow)
        }

        buf[0] = if *self { 1 } else { 0 };
        Ok(())
    }
}

impl FromSong for bool {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized {
        if buf.len() < 1 {
            return Err(FromSongError::BufferOverflow)
        }

        match buf[0] {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(FromSongError::InvalidValue)
        }
    }
}

impl <T: HasSongSize, D: Dimension> HasSongSize for Quantity<T, D> {
    type Size = T::Size;
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
    ($($ty: ty),*) => {
        $(
            impl HasSongSize for $ty {
                type Size = ConstSongSizeValue<{ core::mem::size_of::<$ty>() }>;
            }

            impl SongSize for $ty {
                #[inline]
                fn song_size(&self) -> usize {
                    core::mem::size_of::<$ty>()
                }
            }

            impl ToSong for $ty {
                fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
                    if buf.len() < core::mem::size_of::<$ty>() {
                        return Err(ToSongError::BufferOverflow);
                    }

                    buf[0..core::mem::size_of::<$ty>()].copy_from_slice(&self.to_le_bytes());
                    Ok(())
                }
            }

            impl FromSong for $ty {
                fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
                    if buf.len() < core::mem::size_of::<$ty>() {
                        return Err(FromSongError::BufferOverflow);
                    }
                    let mut arr = [0u8; core::mem::size_of::<$ty>()];
                    arr.copy_from_slice(&buf[0..core::mem::size_of::<$ty>()]);
                    Ok(<$ty>::from_le_bytes(arr))
                }
            }
        )*
    };
}

numeric_impl!(
    u8,
    u16,
    u32,
    u64,
    u128,
    usize,
    i8,
    i16,
    i32,
    i64,
    i128,
    isize,
    f32,
    f64
);

impl <T: HasSongSize, const N: usize> HasSongSize for [T; N]
where T::Size: ConstSongSizeImpl {
    type Size = MultiplyConstSongSizeImpl<T::Size, ConstSongSizeValue<N>>;
}

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

/*

pub struct ChunkedPacket<'a, const N: usize> {
    data: &'a [u8; N]
}

pub struct Chunker<'a, const N: usize>(pub &'a [u8]);

impl <'a, const N: usize> Iterator for Chunker<'a, N> {
    type Item = ChunkedPacket<'a, N>;

    fn next(&mut self) -> Option<Self::Item> {
        
    }
}*/

//pub static EVENT_CHANNEL: PubSubChannel<CriticalSectionRawMutex, Event, 100, 4, 4>;