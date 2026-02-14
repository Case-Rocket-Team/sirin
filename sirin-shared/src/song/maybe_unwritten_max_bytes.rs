use core::ops::{Deref, DerefMut};

use super::*;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
#[repr(transparent)]
pub struct MaybeUnwrittenMaxBytes<T: ConstSongSize>(pub Option<T>);

impl <T: ConstSongSize> Deref for MaybeUnwrittenMaxBytes<T> {
    type Target = Option<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl <T: ConstSongSize> DerefMut for MaybeUnwrittenMaxBytes<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl <T: ConstSongSize> HasSongSize for MaybeUnwrittenMaxBytes<T> {
    type Size = (ConstSongSizeImplFromConstSongSize<T>, ConstSongSizeValue<1>);
}

impl <T: ConstSongSize> SongSize for MaybeUnwrittenMaxBytes<T> {
    fn song_size(&self) -> usize {
        T::SONG_SIZE + 1
    }
}

impl <T: ConstSongSize + FromSong> FromSong for MaybeUnwrittenMaxBytes<T> {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
        if buf.len() < Self::SONG_SIZE {
            return Err(FromSongError::BufferOverflow)
        }

        match buf[0] {
            0xFF => Ok(Self(None)),
            0x18 => Ok(Self(Some(T::from_song(&buf[1..])?))),
            _ => Err(FromSongError::InvalidValue)
        }
    }
    
}

impl <T: ConstSongSize + ToSong> ToSong for MaybeUnwrittenMaxBytes<T> {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
        if buf.len() < T::SONG_SIZE {
            return Err(ToSongError::BufferOverflow)
        }

        match &self.0 {
            Some(value) => {
                buf[0] = 0x18;
                value.to_song(&mut buf[1..])?;
            },
            None => buf[0..T::SONG_SIZE + 1].fill(0xFF),
        }

        Ok(())
    }
}