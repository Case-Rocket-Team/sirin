//! For various values, if we use 0xFFFFFF... to represent a None state of an option,
//! this allows us to go back and write the Some value to the flash (since NOR flash
//! can only turn 1's to 0's). This of course means that Some(0xFFFF...) becomes
//! unrepresentable, so we cannot implement this generally for all options.
//! We will thus use a marker trait and autoref specialization to implement it for the
//! objects which are compatible.

// This ended up being a lot more involved than originally anticipated.

use core::marker::PhantomData;

use super::*;

pub trait NonMaxBytesNiche: ConstSongSize {}

struct OptionHelper<T>(PhantomData<T>);

trait OptionHelperTrait<T> {
    fn helper_song_size(&self, option: &Option<T>) -> usize where T: SongSize;
    fn helper_from_song(&self, buf: &[u8]) -> Result<Option<T>, FromSongError> where T: FromSong;
    fn helper_to_song(&self, option: &Option<T>, buf: &mut [u8]) -> Result<(), ToSongError> where T: ToSong;
}

impl <T: NonMaxBytesNiche + ConstSongSize> OptionHelperTrait<T> for &OptionHelper<T> {
    fn helper_song_size(&self, _: &Option<T>) -> usize {
        T::SONG_SIZE
    }
    fn helper_from_song(&self, buf: &[u8]) -> Result<Option<T>, FromSongError> where T: FromSong {
        if buf.len() < T::SONG_SIZE {
            return Err(FromSongError::BufferOverflow)
        }

        if buf[0..T::SONG_SIZE].iter().all(|b| *b == 0xFF) {
            Ok(None)
        } else {
            Ok(Some(T::from_song(buf)?))
        }
    }
    fn helper_to_song(&self, option: &Option<T>, buf: &mut [u8]) -> Result<(), ToSongError> where T: ToSong {
        if buf.len() < T::SONG_SIZE {
            return Err(ToSongError::BufferOverflow)
        }

        match &option {
            Some(value) => value.to_song(buf)?,
            None => buf[0..T::SONG_SIZE].fill(0xFF),
        }

        Ok(())
    }
}

impl <T> OptionHelperTrait<T> for OptionHelper<T> {
    fn helper_song_size(&self, option: &Option<T>) -> usize where T: SongSize {
        match &option {
            Some(val) => val.song_size() + 1,
            None => 1
        }
    }
    fn helper_from_song(&self, buf: &[u8]) -> Result<Option<T>, FromSongError> where T: FromSong {
        if buf.len() < 1 {
            return Err(FromSongError::BufferOverflow)
        }

        match buf[0] {
            0x00 => Ok(None),
            0x01 => Ok(Some(T::from_song(&buf[1..])?)),
            _ => Err(FromSongError::InvalidValue)
        }
    }
    fn helper_to_song(&self, option: &Option<T>, buf: &mut [u8]) -> Result<(), ToSongError> where T: ToSong {
        if buf.len() < self.helper_song_size(option) {
            return Err(ToSongError::BufferOverflow)
        }

        match &option {
            Some(val) => {
                buf[0] = 0x01;
                val.to_song(&mut buf[1..])
            }
            None => {
                buf[0] = 0;
                Ok(())
            }
        }
    }
}

impl <T: ConstSongSize + NonMaxBytesNiche> HasSongSize for Option<T> {
    type Size = ConstSongSizeImplFromConstSongSize<T>;
}

impl <T: SongSize> SongSize for Option<T> {
    fn song_size(self: &Self) -> usize {
        (&OptionHelper(PhantomData)).helper_song_size(self)
    }
}

impl <T: ToSong> ToSong for Option<T> {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
        (&OptionHelper(PhantomData)).helper_to_song(self, buf)
    }
}

impl <T: FromSong> FromSong for Option<T> {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized {
        (&OptionHelper(PhantomData)).helper_from_song(buf)
    }
}