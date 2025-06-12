use crate::song::{FromSong, FromSongError, SongSize, ToSong, ToSongError};

impl <T: SongSize, E: SongSize> SongSize for Result<T, E> {
    fn song_size(self: &Self) -> usize {
        match self {
            Ok(item) => item.song_size() + 1,
            Err(err) => err.song_size() + 1
        }
    }
}

impl <T: SongSize + ToSong, E: SongSize + ToSong> ToSong for Result<T, E> {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), super::ToSongError> {
        match self {
            Ok(item) => {
                if buf.len() == 0 {
                    return Err(ToSongError::BufferOverflow)
                }

                buf[0] = 0x12;
                item.to_song(&mut buf[1..])?;
                Ok(())
            },
            Err(err) => {
                if buf.len() == 0 {
                    return Err(ToSongError::BufferOverflow)
                }
                
                buf[0] = 0x34;
                err.to_song(&mut buf[1..])?;
                Ok(())
            }
        }
    }
}

impl <T: SongSize + FromSong, E: SongSize + FromSong> FromSong for Result<T, E> {
    fn from_song(buf: &[u8]) -> Result<Self, super::FromSongError> where Self: Sized {
        if buf.len() == 0 {
            return Err(FromSongError::BufferOverflow)
        }

        match buf[0] {
            0x12 => Ok(Ok(T::from_song(&buf[1..])?)),
            0x34 => Ok(Err(E::from_song(&buf[1..])?)),
            _ => Err(FromSongError::InvalidValue)
        }
    }
}