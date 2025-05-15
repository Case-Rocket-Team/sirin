use super::*;

impl <T: SongSize> SongSize for Option<T> {
    fn song_size(self: &Self) -> usize {
        match &self {
            Some(val) => val.song_size() + 1,
            None => 1
        }
    }
}

impl <T: ToSong> ToSong for Option<T> {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
        if buf.len() < self.song_size() {
            return Err(ToSongError::BufferOverflow)
        }

        match &self {
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

impl <T: FromSong> FromSong for Option<T> {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized {
        if buf.len() < 1 {
            return Err(FromSongError::BufferOverflow)
        }

        match buf[0] {
            0x00 => Ok(None),
            0x01 => Ok(Some(T::from_song(&buf[1..])?)),
            _ => Err(FromSongError::InvalidValue)
        }
    }
}