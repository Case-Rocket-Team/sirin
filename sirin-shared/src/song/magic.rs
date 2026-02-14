use super::*;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MagicU8<const BYTE: u8>;

impl <const BYTE: u8> HasSongSize for MagicU8<BYTE> {
    type Size = ConstSongSizeValue<1>;
}

impl <const BYTE: u8> SongSize for MagicU8<BYTE> {
    fn song_size(self: &Self) -> usize {
        1
    }
}

impl <const BYTE: u8> FromSong for MagicU8<BYTE> {
    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> where Self: Sized {
        if buf.len() < 1 {
            return Err(FromSongError::BufferOverflow)
        }

        if buf[0] != BYTE {
            return Err(FromSongError::InvalidValue)
        }

        Ok(Self {})
    }
}

impl <const BYTE: u8> ToSong for MagicU8<BYTE> {
    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
        if buf.len() < 1 {
            return Err(ToSongError::BufferOverflow)
        }

        buf[0] = BYTE;
        Ok(())
    }
}