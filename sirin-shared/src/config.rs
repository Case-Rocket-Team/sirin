use core::fmt::Display;

use sirin_macros::*;

use crate::{byte_array_str, packet::ByteArrayStr, song::*};

pub type NicknameBuf = [u8; 32];
pub type CallsignBuf = [u8; 8];
pub type SirinId = u16;


#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct SirinConfig {
    /// Nickname for this device. Pad end with null bytes.
    pub nickname: NicknameBuf,

    /// FAA HAM Callsign. Set to all zeros if you aren't using one. Pad end with null bytes.
    pub callsign: CallsignBuf,

    /// An ID for this device. It shouldn't be the same as any other device in range on the
    /// same frequency.
    pub id: SirinId,
}

impl Default for SirinConfig {
    fn default() -> Self {
        Self {
            nickname: byte_array_str!(32, b"Sirin"),
            callsign: [0; 8], // Null callsign,
            id: 0
        }
    }
}

impl Display for SirinConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f,
             "Nickname:        {}\
            \nFAA Callsign:    {}\
            \nID:              0x{:X}\
            ",
            self.nickname.as_str(),
            self.callsign.as_str(),
            self.id
        )?;
        Ok(())
    }
}