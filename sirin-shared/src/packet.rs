use core::fmt::Display;

use crate::{song::*, state::State};
use sirin_macros::*;

pub const MAX_OUT_PACKET_SIZE: usize = 256;

#[macro_export]
/// Generate a null-terminated string of fixed length
macro_rules! byte_array_str {
    ($len: expr, $bytestring: expr) => {
        {
            let mut bytes = [0u8; $len];
            let str = $bytestring;
            bytes[0..(str.len())].copy_from_slice(str);
            bytes
        }
    };
}

pub use byte_array_str;

#[derive(Debug)]
pub enum ByteArrayStrError {
    /// Str is too long for the buffer
    TooLong
}

pub trait ByteArrayStr: Sized {
    /// Returns a byte array as `&str`, truncated before the first non-ascii or null byte.
    fn as_str(&self) -> &str;
    fn null_terminate(&self) -> &[u8];
    fn from_str(str: &str) -> Result<Self, ByteArrayStrError>;
}

impl <const SIZE: usize> ByteArrayStr for [u8; SIZE] {
    fn as_str(&self) -> &str {
        for i in 0..self.len() {
            if  self[i] == 0 || !self[i].is_ascii() {
                unsafe {
                    return core::str::from_utf8_unchecked(&self[0..i])
                }
            }
        }

        unsafe {
            return core::str::from_utf8_unchecked(self)
        }
    }

    fn null_terminate(&self) -> &[u8] {
        let mut len = 0;
        while len < self.len() && self[len] != 0 {
            len += 1;
        }
    
        &self[..len]
    }

    fn from_str(str: &str) -> Result<Self, ByteArrayStrError> {
        let mut buf = [0; SIZE];

        if str.len() > SIZE {
            Err(ByteArrayStrError::TooLong)
        } else {
            buf[0..str.len()].copy_from_slice(str.as_bytes());
            Ok(buf)
        }
    }
}

pub type NicknameBuf = [u8; 32];
pub type CallsignBuf = [u8; 32];


#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct SirinConfig {
    /// Nickname for this device. Pad end with null bytes.
    pub nickname: NicknameBuf,

    /// FAA HAM Callsign. Set to all zeros if you aren't using one. Pad end with null bytes.
    pub callsign: CallsignBuf,
}

impl Default for SirinConfig {
    fn default() -> Self {
        Self {
            nickname: byte_array_str!(32, b"Sirin"),
            callsign: [0; 32] // Null callsign
        }
    }
}

impl Display for SirinConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f,
            "Nautki Sirin S1-R2\n    Nickname: {}\n    FAA Callsign: {}",
            self.nickname.as_str(),
            self.callsign.as_str()
        )?;
        Ok(())
    }
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[song(discriminant(OutPacketType = u8))]
pub enum OutPacket {
    Null,
    Config(SirinConfig),
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
    QueryConfig,
    SetConfig(SirinConfig),
    DumpFlash
}

#[derive(Debug, Clone, PartialEq, Eq, SongSize, ToSong, FromSong)]
#[song(discriminant(IoChannelType = u8))]
pub enum IoChannel {
    Broadcast = 0,
    Usb,
    LoRa,
    Flash
}

#[derive(Debug, Clone, PartialEq, Eq, SongSize, ToSong, FromSong)]
pub struct IoPacket<P: SongSize + ToSong + FromSong> {
    pub channel: IoChannel,
    pub packet: P
}

impl <P: SongSize + ToSong + FromSong> IoPacket<P> {
    pub fn new(channel: IoChannel, packet: P) -> Self {
        Self {
            channel,
            packet
        }
    }
}