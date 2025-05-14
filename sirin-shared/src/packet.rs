use crate::{config::{CallsignBuf, SirinConfig, SirinId}, mode::SirinMode, song::*, state::State, time::AbsoluteTimeReference};
use derive_more::Display;
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

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[song(discriminant(OutPacketType = u8))]
pub enum OutPacket {
    Null,
    Error(PacketError),
    Config(SirinConfig),
    Mode(SirinMode),
    FlightStart(u8),
    State(State),
    FlashPageDump(FlashPageDump),
    FlightHeader(Paginated<FlightHeader>)
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct Paginated<T: SongSize + ToSong + FromSong> {
    pub index: u16,
    pub len: u16,
    pub data: T
}

impl <T: SongSize + ToSong + FromSong> Paginated<T> {
    pub fn new(index: u16, len: u16, data: T) -> Self {
        Paginated {
            index,
            len,
            data
        }
    }
}

#[derive(Debug, Display, Clone, PartialEq, Eq, SongSize, ToSong, FromSong)]
#[song(discriminant(PacketErrorType = u8))]
pub enum PacketError {
    #[display("The packet type {_1:?} is not supported over {_0:?}")]
    PacketNotSupportedOverChannel(IoChannel, InPacketType),
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
    QueryMode,
    SetMode(SirinMode),
    QueryFlights
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, SongSize, ToSong, FromSong)]
#[repr(u8)]
pub enum IoChannel {
    Broadcast = 0,
    Usb,
    LoRa,
    Flash
}

#[derive(Debug, Clone, PartialEq, Eq)]
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

    /// TODO: implement full request-response with packet ids
    pub fn reply<R: SongSize + ToSong + FromSong>(&self, packet: R) -> IoPacket<R> {
        IoPacket::new(self.channel, packet)
    }
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct RadioPacket<P: SongSize + ToSong + FromSong> {
    id: SirinId,
    callsign: CallsignBuf,
    packet: P
}

impl <P: SongSize + ToSong + FromSong> RadioPacket<P> {
    pub fn new(config: &SirinConfig, packet: P) -> Self {
        let callsign = config.callsign.clone();

        Self {
            id: config.id,
            callsign,
            packet
        }
    }
}

#[derive(Clone, Debug, SongSize, ToSong, FromSong)]
pub struct FlightHeader {
    /// Tracking byte -- when this flight is overwritten in the cyclic flash, this byte is zeroed out.
    /// This field must be first!
    pub status: FlightHeaderStatus,

    /// Address of the start of flight logs, without offset.
    addr: [u8; 3],

    pub time_reference: Option<AbsoluteTimeReference>,

    /// Time of the flight in terms of ticks since boot
    pub timestamp: u64
}

impl FlightHeader {
    pub fn new(addr: u32, time_reference: Option<AbsoluteTimeReference>, timestamp: u64) -> Self {
        let arr = addr.to_le_bytes();

        Self {
            status: FlightHeaderStatus::Valid,
            addr: [arr[0], arr[1], arr[2]],
            time_reference,
            timestamp
        }
    }

    pub fn addr(&self) -> u32 {
        let arr = [self.addr[0], self.addr[1], self.addr[2], 0];
        u32::from_le_bytes(arr)
    }
}

#[derive(Clone, Copy, Debug, SongSize, ToSong, FromSong, PartialEq, Eq)]
#[repr(u8)]
pub enum FlightHeaderStatus {
    // Initial value of NOR flash is 0xFF
    Null = 0xFF,
    Valid = 0x01,
    Overwritten = 0x00
}