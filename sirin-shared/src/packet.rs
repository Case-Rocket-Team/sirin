use crate::{song::*, state::State};
use sirin_macros::*;

pub const MAX_OUT_PACKET_SIZE: usize = 256;

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
    QueryInfo,
    DumpFlash
}

/*
#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct Packet<D: SongSize + ToSong + FromSong> {
    /// Ticks
    time: u32,
    data: D
}

impl <D: SongSize + ToSong + FromSong> Packet<D> {
    pub fn new(time: u32, data: D) -> Self {
        Self {
            time,
            data
        }
    }
}

pub type OutPacket = Packet<OutData>;
pub type InPacket = Packet<InData>;
*/