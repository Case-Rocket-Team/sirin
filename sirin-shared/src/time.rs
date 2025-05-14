use sirin_macros::{FromSong, SongSize, ToSong};
use crate::song::{option::NonMaxBytesNiche, *};

#[derive(Clone, Debug, SongSize, ToSong, FromSong)]
pub struct AbsoluteTimeReference {
    pub ticks_since_epoch: u64,
    pub tick_hz: u64,
}

impl NonMaxBytesNiche for AbsoluteTimeReference {}