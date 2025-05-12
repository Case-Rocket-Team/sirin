use derive_more::Display;
use sirin_macros::{FromSong, SongSize, ToSong};
use crate::{config::SirinId, song::*};

#[derive(Clone, Copy, Debug, Display, PartialEq, Eq, SongSize, FromSong, ToSong)]
#[repr(u8)]
pub enum SirinMode {
    Standby,
    Flight,
}