use derive_more::Display;
use sirin_macros::{FromSong, SongSize, ToSong};
use crate::song::*;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Display, PartialEq, Eq, SongSize, FromSong, ToSong)]
#[repr(u8)]
pub enum SirinMode {
    Standby,
    Flight,
    Landed,
}