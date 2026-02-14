use sirin_macros::{FromSong, SongSize, ToSong};
use crate::song::*;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, SongSize, ToSong, FromSong)]
pub struct AbsoluteTimeReference {
    pub ms_since_epoch: u64
}