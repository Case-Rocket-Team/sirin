use core::ops::Div;
use sirin_macros::{SongSize, ToSong, FromSong};
use uunit::{Degrees, Meters, Quantity, UnitMeters, UnitSeconds};
use crate::song::*;

type MetersPerSecond<T> = Quantity<T, <UnitMeters as Div<UnitSeconds>>::Output>;
type MetersPerSecond2<T> = Quantity<T, <UnitMeters as Div<<UnitMeters as Div<UnitSeconds>>::Output>>::Output>;


#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[repr(C)]
pub struct Vel {
    pub x: MetersPerSecond<f64>,
    pub y: MetersPerSecond<f64>,
    pub z: MetersPerSecond<f64>,
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[repr(C)]
pub struct Accel {
    pub x: MetersPerSecond2<f64>,
    pub y: MetersPerSecond2<f64>,
    pub z: MetersPerSecond2<f64>,
}

/// ECEF Position
/// https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[repr(C)]
pub struct EcefPos {
    pub x: Meters<f64>,
    pub y: Meters<f64>,
    pub z: Meters<f64>
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[repr(C)]
pub struct State {
    pub pos: EcefPos,
    pub vel: Vel,
    pub accel: Accel,
    pub altitude: Meters<f64>
}
