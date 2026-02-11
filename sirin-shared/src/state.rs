use core::ops::{Add, Sub};
use num_traits::{One, Zero};
use sirin_macros::{SongSize, ToSong, FromSong};
use uunit::{Meters, MetersPerSecond, MetersPerSecond2, RadiansPerSecond, WithUnits};
use crate::song::*;

#[cfg(feature = "serde")]
use serde::{Serialize, Deserialize};

macro_rules! impl_vec {
    ($ident:ident : $($field: ident),*) => {
        impl Add for $ident {
            type Output = Self;
            fn add(mut self, rhs: Self) -> Self::Output {
                $(self.$field += rhs.$field;)*
                self
            }
        }

        impl Sub for $ident {
            type Output = Self;
            fn sub(mut self, rhs: Self) -> Self::Output {
                $(self.$field -= rhs.$field;)*
                self
            }
        }

        impl Zero for $ident {
            fn is_zero(&self) -> bool {
                true
                $(&& self.$field.is_zero())*
            }

            fn zero() -> Self {
                Self {
                    $($field: 0.0f32.with_units(),)*
                }
            }
        }
    };
}

/// ECEF Position
/// https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
pub struct Pos {
    pub x: Meters<f32>,
    pub y: Meters<f32>,
    pub z: Meters<f32>
}

impl_vec!(Pos: x, y, z);

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
pub struct Vel {
    pub x: MetersPerSecond<f32>,
    pub y: MetersPerSecond<f32>,
    pub z: MetersPerSecond<f32>,
}

impl_vec!(Vel: x, y, z);

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
pub struct Accel {
    pub x: MetersPerSecond2<f32>,
    pub y: MetersPerSecond2<f32>,
    pub z: MetersPerSecond2<f32>,
}

impl_vec!(Accel: x, y, z);

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
pub struct AngularVel {
    pub x_pitch: RadiansPerSecond<f32>,
    pub y_roll: RadiansPerSecond<f32>,
    pub z_yaw: RadiansPerSecond<f32>,
}

impl_vec!(AngularVel: x_pitch, y_roll, z_yaw);

// TODO: We can remove the bounds on T and then conditionally implement
// SongSize, ToSong, FromSong if T satisfies those bounds, but there isn't
// a way to do that with the macro right now.
#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
pub struct Quaternion<T: SongSize + ToSong + FromSong> {
    pub r: T,
    pub x: T,
    pub y: T,
    pub z: T
}

impl <T: SongSize + ToSong + FromSong> Quaternion<T> {
    pub fn new(r: T, x: T, y: T, z: T) -> Self {
        Quaternion {
            r, x, y, z
        }
    }

    pub fn identity() -> Self where T: Zero + One {
        Self::new(T::one(), T::zero(), T::zero(), T::zero())
    }
}

// MUST correspond to NominalState in sirin-c.c
#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
pub struct NominalState {
    pub pos: Pos,
    pub vel: Vel,
    pub accel: Accel,

    pub rot_quaternion: Quaternion<f32>,

    pub accel_bias: Accel,
    pub angular_vel_bias: AngularVel,
    //pub gravity: Accel
}

impl Default for NominalState {
    fn default() -> Self {
        Self {
            pos: Pos::zero(),
            vel: Vel::zero(),
            accel: Accel::zero(),

            rot_quaternion: Quaternion::identity(),
            
            accel_bias: Accel::zero(),
            angular_vel_bias: AngularVel::zero(),
            //gravity: Accel::zero()
        }
    }
}

// MUST correspond to ErrorState in sirin-c.c
#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
pub struct ErrorState {
    pub pos: Pos,
    pub vel: Vel,
    pub accel: Accel,

    pub accel_bias: Accel,
    pub angular_vel_bias: AngularVel,
    //pub gravity: Accel

    pub angles_vector: [f32; 3]
}

impl Default for ErrorState {
    fn default() -> Self {
        Self {
            pos: Pos::zero(),
            vel: Vel::zero(),
            accel: Accel::zero(),
            
            accel_bias: Accel::zero(),
            angular_vel_bias: AngularVel::zero(),
            //gravity: Accel::zero()

            angles_vector: [0.0; 3]
        }
    }
}
