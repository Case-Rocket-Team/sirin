use derive_more::Display;
use sirin_macros::{FromSong, SongSize, ToSong};
use crate::song::*;

/// The phase of flight the flight computer believes it is in.
///
/// ```text
/// Standby --launch--> Flight --apogee--> Descent --landing--> Landed
/// ```
///
/// See [`crate::flight::FlightDetector`] for the detection logic that moves between modes.
///
/// The numeric value of each variant is written to flash logs and sent over USB/radio, and is
/// decoded by tools outside this repository (e.g. `sirin-cli`). Never renumber a variant.
/// `Descent` was added after `Landed` already existed, which is why it is not numbered in
/// flight order.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Display, PartialEq, Eq, SongSize, FromSong, ToSong)]
#[repr(u8)]
pub enum SirinMode {
    /// On the pad, waiting for launch.
    Standby = 0,
    /// Launched and ascending; apogee has not been declared yet.
    Flight = 1,
    /// Apogee has been declared; coming down.
    Descent = 3,
    /// On the ground after the flight.
    Landed = 2,
}

impl SirinMode {
    /// True while the rocket is in the air (`Flight` or `Descent`).
    pub fn is_airborne(self) -> bool {
        matches!(self, SirinMode::Flight | SirinMode::Descent)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [SirinMode; 4] = [SirinMode::Standby, SirinMode::Flight, SirinMode::Descent, SirinMode::Landed];

    /// The discriminants are part of the flash and wire formats. If this test fails, logged
    /// flights and the CLI will decode the wrong mode.
    #[test]
    fn discriminants_are_stable() {
        assert_eq!(SirinMode::Standby as u8, 0);
        assert_eq!(SirinMode::Flight as u8, 1);
        assert_eq!(SirinMode::Landed as u8, 2);
        assert_eq!(SirinMode::Descent as u8, 3);
    }

    #[test]
    fn song_round_trip() {
        for mode in ALL {
            let mut buf = [0xFFu8; 1];
            assert_eq!(mode.song_size(), 1);
            mode.to_song(&mut buf).unwrap();
            assert_eq!(buf[0], mode as u8);
            assert_eq!(SirinMode::from_song(&buf).unwrap(), mode);
        }

        assert_eq!(SirinMode::from_song(&[4]), Err(FromSongError::InvalidPacketId));
        assert_eq!(SirinMode::from_song(&[]), Err(FromSongError::BufferOverflow));
    }

    #[test]
    fn airborne_modes() {
        assert!(!SirinMode::Standby.is_airborne());
        assert!(SirinMode::Flight.is_airborne());
        assert!(SirinMode::Descent.is_airborne());
        assert!(!SirinMode::Landed.is_airborne());
    }
}
