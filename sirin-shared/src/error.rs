use crate::song::*;
use embedded_hal::spi::ErrorKind;
use sirin_macros::{SongSize, ToSong, FromSong};
use derive_more::{Display, From};
use paste::paste;

macro_rules! sirin_error_locations {
    ($($loc: ident),*) => {
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[derive(Debug, Display, Copy, Clone, PartialEq, Eq, SongSize, ToSong, FromSong, From)]
        #[repr(u8)]
        pub enum SirinErrorLocation {
            $($loc),*
        }

        $(
            paste! {
                #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
                #[derive(Debug, Display, Clone, PartialEq, Eq, SongSize, ToSong, FromSong, From)]
                #[repr(transparent)]
                #[from(forward)]
                pub struct [< Sirin $loc Error >] {
                    error: SirinErrorVariant
                }

                impl From<[< Sirin $loc Error >]> for SirinErrorLocation {
                    fn from(_: [< Sirin $loc Error >]) -> Self {
                        SirinErrorLocation::$loc
                    }
                }

                impl From<[< Sirin $loc Error >]> for SirinError {
                    fn from(value: [< Sirin $loc Error >]) -> Self {
                        Self {
                            location: SirinErrorLocation::$loc,
                            error: value.error
                        }
                    }
                }
            }
        )*
    };
}

sirin_error_locations!{
    Init,
    Main,
    Radio,
    Imu,
    HighGImu,
    Accel,
    Baro,
    KalmanFilter
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Display, Clone, PartialEq, Eq, SongSize, ToSong, FromSong, From)]
#[display("Error in #{location}: #{error}")]
pub struct SirinError {
    pub location: SirinErrorLocation,
    pub error: SirinErrorVariant,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Display, Clone, PartialEq, Eq, SongSize, ToSong, FromSong, From)]
#[song(discriminant(SirinErrorVariantType = u8))]
pub enum SirinErrorVariant {
    //#[display("The packet type {_1:?} is not supported over {_0:?}.")]
    //PacketNotSupportedOverChannel(IoChannel, RequestPacketDataType),
    SanityCheckFailed,
    // TODO: preserve the kind
    #[from(ErrorKind)]
    SpiError,
    NotYetMeasured,
    
    #[display("Flight #{_0} could not be found.")]
    FlightNotFound(u16),

    #[display("There was an error during serialization: #{_0}")]
    #[from(ToSongError)]
    ToSongError(ToSongError),

    #[display("There was an error during deserialization: #{_0}")]
    #[from(FromSongError)]
    FromSongError(FromSongError),

    /*Rfm9Error(Rfm9Error),
    UsbError(USBError),
    PubSubError(PubSubError),
    SpiError(ErrorKind),
    UsartError(usart::Error),*/
    CorruptedData,
    //GpsPacketParseError(ParserError),

    Unspecified
}

// make sure everything works

fn radio_fn() -> Result<(), SirinRadioError> {
    
}