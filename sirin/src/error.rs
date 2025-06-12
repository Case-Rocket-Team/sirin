use derive_more::From;
use embassy_stm32::usart;
use embedded_hal::spi::ErrorKind;
use sirin_shared::song::{ToSongError, FromSongError};
use embassy_sync::pubsub::Error as PubSubError;
use rfm9::Rfm9Error;
use embassy_usb::driver::EndpointError as USBError;

#[derive(Debug, From)]
pub enum SirinError {
    ToSongError(ToSongError),
    FromSongError(FromSongError),
    Rfm9Error(Rfm9Error),
    UsbError(USBError),
    PubSubError(PubSubError),
    SpiError(ErrorKind),
    UsartError(usart::Error),
    CorruptedData
}