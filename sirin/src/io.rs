use defmt::Debug2Format;
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{PubSubBehavior, PubSubChannel}};
use embassy_usb::{driver::Endpoint, UsbDevice};
use sirin_macros::{FromSong, SongSize, ToSong};
use crate::FlashLogger;

use crate::{error::SirinError, song::{FromSong, FromSongError, OutPacket, SongSize, ToSong, ToSongError, MAX_OUT_PACKET_SIZE}, usb, Flash, Radio, UsbSerial};

//pub static OUT_CHANNEL: Channel<CriticalSectionRawMutex, OutPacket, 10> = Channel::new();
pub static OUT_CHANNEL: PubSubChannel<CriticalSectionRawMutex, OutPacket, 32, 3, 0> = PubSubChannel::new();

pub fn out(packet: OutPacket) {
    OUT_CHANNEL.publish_immediate(packet)
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct RadioOutPacket {
    callsign: [u8; 16],
    packet: OutPacket
}

pub struct USBOutPacket {
    callsign: [u8; 16],
    packet: OutPacket
}

impl RadioOutPacket {
    pub fn new(packet: OutPacket) -> Self {
        let mut callsign = [0u8; 16];
        // TODO: implement actual config
        callsign[0..6].clone_from_slice(b"KF8BAA");

        Self {
            callsign,
            packet
        }
    }
}

#[task]
pub async fn radio_io_task(
    radio: &'static mut Radio,
) {
    loop {
        match radio_task_impl(radio).await {
            Ok(()) => {},
            Err(e) => {
                defmt::error!("Error in radio task: {}", Debug2Format(&e))
            }
        }
    }
}

async fn radio_task_impl(
    radio: &mut Radio,
) -> Result<(), SirinError> {
    let mut sub = OUT_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    loop {
        // todo handle lag error
        let packet = sub.next_message_pure().await;

        packet.to_song(&mut buf)?;

        radio.transmit(&buf[0..packet.song_size()]).await?;
    }
}

#[task]
pub async fn usb_io_task(
    usb: &'static mut UsbSerial
) {
    loop {
        match usb_task_impl(usb).await {
            Ok(()) => {},
            Err(e) => {
                defmt::error!("Error in usb task: {}", Debug2Format(&e))
            }
        }
    }
}


async fn usb_task_impl(
    usb: &mut UsbSerial
) -> Result<(), SirinError > {
    let mut sub = OUT_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    loop {
        usb.wait_connection().await;
        // todo handle lag error
        let packet = sub.next_message_pure().await;

        packet.to_song(&mut buf)?;
        usb.write_packet(&buf[0..packet.song_size()]).await?;
    }
}

pub async fn flash_io_task(flash: &'static mut Flash){
    let mut logger = FlashLogger::new();
    loop {
        match flash_task_impl(flash, &mut logger).await {
            Ok(()) => {},
            Err(e) => {
                defmt::error!("Error in flash task: {}", Debug2Format(&e))
            }
        }
    }
}

pub async fn flash_task_impl(flash: &mut Flash, logger: &mut FlashLogger) -> Result<(), SirinError> {
    let mut sub = OUT_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    loop {
        let packet = sub.next_message_pure().await;
        packet.to_song(&mut buf)?;  
        logger.log(flash, &buf[0..packet.song_size()]).await.unwrap();
    }
}