use defmt::{error, info, println, Debug2Format};
use embassy_executor::task;
use embassy_futures::{join::join, select::{select, Either}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TryReceiveError, TrySendError}, pubsub::{PubSubBehavior, PubSubChannel as EmbassyPubSubChannel, Subscriber}};
use embassy_usb::{driver::{Endpoint, EndpointIn, EndpointOut}, UsbDevice};
use sirin_macros::{FromSong, SongSize, ToSong};
use crate::usb::{SirinUsb, WriteEp, ReadEp};

use crate::{error::SirinError, Flash, Radio};
use sirin_shared::{packet::{InPacket, IoChannel, IoPacket, OutPacket, SirinConfig, MAX_OUT_PACKET_SIZE}, song::{FromSong, FromSongError, SongSize, ToSong, ToSongError}};

//pub static OUT_CHANNEL: Channel<CriticalSectionRawMutex, OutPacket, 10> = Channel::new();

type PubSubChannel<T, const SUBS: usize> = EmbassyPubSubChannel<CriticalSectionRawMutex, T, 32, SUBS, 0>;

// Lower priority general broadcast channel.
pub static BROADCAST_CHANNEL: PubSubChannel<OutPacket, 3> = EmbassyPubSubChannel::new();

// High priority I/O channels
pub static OUT_CHANNEL: PubSubChannel<IoPacket<OutPacket>, 3> = EmbassyPubSubChannel::new();
pub static IN_CHANNEL: Channel<CriticalSectionRawMutex, IoPacket<InPacket>, 32> = Channel::new();

pub fn broadcast(packet: OutPacket) {
    info!("{:?}", Debug2Format(&packet));
    BROADCAST_CHANNEL.publish_immediate(packet);
}

pub fn send_packet(packet: IoPacket<OutPacket>) {
    info!("{:?}", Debug2Format(&packet));
    OUT_CHANNEL.publish_immediate(packet);
}

pub async fn receive_packet() -> IoPacket<InPacket> {
    IN_CHANNEL.receive().await
}

pub fn try_receive_packet() -> Result<IoPacket<InPacket>, TryReceiveError> {
    IN_CHANNEL.try_receive()
}

fn received_packet(mut packet: IoPacket<InPacket>) {
    while let Err(err) = IN_CHANNEL.try_send(packet) {
        // drop the last packet in the queue
        match try_receive_packet() {
            Ok(p) => drop(p),
            Err(err) => {
                defmt::error!("Error trying to receive InPacket: {}", err);
                return
            }
        }

        // put the packet back (it was moved in `.try_send()`)
        match err {
            TrySendError::Full(p) => packet = p
        }
    }
}

async fn next_out_packet(
    broadcast: &mut Subscriber<'static, CriticalSectionRawMutex, OutPacket, 32, 3, 0>,
    out: &mut Subscriber<'static, CriticalSectionRawMutex, IoPacket<OutPacket>, 32, 3, 0>,
    channel: IoChannel
) -> OutPacket {
    loop {
        let res = select(
            out.next_message_pure(),
            broadcast.next_message_pure()
        ).await;

        return match res {
            Either::First(p) => {
                if p.channel == channel {
                    p.packet
                } else {
                    continue;
                }
            },
            Either::Second(p) => p
        }
    }
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct RadioOutPacket {
    callsign: [u8; 32],
    packet: OutPacket
}

impl RadioOutPacket {
    pub fn new(config: &SirinConfig, packet: OutPacket) -> Self {
        let callsign = config.callsign.clone();

        Self {
            callsign,
            packet
        }
    }
}

#[task]
pub async fn radio_io_task(
    config: &'static SirinConfig,
    radio: &'static mut Radio,
) {
    loop {
        match radio_task_impl(config, radio).await {
            Ok(()) => {},
            Err(e) => {
                defmt::error!("Error in radio task: {}", Debug2Format(&e))
            }
        }
    }
}

async fn radio_task_impl(
    config: &'static SirinConfig,
    radio: &mut Radio,
) -> Result<(), SirinError> {
    let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = OUT_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    loop {
        let packet = next_out_packet(
            &mut broadcast_sub,
            &mut out_sub,
            IoChannel::Usb
        ).await;
        let radio_packet = RadioOutPacket::new(config, packet);

        radio_packet.to_song(&mut buf)?;

        radio.transmit(&buf[0..radio_packet.song_size()]).await?;
    }
}

#[task]
pub async fn usb_output_task(
    usb: &'static mut WriteEp
) {
    loop {
        match usb_output_task_impl(usb).await {
            Ok(()) => {},
            Err(e) => {
                defmt::error!("Error in usb output task: {}", Debug2Format(&e))
            }
        }
    }
}

async fn usb_output_task_impl(
    usb: &mut WriteEp
) -> Result<(), SirinError> {
    let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = OUT_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    loop {
        usb.wait_enabled().await;

        let packet = next_out_packet(
            &mut broadcast_sub,
            &mut out_sub,
            IoChannel::Usb
        ).await;

        packet.to_song(&mut buf)?;
        let len = packet.song_size();

        // Need to chop it up into 64-byte sized packets (full speed device)
        let mut i = 0;
        while i < len {
            let j = (i + 64).min(len);
            usb.write(&buf[i..j]).await?;
            i = j;
        }

        info!("{}", packet.song_size())
    }
}

#[task]
pub async fn usb_input_task(
    usb: &'static mut ReadEp
) {
    loop {
        match usb_input_task_impl(usb).await {
            Ok(()) => {},
            Err(e) => {
                defmt::error!("Error in usb input task: {}", Debug2Format(&e))
            }
        }
    }
}

async fn usb_input_task_impl(
    usb: &mut ReadEp
) -> Result<(), SirinError> {
    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];
    usb.wait_enabled().await;
    usb.read(&mut buf).await?;
    error!("Read packet: {:?}", buf);
    let packet = InPacket::from_song(&buf)?;
    received_packet(IoPacket::new(IoChannel::Usb, packet));
    Ok(())
}

pub async fn flash_io_task(flash: &'static mut Flash){
    loop {
        match flash_task_impl(flash).await {
            Ok(()) => {},
            Err(e) => {
                defmt::error!("Error in flash task: {}", Debug2Format(&e))
            }
        }
    }
}

pub async fn flash_task_impl(flash: &mut Flash) -> Result<(), SirinError> {
    let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = OUT_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    loop {
        let packet = next_out_packet(
            &mut broadcast_sub,
            &mut out_sub,
            IoChannel::Flash
        ).await;
        packet.to_song(&mut buf)?;  
        flash.log(&buf[0..packet.song_size()]).await.unwrap();
    }
}