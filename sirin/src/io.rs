use core::{char::MAX, future::{Future, poll_fn}, marker::PhantomData, mem::transmute, sync::atomic::{AtomicBool, Ordering}, task::Poll};

use defmt::{error, info, println, Debug2Format};
use embassy_executor::task;
use embassy_futures::{join::join, select::{select, Either}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TryReceiveError, TrySendError}, pubsub::{PubSubBehavior, PubSubChannel as EmbassyPubSubChannel, Subscriber}, signal::Signal, waitqueue::AtomicWaker};
use embassy_time::Instant;
use embassy_usb::{driver::{Endpoint, EndpointIn, EndpointOut}, UsbDevice};
use sirin_macros::{FromSong, SongSize, ToSong};
use uunit::Milliseconds;
use crate::{sync::Mutex, usb::{ReadEp, SirinUsb, WriteEp}};

use crate::{error::SirinError, Flash, Radio};
use sirin_shared::{config::{CallsignBuf, SirinConfig}, packet::{InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, RadioPacket, MAX_OUT_PACKET_SIZE}, song::{FromSong, FromSongError, SongSize, ToSong, ToSongError}};

//pub static OUT_CHANNEL: Channel<CriticalSectionRawMutex, OutPacket, 10> = Channel::new();

type PubSubChannel<T, const SUBS: usize> = EmbassyPubSubChannel<CriticalSectionRawMutex, T, 32, SUBS, 0>;

// Lower priority general broadcast channel.
pub static BROADCAST_CHANNEL: PubSubChannel<OutPacket, 3> = EmbassyPubSubChannel::new();

// High priority I/O channels
pub static OUT_CHANNEL: PubSubChannel<IoPacket<OutPacket>, 3> = EmbassyPubSubChannel::new();
pub static IN_CHANNEL: Channel<CriticalSectionRawMutex, IoPacket<InPacket>, 32> = Channel::new();
pub static INTERNAL_CHANNEL: Channel<CriticalSectionRawMutex, IoPacket<InPacket>, 32> = Channel::new();
pub static INTERNAL_SENDING_ENABLED: AtomicBool = AtomicBool::new(false);

static USB_BROADCASTING_ENABLED: AtomicBool = AtomicBool::new(false);
pub static FLASH_LOGGING_ENABLED: AtomicBool = AtomicBool::new(false);

pub fn set_inpacket_sending_enabled(bool: bool){
    INTERNAL_SENDING_ENABLED.store(bool, Ordering::Relaxed);
}

pub fn broadcast_log(time: Milliseconds<u32>, log: Log) {
    broadcast(OutPacket::LogEntry(LogEntry::new(time, log)));
}

pub fn broadcast(packet: OutPacket) {
    //info!("Broadcast: {:?}", Debug2Format(&packet));
    BROADCAST_CHANNEL.publish_immediate(packet);
}

pub fn send_packet(packet: IoPacket<OutPacket>) {
    info!("{:?}", Debug2Format(&packet));
    OUT_CHANNEL.publish_immediate(packet);
}

pub async fn receive_packet() -> IoPacket<InPacket> {
    INTERNAL_CHANNEL.receive().await
}

//Receives only InPackets coming from LoRa
pub fn try_receive_packet() -> Result<IoPacket<InPacket>, TryReceiveError> {
    let packet = INTERNAL_CHANNEL.try_receive();
    //info!("Packet try_received: {:?}", Debug2Format(&packet));
    packet
}

fn received_packet(mut packet: IoPacket<InPacket>) {
    while let Err(err) = INTERNAL_CHANNEL.try_send(packet) {
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

pub async fn next_out_packet(
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
            Either::Second(p) => {
                // TODO: I don't like how this is organized, i wish USB code could stick to its own
                // functions
                if channel == IoChannel::Usb && !USB_BROADCASTING_ENABLED.load(Ordering::Relaxed) {
                    continue;
                }

                p
            }
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
    info!("Radio task is running!");
    radio.set_mode(rfm9::Mode::Sleep).await?;
    //let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = OUT_CHANNEL.subscriber()?;
    let mut in_sub = IN_CHANNEL.sender();
    let in_receiver = IN_CHANNEL.receiver();
    let internal_sender = INTERNAL_CHANNEL.sender();

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];
    let mut buf2 = [0u8; 255];
    let mut buf3 = [0u8; 255];

    loop {
        //Send OutPackets
        let packet = out_sub.try_next_message_pure();
        match packet {
            Some(io_packet) => {
                //info!("OutPacket found!");
                if io_packet.channel == IoChannel::ToLoRa {
                    //info!("OutPacket wants to be sent via radio!");
                    let packet = io_packet.packet;

                    let radio_packet = RadioPacket::new(config, packet);

                    radio_packet.to_song(&mut buf)?;

                    radio.transmit(&buf[0..radio_packet.song_size()]).await?;
                    radio.set_mode(rfm9::Mode::Sleep).await?;
                    //info!("Outpacket sent over radio!");
                }
            }
            None => {}
        }

        //Send InPackets
        if INTERNAL_SENDING_ENABLED.load(Ordering::Relaxed){
            let packet = IN_CHANNEL.try_receive();
            match packet{
                Ok(packet) => {
                    //info!("IoPacket found!");
                    let packet: IoPacket<InPacket> = packet;
                    if packet.channel == IoChannel::ToLoRa{
                        //info!("InPacket wants to be sent via radio!");
                        let radio_packet = RadioPacket::new(config, packet.packet);
                        radio_packet.to_song(&mut buf3);
                        radio.transmit(&buf3[0..radio_packet.song_size()]).await;
                    //info!("InPacket is sent over radio!");
                    }
                },
                Err(..) => {}
            }
        }
        
        //Receive InPackets
        match radio.recieve(&mut buf2).await{
            Ok(len) => {
                //info!("Radio data received!");
                let len = len as usize;
                let radio_packet = RadioPacket::from_song(&buf2[..len]);
                if let Ok(packet) = radio_packet{
                    //info!("InPacket is received from radio!");
                    let inpacket: InPacket = packet.packet;
                    internal_sender.send(IoPacket::new(IoChannel::FromLoRa, inpacket)).await;
                    //info!("InPacket is sent to proper channel!");
                    //info!("Free capacity of InChannel: {}", INTERNAL_CHANNEL.free_capacity());
                }
            },
            Err(..) => {
                //info!("No radio packet received");
            }
        };
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

pub fn set_usb_broadcasting_enabled(bool: bool) {
    USB_BROADCASTING_ENABLED.store(bool, Ordering::Relaxed);
}

async fn usb_output_task_impl(
    usb: &mut WriteEp
) -> Result<(), SirinError> {
    let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = OUT_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    set_usb_broadcasting_enabled(true);
    loop {
        //usb.wait_enabled().await;

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
    set_usb_broadcasting_enabled(true);
    //usb.wait_enabled().await;
    usb.read(&mut buf).await?;
    let packet = InPacket::from_song(&buf)?;
    //info!("Packet received and forwarded through USB: {:?}", Debug2Format(&packet));
    received_packet(IoPacket::new(IoChannel::Usb, packet));
    //INTERNAL_CHANNEL.send(IoPacket::new(IoChannel::Usb, packet));
    //info!("Free space in Internal Channel: {}", INTERNAL_CHANNEL.capacity());
    Ok(())
}

#[task]
pub async fn flash_io_task(flash: &'static Mutex<&'static mut Flash>){
    loop {
        match flash_task_impl(flash).await {
            Ok(()) => {},
            Err(e) => {
                defmt::error!("Error in flash task: {}", Debug2Format(&e))
            }
        }
    }
}

pub async fn flash_task_impl(flash_mutex: &Mutex<&mut Flash>) -> Result<(), SirinError> {
    let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = OUT_CHANNEL.subscriber()?;

    loop {
        let packet = next_out_packet(
            &mut broadcast_sub,
            &mut out_sub,
            IoChannel::Flash
        ).await;

        if FLASH_LOGGING_ENABLED.load(Ordering::Relaxed) {
            let mut flash = flash_mutex.lock().await;
            flash.log(&packet).await.unwrap();
            drop(flash);
        }       
    }
}



/*
#[task]
fn background_task() {

}

pub trait BackgroundIo {
    fn background_task(&mut self)

    #[must_use]
    fn background(&mut self) -> Backgrounded<&'static Self> {

    }
}

pub struct Backgrounded<T> {
    data: T,
    premptor: Signal<CriticalSectionRawMutex, ()>
}

impl <T: 'static> Backgrounded<T> {
    async fn preemptible<O>(&self, fut: impl Future<Output = O>) -> O {
        select(self.premptor.wait(), fut).await
    }
}*/