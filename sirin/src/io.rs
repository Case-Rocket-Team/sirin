use core::{char::MAX, future::{Future, poll_fn}, marker::PhantomData, mem::transmute, sync::atomic::{AtomicBool, AtomicU32, Ordering}, task::Poll};

use defmt::{error, info, println, warn, Debug2Format};
use embassy_executor::task;
use embassy_futures::{join::join, select::{select, select3, Either, Either3}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TryReceiveError, TrySendError}, pubsub::{PubSubBehavior, PubSubChannel as EmbassyPubSubChannel, Subscriber}, signal::Signal, waitqueue::AtomicWaker};
use embassy_time::{with_timeout, Duration, Instant};
use embassy_usb::{driver::{Endpoint, EndpointIn, EndpointOut}, UsbDevice};
use sirin_macros::{FromSong, SongSize, ToSong};
use uunit::Milliseconds;
use crate::{sync::Mutex, usb::{ReadEp, SirinUsb, WriteEp}};

use crate::{error::SirinError, Flash, Radio};
use sirin_shared::{config::{CallsignBuf, SirinConfig}, packet::{EkfDebug, InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, RadioPacket, MAX_OUT_PACKET_SIZE}, song::{FromSong, FromSongError, SongSize, ToSong, ToSongError}};

//pub static OUT_CHANNEL: Channel<CriticalSectionRawMutex, OutPacket, 10> = Channel::new();

type PubSubChannel<T, const SUBS: usize> = EmbassyPubSubChannel<CriticalSectionRawMutex, T, 32, SUBS, 0>;

// Lower priority general broadcast channel.
pub static BROADCAST_CHANNEL: PubSubChannel<OutPacket, 3> = EmbassyPubSubChannel::new();

// High priority I/O channels
pub static OUT_CHANNEL: PubSubChannel<IoPacket<OutPacket>, 3> = EmbassyPubSubChannel::new();
pub static IN_CHANNEL: Channel<CriticalSectionRawMutex, IoPacket<InPacket>, 32> = Channel::new();

// Latest EKF debug snapshot, sent over USB only. Kept out of OUT_CHANNEL so its
// high rate can't push out radio or flash packets; only the newest is kept.
pub static EKF_DEBUG: Signal<CriticalSectionRawMutex, EkfDebug> = Signal::new();

// Flash log queue. Isolated so that radio and USB do not fill the same buffer and drop packets
const FLASH_QUEUE_LEN: usize = 64;
static FLASH_CHANNEL: Channel<CriticalSectionRawMutex, OutPacket, FLASH_QUEUE_LEN> = Channel::new();
static FLASH_DROPPED: AtomicU32 = AtomicU32::new(0);

// USB replies that must not be dropped
static USB_REPLY_CHANNEL: Channel<CriticalSectionRawMutex, OutPacket, 4> = Channel::new();
// Give up on an export if the host stops reading for this long
const USB_REPLY_TIMEOUT: Duration = Duration::from_secs(2);

static USB_BROADCASTING_ENABLED: AtomicBool = AtomicBool::new(false);
pub static FLASH_LOGGING_ENABLED: AtomicBool = AtomicBool::new(false);


pub fn set_flash_logging_enabled(bool: bool) {
    FLASH_LOGGING_ENABLED.store(bool, Ordering::Relaxed);
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

/// Queue a packet for the flash log without blocking. Dropped when logging is off.
/// Dropped and counted when the queue is full.
pub fn log_to_flash(packet: OutPacket) {
    if !FLASH_LOGGING_ENABLED.load(Ordering::Relaxed) {
        return;
    }
    if FLASH_CHANNEL.try_send(packet).is_err() {
        let dropped = FLASH_DROPPED.fetch_add(1, Ordering::Relaxed) + 1;
        // Warn on 1, 2, 4, 8... so a long stall doesn't flood the log
        if dropped.is_power_of_two() {
            warn!("Flash log queue full, {} entries dropped so far", dropped);
        }
    }
}

/// Send a reply that must arrive. USB replies wait for room in the USB queue and other
/// channels fall back to `send_packet`. Returns false if the host stopped reading.
pub async fn send_packet_reliable(packet: IoPacket<OutPacket>) -> bool {
    if packet.channel != IoChannel::Usb {
        send_packet(packet);
        return true;
    }
    with_timeout(USB_REPLY_TIMEOUT, USB_REPLY_CHANNEL.send(packet.packet)).await.is_ok()
}

pub async fn receive_packet() -> IoPacket<InPacket> {
    IN_CHANNEL.receive().await
}

//Receives only InPackets coming from LoRa
pub fn try_receive_packet() -> Result<IoPacket<InPacket>, TryReceiveError> {
    let packet = IN_CHANNEL.try_receive();
    //info!("Packet try_: {:?}", Debug2Format(&packet));
    packet
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
    //info!("Radio task is running!");
    radio.set_mode(rfm9::Mode::Sleep).await?;
    let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = OUT_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    loop {
        //Send OutPackets
        let packet = next_out_packet(
            &mut broadcast_sub,
            &mut out_sub,
            IoChannel::ToLoRa
        ).await;
        //info!("Message slots left: {}", broadcast_sub.free_capacity());

        let radio_packet = RadioPacket::new(config, packet);

        radio_packet.to_song(&mut buf)?;

        radio.transmit(&buf[0..radio_packet.song_size()]).await?;
        //println!("OUT_CHANNEL free capacity: {}", OUT_CHANNEL.free_capacity());
        radio.set_mode(rfm9::Mode::Sleep).await?;
        
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

    loop {
        usb.wait_enabled().await;

        // Replies first so an export isn't held up by live data
        let packet = match select3(
            USB_REPLY_CHANNEL.receive(),
            next_out_packet(&mut broadcast_sub, &mut out_sub, IoChannel::Usb),
            EKF_DEBUG.wait()
        ).await {
            Either3::First(p) => p,
            Either3::Second(p) => p,
            Either3::Third(d) => OutPacket::EkfDebug(d),
        };

        packet.to_song(&mut buf)?;
        let len = packet.song_size();

        // Need to chop it up into 64-byte sized packets (full speed device)
        let mut i = 0;
        while i < len {
            let j = (i + 64).min(len);
            usb.write(&buf[i..j]).await?;
            i = j;
        }

        // The host ends a read on a short chunk. A packet that fills its last chunk
        // needs an empty one, or the host merges it with the next packet.
        if len % 64 == 0 {
            usb.write(&[]).await?;
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
    usb.wait_enabled().await;
    usb.read(&mut buf).await?;
    let packet = InPacket::from_song(&buf)?;
    received_packet(IoPacket::new(IoChannel::Usb, packet));
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

    loop {
        let packet = match select(
            FLASH_CHANNEL.receive(),
            broadcast_sub.next_message_pure()
        ).await {
            // Already checked against FLASH_LOGGING_ENABLED when queued, so entries
            // queued just before logging is turned off still get written
            Either::First(p) => p,
            Either::Second(p) => {
                if !FLASH_LOGGING_ENABLED.load(Ordering::Relaxed) {
                    continue;
                }
                p
            }
        };

        let mut flash = flash_mutex.lock().await;
        flash.log(&packet).await.unwrap();
        drop(flash);
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