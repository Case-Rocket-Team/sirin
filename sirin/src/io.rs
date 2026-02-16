use core::{cell::UnsafeCell, char::MAX, future::{Future, poll_fn}, marker::PhantomData, mem::{MaybeUninit, transmute}, sync::atomic::{AtomicBool, Ordering}, task::Poll};

use defmt::{Debug2Format, error, info, println, warn};
use embassy_executor::{Spawner, task};
use embassy_futures::{join::join, select::{select, Either}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TryReceiveError, TrySendError}, pubsub::{PubSubBehavior, PubSubChannel as EmbassyPubSubChannel, Subscriber}, signal::Signal, waitqueue::AtomicWaker, watch::{Sender, Watch}};
use embassy_time::Instant;
use embassy_usb::{driver::{Endpoint, EndpointIn, EndpointOut}, UsbDevice};
use sirin_macros::{FromSong, SongSize, ToSong};
use uunit::Milliseconds;
use crate::{UnsafeSync, usb::{ReadEp, SirinUsb, WriteEp}};
use crate::sync::Mutex;
use crate::{error::SirinError, Flash, Radio};
use sirin_shared::{config::{CallsignBuf, SirinConfig}, packet::{IoChannel, Log, LogPacket, MAX_OUT_PACKET_SIZE, OutPacket, PacketId, RadioPacket, Request, RequestPacket, Response, ResponsePacket, SirinState}, song::{FromSong, FromSongError, SongSize, ToSong, ToSongError}};

type PubSubChannel<T, const SUBS: usize> = EmbassyPubSubChannel<CriticalSectionRawMutex, T, 32, SUBS, 0>;

static IO: UnsafeSync<UnsafeCell<MaybeUninit<Io>>> = UnsafeSync(UnsafeCell::new(MaybeUninit::uninit()));
static SIRIN_STATE: Watch<CriticalSectionRawMutex, SirinState, 4> = Watch::new();

static BROADCAST_CHANNEL: PubSubChannel<LogPacket, 3> = EmbassyPubSubChannel::new();
pub static REQUEST_CHANNEL: Channel<CriticalSectionRawMutex, IoRequest, 32> = Channel::new();
pub static RESPONSE_CHANNEL: PubSubChannel<IoResponse, 3> = EmbassyPubSubChannel::new();

pub struct Io {
    config: &'static SirinConfig,
    spawner: Spawner,
    pub radio: Mutex<Radio>,
    pub flash: Mutex<Flash>,
    pub usb_write: Mutex<WriteEp>,
    pub usb_read: Mutex<ReadEp>,
    state_sender: Sender<'static, CriticalSectionRawMutex, SirinState, 4>,
}

impl Io {
    pub unsafe fn new(
        config: &'static SirinConfig,
        spawner: Spawner,
        radio: Radio,
        flash: Flash,
        usb_write: WriteEp,
        usb_read: ReadEp
    ) -> &'static Io {
        let ptr = IO.0.get();

        (*ptr).write(Io {
            config,
            spawner,
            radio: Mutex::new(radio),
            flash: Mutex::new(flash),
            usb_write: Mutex::new(usb_write),
            usb_read: Mutex::new(usb_read),
            state_sender: SIRIN_STATE.sender()
        });

        let this = &(*ptr).assume_init_ref();

        spawner.must_spawn(radio_io_task(config, &this.radio));
        spawner.must_spawn(usb_input_task(&this.usb_read));
        spawner.must_spawn(usb_output_task(&this.usb_write));
        spawner.must_spawn(flash_io_task(&this.flash));

        this
    }

    pub async fn broadcast_log(&self, log: Log) {
        BROADCAST_CHANNEL.publish_immediate(LogPacket {
            time: 0,
            log
        });
    }

    pub async fn update_state(&self, state: SirinState) {
        self.state_sender.send(state)
    }

    pub fn take_request(&self) -> Option<IoRequest> {
        match REQUEST_CHANNEL.try_receive() {
            Ok(r) => Some(r),
            Err(TryReceiveError::Empty) => return None
        }
    }
}

#[derive(Debug, Clone)]
pub struct IoRequest {
    pub packet_id: PacketId,
    pub channel: IoChannel,
    pub request: Request,
}

impl IoRequest {
    pub fn reply(&self, response: Response) {
        RESPONSE_CHANNEL.publish_immediate(
            IoResponse {
                responding_to: self.packet_id,
                channel: self.channel,
                response
            }
        );
    }
}

#[derive(Debug, Clone)]
pub struct IoResponse {
    pub responding_to: PacketId,
    pub channel: IoChannel,
    pub response: Response
}

impl From<IoResponse> for ResponsePacket {
    fn from(value: IoResponse) -> Self {
        Self {
            responding_to: value.responding_to,
            response: value.response
        }
    }
}

fn handle_received_packet(mut packet: IoRequest) {
    while let Err(err) = REQUEST_CHANNEL.try_send(packet) {
        // drop the last packet in the queue
        match REQUEST_CHANNEL.try_receive() {
            Ok(p) => drop(p),
            Err(TryReceiveError::Empty) => {}
        }

        // put the packet back (it was moved in `.try_send()`)
        match err {
            TrySendError::Full(p) => packet = p
        }
    }
}

pub async fn next_out_packet(
    broadcast: &mut Subscriber<'static, CriticalSectionRawMutex, LogPacket, 32, 3, 0>,
    out: &mut Subscriber<'static, CriticalSectionRawMutex, IoResponse, 32, 3, 0>,
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
                    OutPacket::Response(p.into())
                } else {
                    continue;
                }
            },
            Either::Second(p) => {
                OutPacket::Log(p)
            }
        }
    }
}

#[task]
pub async fn radio_io_task(
    config: &'static SirinConfig,
    radio: &'static Mutex<Radio>,
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
    radio_mutex: &Mutex<Radio>,
) -> Result<(), SirinError> {
    match radio_mutex.try_lock() {
        Ok(mut radio) => radio.set_mode(rfm9::Mode::Sleep).await?,
        Err(_) => {}
    };
    
    let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = RESPONSE_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    loop {
        let packet = next_out_packet(
            &mut broadcast_sub,
            &mut out_sub,
            IoChannel::Radio
        ).await;

        let mut radio = radio_mutex.lock().await;

        let radio_packet = RadioPacket::new(config, packet);

        radio_packet.to_song(&mut buf)?;

        radio.transmit(&buf[0..radio_packet.song_size()]).await?;
        radio.set_mode(rfm9::Mode::Sleep).await?;
    }
}

#[task]
pub async fn usb_output_task(
    usb: &'static Mutex<WriteEp>
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
    usb_mutex: &Mutex<WriteEp>
) -> Result<(), SirinError> {
    let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = RESPONSE_CHANNEL.subscriber()?;

    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    loop {
        let packet = next_out_packet(
            &mut broadcast_sub,
            &mut out_sub,
            IoChannel::Usb
        ).await;

        let mut usb = usb_mutex.lock().await;
        usb.wait_enabled().await;

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
    usb: &'static Mutex<ReadEp>
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
    usb: &'static Mutex<ReadEp>
) -> Result<(), SirinError> {
    let mut buf = [0u8; MAX_OUT_PACKET_SIZE];

    let mut usb = usb.lock().await;
    usb.wait_enabled().await;
    usb.read(&mut buf).await?;
    
    let packet = RequestPacket::from_song(&buf)?;

    handle_received_packet(IoRequest {
        packet_id: packet.packet_id,
        channel: IoChannel::Usb,
        request: packet.request
    });

    Ok(())
}

#[task]
pub async fn flash_io_task(flash: &'static Mutex<Flash>){
    loop {
        match flash_task_impl(flash).await {
            Ok(()) => {},
            Err(e) => {
                defmt::error!("Error in flash task: {}", Debug2Format(&e))
            }
        }
    }
}

pub async fn flash_task_impl(flash_mutex: &Mutex<Flash>) -> Result<(), SirinError> {
    let mut broadcast_sub = BROADCAST_CHANNEL.subscriber()?;
    let mut out_sub = RESPONSE_CHANNEL.subscriber()?;

    loop {
        let packet = next_out_packet(
            &mut broadcast_sub,
            &mut out_sub,
            IoChannel::Flash
        ).await;

        let OutPacket::Log(log) = packet else {
            warn!("Flash should not be sent responses!");
            continue;
        };

        let mut flash = flash_mutex.lock().await;
        flash.log(&log).await.unwrap();
        drop(flash);
    }
}
