#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin, u16};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::{debug, error, info, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::{Duration, Instant, Timer, TICK_HZ};
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::{ReadRfm9, Rfm9};
use sirin_c::update_with_imu;
use w25qx::W25Q;
use {defmt_rtt as _, panic_probe as _};
use sirin::{Radio, Sirin, error::SirinError, flash::Flash, gps::gps_task, io::{IN_CHANNEL, broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task}, packet::{GpsFix, InPacket, IoChannel, IoPacket, Log, LogEntry, MAX_OUT_PACKET_SIZE, OutPacket, PacketError, Page, RadioPacket}, song::{FromSong, SongSize, ToSong}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, MetersPerSecond2, WithUnits}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{Publisher, Subscriber}};
use sirin_shared::{mode::SirinMode, physics::approx_pressure_altitude, time::AbsoluteTimeReference};
use sirin::song::SongDiscriminant;
use embassy_usb::driver::EndpointIn;
use embassy_usb::driver::Endpoint;

unsafe fn transmute_into_static<T>(item: &mut T) -> &'static mut T {
    core::mem::transmute(item)
}

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    let mut executor = Executor::new();
    let executor: &'static mut Executor = transmute_into_static(&mut executor);
    let mut sirin = MaybeUninit::<Sirin>::uninit();
    let sirin = transmute_into_static(&mut sirin);
    executor.run(|spawner| {
        spawner.must_spawn(setup_task(spawner, sirin))
    })
}

#[task()]
async fn setup_task(spawner: Spawner, sirin: &'static mut MaybeUninit<Sirin>) {
    debug!("Begin Sirin init");

    let sirin = Sirin::init(sirin, spawner).await;

    debug!("End Sirin init");

    match main_task(sirin).await {
        Ok(()) => {
            panic!("The main task ended! (It shouldn't do that)")
        },
        Err(e) => {
            panic!("The main task ran into an error: {:?}", e)
        }
    }
}

async fn main_task(sirin: &'static mut Sirin) -> Result<(), SirinError> {
    info!("Start main");
    loop {
        let mut buf = [0; MAX_OUT_PACKET_SIZE];
        let len = match sirin.radio.recieve(&mut buf).await {
            Ok(len) => len,
            Err(e) => {
                //error!("Error in Sirin: {}", Debug2Format(&e));
                continue;
            }
        };



        let len = len as usize;
        info!("Received: {:x}", buf[..len]);
        info!("Note: this will hang if no one is connected to USB.");

        let radio_packet = RadioPacket::<OutPacket>::from_song(&buf[..len]);
        let radio_packet = match radio_packet {
            Ok(p) => {
                info!("Radio packet: {}", Debug2Format(&p));
                p
            }
            Err(e) => {
                error!("Error parsing radio packet: {}", Debug2Format(&e));
                continue;
            }
        };

        radio_packet.packet.to_song(&mut buf)?;
        let len = radio_packet.packet.song_size();

        // Need to chop it up into 64-byte sized packets (full speed device)
        let mut i = 0;
        while i < len {
            let j = (i + 64).min(len);
            sirin.usb.write_ep.wait_enabled().await;
            sirin.usb.write_ep.write(&buf[i..j]).await?;
            i = j;
        }
    }
}

