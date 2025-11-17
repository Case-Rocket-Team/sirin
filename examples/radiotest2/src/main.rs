#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin, sync::atomic::Ordering, u16};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::{debug, info, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::{Duration, Instant, Ticker, Timer, TICK_HZ};
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::{ReadRfm9, Rfm9};
use w25qx::W25Q;
use {defmt_rtt as _, panic_probe as _};
use sirin::{Radio, Sirin, error::SirinError, flash::Flash, gps::{GPS_FIX, gps_task}, io::{FLASH_LOGGING_ENABLED, IN_CHANNEL, OUT_CHANNEL, broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_inpacket_receiving_enabled, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task}, packet::{GpsFixType, InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page, SirinData, SirinState}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, subsystems::measure_sirin, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, Meters, MetersPerSecond2, MicroGs, WithUnits}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{PubSubBehavior, Publisher, Subscriber}};
use sirin_shared::{mode::SirinMode, physics::approx_pressure_altitude, time::AbsoluteTimeReference};
use sirin::song::SongDiscriminant;

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
    loop{
        while let Ok(io_packet) = try_receive_packet(){
            info!("Packet received in main!");
            match io_packet.packet {
                InPacket::DeployApo => {
                    Sirin::deploy_chute_apo(&mut sirin.parachute_apo); 
                    info!("Apo deployed!");
                },
                InPacket::DeployMain => {
                    Sirin::deploy_chute_main(&mut sirin.parachute_main); 
                    info!("Main deployed!");
                },
                _ => {}
            }
        }
        Timer::after_millis(500).await;
    }
}