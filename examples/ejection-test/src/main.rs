#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32, f64::consts::PI, mem::{self, MaybeUninit}};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::{debug, info, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::Timer;
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::ReadRfm9;
use {defmt_rtt as _, panic_probe as _};
use sirin::{error::SirinError, flash::Flash, gps::{gps_task, GPS_FIX}, io::{broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task, FLASH_LOGGING_ENABLED, IN_CHANNEL, OUT_CHANNEL}, packet::{GpsFixType, InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page, SirinData, SirinState}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, subsystems::measure_sirin, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, Meters, MetersPerSecond2, MicroGs, WithUnits}, Radio, Sirin};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{PubSubBehavior, Publisher, Subscriber}};
use embassy_stm32::usb::{Driver, Instance};
use embassy_usb::class::cdc_acm;
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;

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

    main_task(sirin).await
}

async fn main_task(sirin: &'static mut Sirin) {
    let mut i = 5;
    loop{
        info!("{}", i);
        i -= 1;
        match i {
            0 => break,
            _ => {}
        }
        Timer::after_millis(1000).await;
    }
    IN_CHANNEL.send(IoPacket::new(IoChannel::LoRa,InPacket::DeployMain)).await;
}