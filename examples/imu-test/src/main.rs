#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unreachable_code)]

use core::mem::{self, MaybeUninit};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::*;
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::Timer;
use rfm9::ReadRfm9;
use {defmt_rtt as _, panic_probe as _};
use sirin::{Radio, Sirin, error::SirinError, flash::Flash, gps::{GPS_FIX, gps_task}, io::{FLASH_LOGGING_ENABLED, REQUEST_CHANNEL, RESPONSE_CHANNEL, broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task}, packet::{GpsFixType, Request, IoChannel, IoPacket, Log, LogEntry, Log, SirinError, Page, SirinData, SirinState}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, subsystems::measure_sirin, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, Meters, MetersPerSecond2, MicroGs, Quantity, WithUnits}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{PubSubBehavior, Publisher, Subscriber}};
use sirin_shared::physics::approx_pressure_altitude;

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

    let sirin = Sirin::new(sirin, spawner).await;

    debug!("End Sirin init");

    match main_task(sirin).await {
        Ok(()) => {
            info!("The main task ended! (It shouldn't do that)")
        },
        Err(e) => {
            info!("The main task ran into an error")
        }
    }
}

async fn main_task(sirin: &'static mut Sirin)  -> Result<(), SirinError> {
    let state = SirinState::default();

    //sirin.spawner.spawn(radio_io_task(&sirin.config, &mut sirin.radio)).unwrap();
    //sirin.spawner.spawn(usb_input_task(&mut sirin.usb.read_ep)).unwrap();
    //sirin.spawner.spawn(usb_output_task(&mut sirin.usb.write_ep)).unwrap();
    //sirin.spawner.spawn(gps_task(&mut sirin.gps_rx, &mut sirin.gps_tx)).unwrap();

    let mut flash = Mutex::new(&mut sirin.flash);
    
    sirin.spawner.spawn(flash_io_task(unsafe {
        transmute_into_static(&mut flash)
    })).unwrap();

    info!("Start main");
    loop{}
    
    // println!("set sensitivity: {}", sirin.imu.set_accel_sensitivity(4).await.unwrap());
    // println!("read ctrl: {}", sirin.imu.read_reg(0x10).await.unwrap());
    
    // println!("read real sensitivity: {}", sirin.imu.accel_sensitivity().await.unwrap());
    // println!("read bits: {}", sirin.imu.test_fs().await.unwrap());
    // let this = (((0b0011_00_00 & (1 << (3 + 1))) as u8) >>2) << 0;
    // println!("{}", this);

    
    // println!("set sensitivity to 4: {}", sirin.imu.set_accel_sensitivity(0).await.unwrap());
    // println!("raw accel: {} \nadjusted accel: {}", sirin.imu.raw_accel().await.unwrap(), sirin.imu.accel().await.unwrap());
    // println!("set sensitivity to 8: {}", sirin.imu.set_accel_sensitivity(1).await.unwrap());
    // println!("raw accel: {} \nadjusted accel: {}", sirin.imu.raw_accel().await.unwrap(), sirin.imu.accel().await.unwrap());
    // println!("set sensitivity to 16: {}", sirin.imu.set_accel_sensitivity(2).await.unwrap());
    // println!("raw accel: {} \nadjusted accel: {}", sirin.imu.raw_accel().await.unwrap(), sirin.imu.accel().await.unwrap());
    // println!("set sensitivity to 32: {}", sirin.imu.set_accel_sensitivity(3).await.unwrap());
    // println!("raw accel: {} \nadjusted accel: {}", sirin.imu.raw_accel().await.unwrap(), sirin.imu.accel().await.unwrap());
    
    // println!("set sensitivity to 250: {}", sirin.imu.set_gyro_sensitivity(0).await.unwrap());
    // println!("raw gyro: {}\nadjusted: {}", sirin.imu.raw_gyro().await.unwrap(), sirin.imu.gyro().await.unwrap());
    // println!("set sensitivity to 500: {}", sirin.imu.set_gyro_sensitivity(1).await.unwrap());
    // println!("raw gyro: {}\nadjusted: {}", sirin.imu.raw_gyro().await.unwrap(), sirin.imu.gyro().await.unwrap());
    // println!("set sensitivity to 1000: {}", sirin.imu.set_gyro_sensitivity(2).await.unwrap());
    // println!("raw gyro: {}\nadjusted: {}", sirin.imu.raw_gyro().await.unwrap(), sirin.imu.gyro().await.unwrap());
    // println!("set sensitivity to 2000: {}", sirin.imu.set_gyro_sensitivity(3).await.unwrap());
    // println!("raw gyro: {}\nadjusted: {}", sirin.imu.raw_gyro().await.unwrap(), sirin.imu.gyro().await.unwrap());

    loop {
        // println!("raw accel: {} \nadjusted accel: {}", sirin.imu.raw_accel().await.unwrap(), sirin.imu.accel_autoscale().await.unwrap());
        // println!("gyro: {}\n adjusted: {}", sirin.imu.raw_gyro().await.unwrap(), sirin.imu.gyro().await.unwrap());

    }
}