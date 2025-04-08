#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::mem::{self, MaybeUninit};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::*;
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::Timer;
use rfm9x::ReadRfm9x;
use {defmt_rtt as _, panic_probe as _};
use sirin::Sirin;

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

#[allow(unused_variables)]
async fn main_task(sirin: &'static mut Sirin) {
    // loop {
    //     println!("{} {} {} {} {} {}", 
    //     sirin.high_g_imu.acceleration().await.unwrap().0,
    //     sirin.high_g_imu.acceleration().await.unwrap().1,
    //     sirin.high_g_imu.acceleration().await.unwrap().2,  
    //     sirin.imu.accel().await.unwrap().0,
    //     sirin.imu.accel().await.unwrap().1,
    //     sirin.imu.accel().await.unwrap().2,);
        
    //     Timer::after_millis(10).await;
    // }
}