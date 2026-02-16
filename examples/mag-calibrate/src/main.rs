#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::mem::{self, MaybeUninit};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use cortex_m::interrupt::free;
use defmt::*;
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::Timer;
use rfm9::ReadRfm9;
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

    let sirin = Sirin::new(sirin, spawner).await;

    debug!("End Sirin init");

    main_task(sirin).await
}

#[allow(unused_variables)]
async fn main_task(sirin: &'static mut Sirin) {
    let mut i = 0;
    // calibration data for a sirin not in ebay
    let free_hard_iron_bias_x = -42.960563;
    let free_hard_iron_bias_y = 48.877747;
    let free_hard_iron_bias_z = -166.783874;

    let free_soft_iron_bias_xx = 23.226423;
    let free_soft_iron_bias_xy = 0.593897;
    let free_soft_iron_bias_xz = 0.491833;

    let free_soft_iron_bias_yx = 0.593897;
    let free_soft_iron_bias_yy = 21.800691;
    let free_soft_iron_bias_yz = 0.239927;

    let free_soft_iron_bias_zx = 0.491833;
    let free_soft_iron_bias_zy = 0.239927;
    let free_soft_iron_bias_zz = 16.148427;

    loop {
        let (x_raw, y_raw, z_raw) = sirin.magnetometer.magnetic().await.unwrap();
        // divide by 6842 for Gauss, mult by 100 for micro Teslas (uT)
        let mag_reading = [
            (x_raw as f32) / 6842.0 * 100.0,
            (y_raw as f32) / 6842.0 * 100.0,
            (z_raw as f32) / 6842.0 * 100.0,
        ];

        let mag_offset = [
            mag_reading[0] - free_hard_iron_bias_x,
            mag_reading[1] - free_hard_iron_bias_y,
            mag_reading[2] - free_hard_iron_bias_z,
        ];

        let mag_calibrated = [
            mag_offset[0] * free_soft_iron_bias_xx + mag_offset[1] * free_soft_iron_bias_yx + mag_offset[2] * free_soft_iron_bias_zx,
            mag_offset[0] * free_soft_iron_bias_xy + mag_offset[1] * free_soft_iron_bias_yy + mag_offset[2] * free_soft_iron_bias_zy,
            mag_offset[0] * free_soft_iron_bias_xz + mag_offset[1] * free_soft_iron_bias_yz + mag_offset[2] * free_soft_iron_bias_zz,
        ];

        // when calibrating, set to mag_reading
        if i % 100 == 0 {
            info!("Magnetometer: {:?}", mag_reading);
        }
        i += 1;
    }
}