#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32::consts::PI, fmt::Debug, mem::{self, MaybeUninit}};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::*;
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::{Instant, Timer};
use rfm9::ReadRfm9;
use crate::test_rot::test_180deg_roll_kalman;

use {defmt_rtt as _, panic_probe as _};
use sirin::{Sirin, packet::SirinState, state::{CovarianceMatrixP, ErrorState, NominalState}, uunit::WithUnits};

unsafe fn transmute_into_static<T>(item: &mut T) -> &'static mut T {
    core::mem::transmute(item)
}

mod test_rot;

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

// est_roll=179.78671   <-- Euler method integration

#[allow(unused_variables)]
async fn main_task(sirin: &'static mut Sirin) {
    let mut i = 0;

    let mut prev_reading = Instant::now();

    let mut nominal = NominalState::default();
    let mut error = ErrorState::default();
    let mut cov = CovarianceMatrixP::default();

    {
        let accel = sirin.imu.accel().await.unwrap();
        let accel = [
            (accel.x.value as f32) / 1e6,
            (accel.y.value as f32) / 1e6,
            (accel.z.value as f32) / 1e6,
        ];

        let angular = sirin.imu.angular_vel().await.unwrap();
        let angular = [
            (angular.x_pitch.value as f32) * PI / 180.0 / 1e6,
            (angular.y_roll.value as f32) * PI / 180.0 / 1e6,
            (angular.z_yaw.value as f32) * PI / 180.0 / 1e6,
        ];
        unsafe {
            sirin_c::init_with_imu(
                &mut nominal,
                &mut error,
                &accel as *const f32,
                &angular as *const f32
            );
        }
    }

    let mut roll = 0.0;

    loop {
        // info!("Running loop");

        let curr_reading = Instant::now();
        let dt = curr_reading.duration_since(prev_reading);

        let accel = sirin.imu.accel().await.unwrap();
        let accel = [
            (accel.x.value as f32) / 1e6 *  9.81,
            (accel.y.value as f32) / 1e6 *  9.81,
            (accel.z.value as f32) / 1e6 *  9.81,
        ];

        let angular = sirin.imu.angular_vel().await.unwrap();

        //roll += (angular.x_pitch.value as f32) / 1e6 * (dt.as_millis() as f32 / 1000.0);
        //info("roll: {} deg", roll);

        let angular = [
            (angular.x_pitch.value as f32) * PI / 180.0 / 1e6,
            (angular.y_roll.value as f32) * PI / 180.0 / 1e6,
            (angular.z_yaw.value as f32) * PI / 180.0 / 1e6,
        ];

        unsafe {
            sirin_c::update_with_imu(
                &mut nominal,
                &mut error,
                &mut cov,
                (dt.as_micros() as f32 / 1e6).with_units(),
                &accel as *const f32,
                &angular as *const f32
            );
        }

        if i % 50 == 0 {
            info!("Nominal: {}", Debug2Format(&nominal));
        }

        prev_reading = curr_reading;
        i += 1;
    }
}