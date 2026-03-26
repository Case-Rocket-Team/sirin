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
use nalgebra as na;
use na::{Matrix3, Matrix6, Vector3, UnitQuaternion, Rotation3};
use sirin_filter::converge_states;

use {defmt_rtt as _, panic_probe as _};
use sirin::{Sirin, error::SirinError, packet::SirinState, uunit::WithUnits};
//use sirin::{state::{CovarianceMatrixP, ErrorState, NominalState}};



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

    let res = main_task(sirin).await;
    info!("There was an error and the main task was restarted: {}", Debug2Format(&res.err().unwrap()));
    Timer::after_millis(200).await;
    //Sirin::reboot();
}


// est_roll=179.78671   <-- Euler method integration

#[allow(unused_variables)]
async fn main_task(sirin: &'static mut Sirin) -> Result<(), SirinError> {
    let mut i = 0;

    let mut prev_reading = Instant::now();

    let mut nominal = sirin_filter::NominalState::default();
    let mut error = sirin_filter::ErrorState::default();
    let mut cov = sirin_filter::CovarianceMatrixP::default();


    // calibration data for a sirin not in ebay
    // let free_hard_iron_bias_x = -47.573810;
    // let free_hard_iron_bias_y = 44.599533;
    // let free_hard_iron_bias_z = -169.833372;

    // let free_scale_x = 0.976603;
    // let free_scale_y = 1.056380;
    // let free_scale_z = 0.971427;
    
    // Sirin D offset calibration
    let hard_iron_bias_x = -18.269513;
    let hard_iron_bias_y = 17.129495;
    let hard_iron_bias_z = -42.261032;

    let free_soft_iron_bias_xx = 18.728720;
    let free_soft_iron_bias_xy = 1.581941;
    let free_soft_iron_bias_xz = 0.091705;

    let free_soft_iron_bias_yx = 1.581941;
    let free_soft_iron_bias_yy = 16.487833;
    let free_soft_iron_bias_yz = -0.505229;

    let free_soft_iron_bias_zx = 0.091705;
    let free_soft_iron_bias_zy = -0.505229;
    let free_soft_iron_bias_zz = 18.669202;

    let mut is_first_reading = true;

    let mut angular_test = Vector3::new(0.0 as f32, 0.0 as f32, 0.0 as f32);
    let mut angular_sum = Vector3::new(0.0 as f32,0.0 as f32,0.0 as f32);
    let mut vel_naive = Vector3::new(0.0 as f32,0.0 as f32,0.0 as f32);
    let mut pos_naive = Vector3::new(0.0 as f32,0.0 as f32,0.0 as f32);
    let mut g_naive = Vector3::new(0.0 as f32,0.0 as f32,0.0 as f32);
    loop {
        // info!("Running loop");

        let curr_reading = Instant::now();
        let dt = curr_reading.duration_since(prev_reading);

        let accel = sirin.imu.accel_autoscale().await?;
        
        let accel = Vector3::new(
            (accel.x.value as f32) / 1e6 *  9.80665,
            -(accel.y.value as f32) / 1e6 *  9.80665,
            (accel.z.value as f32) / 1e6 *  9.80665,
        );

        let angular = sirin.imu.angular_vel().await?;

        let angular = Vector3::new(
            (angular.x_pitch.value as f32) * PI / 180.0 / 1e6 * -1.0,
            (angular.y_roll.value as f32) * PI / 180.0 / 1e6,
            (angular.z_yaw.value as f32) * PI / 180.0 / 1e6 * -1.0,
        );

        let (x_raw, y_raw, z_raw) = sirin.magnetometer.magnetic().await?;
        // divide by 6842 for Gauss, mult by 100 for micro Teslas (uT)
        let mag_reading = [
            (x_raw as f32) / 6842.0 * 100.0,
            (y_raw as f32) / 6842.0 * 100.0,
            (z_raw as f32) / 6842.0 * 100.0,
        ];

        let mag_offset = Vector3::new(
            mag_reading[0] - hard_iron_bias_x,
            mag_reading[1] - hard_iron_bias_y,
            mag_reading[2] - hard_iron_bias_z,
        );


        if is_first_reading {
            is_first_reading = false;
            g_naive = accel;

            let magn = sirin.magnetometer.magnetic().await?;
            let magn = Vector3::new(
                magn.0 as f32,
                magn.1 as f32,
                magn.2 as f32,
                // (magn.0 as f32) - hard_iron_bias_x,
                // (magn.1 as f32) - hard_iron_bias_y,
                // (magn.2 as f32) - hard_iron_bias_z,
            );

            let _ = sirin_filter::init_with_imu(
                    &mut nominal,
                    &accel,
                    &angular,
                    &magn,
            );
        } else {
            sirin_filter::update_with_imu(
                &mut nominal,
                &mut error,
                &mut cov,
                dt.as_micros() as f32 / 1e6,
                &accel,
                &angular
            );
            if i%10==0 {
                sirin_filter::fuse_magnetometer(&mut nominal, &mut error, &mut cov, &mag_offset);
            }
        }

        angular_sum += angular * dt.as_micros() as f32 * 10e-6;
        vel_naive += (accel - g_naive) * dt.as_micros() as f32 * 10e-6;
        pos_naive += vel_naive * dt.as_micros() as f32 * 10e-6;

        // let mag_calibrated = [
        //     mag_offset[0] * free_soft_iron_bias_xx + mag_offset[1] * free_soft_iron_bias_yx + mag_offset[2] * free_soft_iron_bias_zx,
        //     mag_offset[0] * free_soft_iron_bias_xy + mag_offset[1] * free_soft_iron_bias_yy + mag_offset[2] * free_soft_iron_bias_zy,
        //     mag_offset[0] * free_soft_iron_bias_xz + mag_offset[1] * free_soft_iron_bias_yz + mag_offset[2] * free_soft_iron_bias_zz,
        // ];

        if i % 100 == 0 {
            info!("Nominal: {}", Debug2Format(&nominal));
            // info!("Error: {}", Debug2Format(&error));
            info!("Magnetometer: ({}, {}, {})", mag_offset.x, mag_offset.y,mag_offset.z);
            // info!("Rotation: ({}, {}, {})", angular.x, angular.y, angular.z);
            // info!("Rot sum: ({}, {}, {})", angular_sum.x, angular_sum.y, angular_sum.z);
            // info!("pos naive: ({}, {}, {})", pos_naive.x, pos_naive.y, pos_naive.z);
            // info!("accel: ({}, {}, {})", accel.x, accel.y, accel.z);
            // info!("sens: {:?}", sirin.imu.accel_sensitivity().await?);
            info!("dt: {}, Hz: {}", dt.as_micros(), 1.0/((dt.as_micros() as f32)*1e-6));
            // converge_states(&mut nominal, &mut error, &mut cov);
        }
        

        prev_reading = curr_reading;
        i += 1;
    }
}