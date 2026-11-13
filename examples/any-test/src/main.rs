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
use rfm9::{ReadRfm9, Rfm9};
use {defmt_rtt as _, panic_probe as _};
use sirin::{Radio, Sirin, error::SirinError, flash::Flash, gps::{GPS_FIX, gps_task}, io::{FLASH_LOGGING_ENABLED, IN_CHANNEL, OUT_CHANNEL, broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_flash_logging_enabled, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task}, packet::{GpsFixType, InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page, SirinData, SirinState, TrackMe}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, subsystems::measure_sirin, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, Meters, MetersPerSecond2, MicroGs, WithUnits}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{PubSubBehavior, Publisher, Subscriber}};
use sirin_shared::{mode::SirinMode, physics::approx_pressure_altitude, time::AbsoluteTimeReference};
use sirin::song::SongDiscriminant;
use micromath::F32Ext;

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
    /*

    Use this file for any test you may have. I was tired of making a new file for every time I wanted to test something so small.

    CURRENT TEST: Arlo's Drop Test
    PURPOSE: Logging acceleration to measure parachute performance

    Set RUN_ACCEL_CALIBRATION to true to run the accelerometer six-position
    calibration instead (finds the offsets and scales for each axis).

     */

    const RUN_ACCEL_CALIBRATION: bool = true;
    if RUN_ACCEL_CALIBRATION {
        accel_calibration(sirin).await;
        loop {
            Timer::after_millis(1000).await;
        }
    }
    
    loop{
        Timer::after_millis(2000).await;

        let mut state = SirinState::default();

        if let Some(fix) = GPS_FIX.try_take() {
            //if fix.fix_type != GpsFixType::NoFix {
                state.gps_fix = fix;
            //}
        }

        let track_me: TrackMe = TrackMe{
            lat: state.gps_fix.lat,
            lon: state.gps_fix.lon
        };

        OUT_CHANNEL.publish_immediate(IoPacket::new(
                IoChannel::ToLoRa, OutPacket::LogEntry(LogEntry::new(
                    sirin.data.time,
                    Log::TrackMe(track_me)
                ))
            ));
    }
}

/// Accelerometer six-position test.
///
/// For each axis the board is held still pointing straight up, then straight
/// down. Gravity is exactly 1 g, so each axis gets two known reference points:
///     offset = (up + down) / 2
///     scale  = 2 / (up - down)
/// Rest the board against something square and level (table + box edge),
/// mounted the way it will fly. Don't hand-hold it.
async fn accel_calibration(sirin: &mut Sirin) {
    // (axis index, pointing up?, instruction)
    const POSITIONS: [(usize, bool, &str); 6] = [
        (0, true, "+X pointing UP"),
        (0, false, "+X pointing DOWN"),
        (1, true, "+Y pointing UP"),
        (1, false, "+Y pointing DOWN"),
        (2, true, "+Z pointing UP"),
        (2, false, "+Z pointing DOWN"),
    ];
    const AXIS_NAMES: [&str; 3] = ["X", "Y", "Z"];
    // Time to move the board before each position, in seconds
    const SETTLE_S: u32 = 8;
    // Samples per position, every 10 ms (500 = 5 s)
    const SAMPLES: u32 = 500;
    // Redo a position if the axis being measured wobbles more than this (g, std dev)
    const MAX_STD_G: f32 = 0.01;

    // up_g[axis] / down_g[axis]: averaged reading of that axis, in g
    let mut up_g = [0.0f32; 3];
    let mut down_g = [0.0f32; 3];
    // Full averaged vector for each position, for the check at the end
    let mut position_means = [[0.0f32; 3]; 6];

    info!("==== Accelerometer six-position calibration ====");

    for (p, &(axis, up, instruction)) in POSITIONS.iter().enumerate() {
        loop {
            info!("Position {}/6: place the board with {}", p + 1, instruction);
            for s in (1..=SETTLE_S).rev() {
                info!("  measuring in {}...", s);
                Timer::after_millis(1000).await;
            }
            info!("  measuring - keep it still");

            // Running sums for mean and variance, in g
            let mut sum = [0.0f32; 3];
            let mut sum_sq = [0.0f32; 3];
            let mut n: u32 = 0;
            let mut ticker = Ticker::every(Duration::from_millis(10));
            for _ in 0..SAMPLES {
                ticker.next().await;
                let Ok(a) = sirin.imu.accel().await else { continue };
                let g = [
                    a.x.value as f32 * 1e-6,
                    a.y.value as f32 * 1e-6,
                    a.z.value as f32 * 1e-6,
                ];
                for i in 0..3 {
                    sum[i] += g[i];
                    sum_sq[i] += g[i] * g[i];
                }
                n += 1;
            }

            if n < SAMPLES / 2 {
                info!("  too many failed IMU reads ({} of {}), redoing this position", SAMPLES - n, SAMPLES);
                continue;
            }

            let nf = n as f32;
            let mean = [sum[0] / nf, sum[1] / nf, sum[2] / nf];
            let var = (sum_sq[axis] / nf - mean[axis] * mean[axis]).max(0.0);
            info!(
                "  mean: x={}g y={}g z={}g ({} samples)",
                mean[0], mean[1], mean[2], n,
            );

            // Compare variance against MAX_STD_G² so no square root is needed
            if var > MAX_STD_G * MAX_STD_G {
                info!("  board moved during the measurement, redoing this position");
                continue;
            }
            // The measured axis should read roughly ±1 g; if not, it's probably
            // in the wrong orientation.
            if (mean[axis].abs() - 1.0).abs() > 0.5 {
                info!(
                    "  {} reads {}g, expected about {}1 g - check the orientation, redoing",
                    AXIS_NAMES[axis], mean[axis], if up { "+" } else { "-" },
                );
                continue;
            }

            if up {
                up_g[axis] = mean[axis];
            } else {
                down_g[axis] = mean[axis];
            }
            position_means[p] = mean;
            break;
        }
    }

    let mut offset = [0.0f32; 3];
    let mut scale = [1.0f32; 3];
    info!("==== Results ====");
    for axis in 0..3 {
        offset[axis] = (up_g[axis] + down_g[axis]) / 2.0;
        scale[axis] = 2.0 / (up_g[axis] - down_g[axis]);
        info!(
            "{}: up={}g down={}g -> offset={}g scale={}",
            AXIS_NAMES[axis], up_g[axis], down_g[axis], offset[axis], scale[axis],
        );
    }

    // Check: apply the calibration to every position. Each should now read
    // 1.00 g total. Big misses mean a position wasn't level.
    info!("Check (should all be about 1.00 g):");
    for (p, mean) in position_means.iter().enumerate() {
        let c = [
            (mean[0] - offset[0]) * scale[0],
            (mean[1] - offset[1]) * scale[1],
            (mean[2] - offset[2]) * scale[2],
        ];
        let magnitude = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        info!("  {}: |a| = {}g", POSITIONS[p].2, magnitude);
    }

    info!("const ACCEL_OFFSET_G: [f32; 3] = [{}, {}, {}];", offset[0], offset[1], offset[2]);
    info!("const ACCEL_SCALE: [f32; 3] = [{}, {}, {}];", scale[0], scale[1], scale[2]);
}
