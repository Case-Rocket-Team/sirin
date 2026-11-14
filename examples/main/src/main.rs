#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin, sync::atomic::Ordering, u16};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use lsm6dsv_spi::{SflpOdr};
use defmt::{debug, info, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::{Duration, Instant, Ticker, Timer, TICK_HZ};
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::{ReadRfm9, Rfm9};
use w25qx::W25Q;
use {defmt_rtt as _, panic_probe as _};
use sirin::{Radio, Sirin, error::SirinError, flash::Flash, gps::{GPS_FIX, gps_task}, io::{FLASH_LOGGING_ENABLED, IN_CHANNEL, OUT_CHANNEL, broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_flash_logging_enabled, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task}, packet::{SirinDataState, GpsFixType, InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page, SirinData, SirinState}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, subsystems::measure_sirin, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, Meters, MetersPerSecond2, MicroGs, WithUnits}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{PubSubBehavior, Publisher, Subscriber}};
use sirin_shared::{mode::SirinMode, physics::approx_pressure_altitude, time::AbsoluteTimeReference};
use sirin::song::SongDiscriminant;
use sirin::packet::MatchVoltage;
use nalgebra as na;
use na::{Matrix3, Matrix6, Vector3, UnitQuaternion, Rotation3, ComplexField};
use extended_kalman::{Ekf, Ekf_settings};
mod config;

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
    //println!("Time since epoch: {}", duration_since_epoch().unwrap());
    
    // Flight parameters live in config.rs
    let accel_threshold: Gs<f64> = config::ACCEL_THRESHOLD_GS_SQUARED.with_units(); 
    let altitude_threshold = config::ALTITUDE_THRESHOLD_M; 
    let main_deployment_altitude = config::MAIN_DEPLOYMENT_ALTITUDE_M; 
    let flight_duration = config::FLIGHT_DURATION_S; 
    let apogee_error = config::APOGEE_ERROR_M; 
    let timeout = config::APOGEE_TIMEOUT_S; 

    let mut apo_deployed = false;
    let mut main_deployed = false;
    

    let mut state = SirinState::default();

    //For debugging purposes
    FLASH_LOGGING_ENABLED.store(false, Ordering::Relaxed);

    //Find the true barometric reading (bug causes large negative values upon initialization)
    let mut altitude_array: [f64; 100] = [0.0; 100];
    for i in 0..100 {
        let altitude = approx_pressure_altitude(sirin.baro.read().await?.pressure.convert());
        altitude_array[i] = altitude.value;
        Timer::after_millis(25).await;
    }
    altitude_array.sort_unstable_by(|a, b | a.partial_cmp(b).unwrap());
    let initial_altitude: Meters<f64> = altitude_array[50].with_units();
    info!("Initial altitude: {}", initial_altitude.value);

    //Spawn background tasks
    sirin.spawner.spawn(radio_io_task(&sirin.config, &mut sirin.radio)).unwrap();
    sirin.spawner.spawn(usb_input_task(&mut sirin.usb.read_ep)).unwrap();
    sirin.spawner.spawn(usb_output_task(&mut sirin.usb.write_ep)).unwrap();
    sirin.spawner.spawn(gps_task(&mut sirin.gps_rx, &mut sirin.gps_tx)).unwrap();
    
    let mut flash = Mutex::new(&mut sirin.flash);
    sirin.spawner.spawn(flash_io_task(unsafe {
        transmute_into_static(&mut flash)
    })).unwrap();

    info!("Keep Sirin still while blue light is on.");
    for i in 0..5{
        if i == 5{
            break;
        }
        sirin.led_blue.set_high();
        Timer::after_millis(100).await;
        sirin.led_blue.set_low();
        Timer::after_millis(300).await;
    }
    sirin.led_blue.set_high();

    // Checking for gravity vector and gyro bias while the board is still
    let (gravity, gyro_bias): (Accel, AngularVel) = {
        const SAMPLES: u32 = 400;
        // Skip a sample if any gyro axis exceeds this
        const STILL_GYRO_UDPS: i64 = 3_000_000;
        // Start over if any axis varies more than this
        const MAX_STD_UG: f64 = 20_000.0;
        // Start over if any gyro axis varies more than this (1 dps)
        const MAX_GYRO_STD_UDPS: f64 = 1_000_000.0;

        let mut ticker = Ticker::every(Duration::from_millis(5));
        loop {
            let mut sum = [0i64; 3];
            let mut sum_sq = [0f64; 3];
            let mut gyro_sum = [0i64; 3];
            let mut gyro_sum_sq = [0f64; 3];
            let mut n: u32 = 0;
            for _ in 0..SAMPLES {
                ticker.next().await;
                let (Ok(a), Ok(w)) = (sirin.imu.accel().await, sirin.imu.angular_vel().await) else {
                    continue;
                };
                if [w.x_pitch.value, w.y_roll.value, w.z_yaw.value]
                    .iter()
                    .any(|v| v.abs() > STILL_GYRO_UDPS)
                {
                    continue;
                }
                for (i, v) in [a.x.value, a.y.value, a.z.value].into_iter().enumerate() {
                    sum[i] += v as i64;
                    sum_sq[i] += (v as f64) * (v as f64);
                }
                for (i, v) in [w.x_pitch.value, w.y_roll.value, w.z_yaw.value].into_iter().enumerate() {
                    gyro_sum[i] += v;
                    gyro_sum_sq[i] += (v as f64) * (v as f64);
                }
                n += 1;
            }

            if n < SAMPLES / 2 {
                info!("Gravity: board moving or IMU reads failing ({}/{} samples kept), retrying", n, SAMPLES);
                continue;
            }
            let nf = n as f64;
            let mean_ug = [sum[0] as f64 / nf, sum[1] as f64 / nf, sum[2] as f64 / nf];
            let moved = (0..3).any(|i| sum_sq[i] / nf - mean_ug[i] * mean_ug[i] > MAX_STD_UG * MAX_STD_UG);
            if moved {
                info!("Gravity: board moved during measurement, retrying");
                continue;
            }
            let gyro_mean_udps = [gyro_sum[0] as f64 / nf, gyro_sum[1] as f64 / nf, gyro_sum[2] as f64 / nf];
            let gyro_std_udps = [0, 1, 2].map(|i| {
                ComplexField::sqrt((gyro_sum_sq[i] / nf - gyro_mean_udps[i] * gyro_mean_udps[i]).max(0.0))
            });
            if gyro_std_udps.iter().any(|&s| s > MAX_GYRO_STD_UDPS) {
                info!(
                    "Gyro bias: board turned during measurement (std x={} y={} z={} µdps), retrying",
                    gyro_std_udps[0], gyro_std_udps[1], gyro_std_udps[2],
                );
                continue;
            }

            let ms2 = |axis: usize| {
                ((mean_ug[axis] * 1e-6) as f32) * 9.80665
            };
            let g = Accel {
                x: ms2(0).with_units(),
                y: ms2(1).with_units(),
                z: ms2(2).with_units(),
            };
            info!(
                "Gravity ({} samples): x={} y={} z={} m/s²",
                n, g.x.value, g.y.value, g.z.value,
            );

            let rad_s = |axis: usize| ((gyro_mean_udps[axis] * 1e-6) as f32) * (PI / 180.0);
            let bias = AngularVel {
                x_pitch: rad_s(0).with_units(),
                y_roll: rad_s(1).with_units(),
                z_yaw: rad_s(2).with_units(),
            };
            info!(
                "Gyro bias ({} samples): x={} y={} z={} µdps",
                n, gyro_mean_udps[0], gyro_mean_udps[1], gyro_mean_udps[2],
            );
            break (g, bias);
        }
    };

    // Checking for an initial magnetic heading while the board is still
    let mag_reference: sirin::packet::MagnetometerData = {
        const SAMPLES: u32 = 30;
        // Start over if any axis varies more than this
        const MAX_STD_COUNTS: f64 = 150.0;

        let mut ticker = Ticker::every(Duration::from_millis(100));
        loop {
            let mut sum = [0i64; 3];
            let mut sum_sq = [0f64; 3];
            let mut n: u32 = 0;
            for _ in 0..SAMPLES {
                ticker.next().await;
                let Ok((x, y, z)) = sirin.magnetometer.magnetic().await else {
                    continue;
                };
                for (i, v) in [x, y, z].into_iter().enumerate() {
                    sum[i] += v as i64;
                    sum_sq[i] += (v as f64) * (v as f64);
                }
                n += 1;
            }

            if n < SAMPLES / 2 {
                info!("Magnetometer: reads failing ({}/{} samples kept), retrying", n, SAMPLES);
                continue;
            }
            let nf = n as f64;
            let mean = [sum[0] as f64 / nf, sum[1] as f64 / nf, sum[2] as f64 / nf];
            let moved = (0..3).any(|i| sum_sq[i] / nf - mean[i] * mean[i] > MAX_STD_COUNTS * MAX_STD_COUNTS);
            if moved {
                info!("Magnetometer: board moved during measurement, retrying");
                continue;
            }
            let m = sirin::packet::Vec3 {
                x: mean[0].round() as i16,
                y: mean[1].round() as i16,
                z: mean[2].round() as i16,
            };
            info!(
                "Magnetometer reference ({} samples): x={} y={} z={} counts",
                n, m.x, m.y, m.z,
            );
            break sirin::packet::MagnetometerData { mag: Ok(m), temp: Ok(0) };
        }
    };

    sirin.led_blue.set_low();


    info!("Start main");

    let mut ticker = Ticker::every(Duration::from_millis((1000.0 / config::LOOP_RATE_HZ) as u64));

    let mut launched_at = None;
    let mut dur: Option<Duration> = None;
    let mut max_altitude: Meters<f64> = 0.0.with_units();
    let mut desired_mode = None;
    let mut ticks_since_broadcast: u32 = 0;
    let mut usb_ticks_since_broadcast: u32 = 0;
    let mut apogee_check = ThresholdWindow::<{ config::THRESHOLD_WINDOW }>::new();
    let mut landed_check = ThresholdWindow::<{ config::THRESHOLD_WINDOW }>::new();


    // Kalman filter init
    let ekf_settings = Ekf_settings {
        accel_var_f2ps4: 0.011196358441216, // σ = 0.1058 ft/s² (≈ 3.3 mg)
        accel_bias_var_f2ps4: 1e-4,
        gyro_var_rad2ps2: 1.09662e-6,
        gyro_bias_var_rad2ps2: 1e-5,        // σ ≈ 0.00105 rad/s (≈ 0.06 °/s)
        baro_var_ft2: 100.0,                // σ = 10 ft (≈ 3 m
        magn_covar_gauss2: [                // σ = 3.2 mG (x, y), 4.1 mG (z)
            3.2e-3 * 3.2e-3, 0.0, 0.0,
            0.0, 3.2e-3 * 3.2e-3, 0.0,
            0.0, 0.0, 4.1e-3 * 4.1e-3,
        ],
        magn_bias_var_gauss2: 1e-5,
        gnss_covar_ft2: [                   // unused while you don't inject GPS
            18.0, 0.0, 0.0,
            0.0, 18.0, 0.0,
            0.0, 0.0, 200.0,
        ],
    };

    let t_us = Instant::now().as_micros() as u32;
    let mut mag_inject_counter = 0;

    let mut filter = Ekf::new(&ekf_settings, &gravity, &gyro_bias, &mag_reference, t_us).unwrap();

    loop {
        //True only once velocity is successfully calculated
        let mut velocity_valid = false;

        //info!("Handle input packets");
        while let Ok(io_packet) = try_receive_packet() {
            info!("Received packet: {:?}", Debug2Format(&io_packet));
            match io_packet.packet {
                InPacket::DeployMain => {
                    sirin.parachute_main.set_high();
                }
                InPacket::DeployApo => {
                    sirin.parachute_apo.set_high();
                }
                InPacket::Null => {
                    continue;
                },
                InPacket::Ping => {}
                InPacket::SetTime(ref reference) => {
                    info!("SetTime packet received!");
                    if duration_since_epoch().is_some() {
                        continue;
                    }

                    // Subtract current uptime from time since boot
                    let ms_since_epoch = reference.ms_since_epoch - Instant::now().as_millis();

                    set_duration_since_epoch(Duration::from_millis(ms_since_epoch));
                    //set_flash_logging_enabled(true);
                    info!("Awaiting flash lock...");
                    let mut flash = flash.lock().await;
                    info!("Flash locked in main");
                    flash.set_absolute_time_reference(AbsoluteTimeReference { ms_since_epoch }).await?;
                    info!("Flash task complete");
                    //set_flash_logging_enabled(false);

                    // skip OK packet
                    continue;
                }
                InPacket::Reboot => {
                    Sirin::reboot();
                }
                InPacket::QueryConfig => {
                    send_packet(IoPacket::new(
                        io_packet.channel,
                        OutPacket::Config(sirin.config.clone())
                    ));
                }
                InPacket::SetConfig(ref config) => {
                    //info!("Updating the config to {:?}", Debug2Format(&config));

                    flash.lock().await.save_config(&config).await.unwrap();
                    Sirin::reboot();
                }
                InPacket::QueryMode => {
                    send_packet(io_packet.reply(OutPacket::Mode(state.mode)));
                }
                InPacket::SetMode(m) => {
                    desired_mode = Some(m);
                }
                InPacket::QueryFlights => {
                    //info!("QueryFlights packet received!");
                    let flash = flash.lock().await;

                    //info!("Querying flights...");

                    for (i, header) in flash.flight_headers.iter().enumerate() {
                        send_packet(io_packet.reply(OutPacket::FlightHeader(
                            Page::new(i as u16, header.header.clone())
                        )));
                    }
                }
                InPacket::ReadFlight(index) => {
                    let mut flash = flash.lock().await;
                    let Some(header) = flash.flight_headers.get(index as usize) else {
                        //TODO: Fix error
                        //send_packet(io_packet.reply(OutPacket::Error(PacketError::FlightNotFound(index))));
                        continue;
                    };

                    //info!("Reading flight with header: {:?}", Debug2Format(&header));

                    // borrow checker :(
                    let header = header.clone();

                    let mut iter = flash.read_logs(&header);
                    while let Some(log) = iter.next().await {
                        //info!("Sent log: {}", Debug2Format(&log));
                        send_packet(io_packet.reply(log?));
                    }

                    //info!("Done writing logs.")
                }
                InPacket::Tail(enabled) => {
                    set_usb_broadcasting_enabled(enabled);
                    continue;
                }
                InPacket::EraseFlash(..) => {
                    let mut flash = flash.lock().await;
                    info!("Starting chip erase...");
                    flash.w25q.chip_erase().await?;
                    embassy_time::Timer::after_millis(50).await;
                    info!("Waiting until flash is ready...");
                    flash.w25q.until_ready().await?;
                    info!("Finished chip erase.");

                    send_packet(io_packet.reply(OutPacket::Ok));

                    Timer::after_millis(500).await;

                    Sirin::reboot();
                }
            }

            send_packet(io_packet.reply(OutPacket::Ok));
        }

        ticker.next().await;

        //info!("Measure Sirin data");
        sirin.data = measure_sirin(
            &mut sirin.baro,
            &mut sirin.imu,
            &mut sirin.high_g_imu,
            &mut sirin.magnetometer
        ).await;
        //Add a Kalman filter function here
        filter.inject_imu(&sirin.data.imu, Instant::now().as_micros() as u32);

        if let Some(fix) = GPS_FIX.try_take() {
            if fix.fix_type != GpsFixType::NoFix {
                state.gps_fix = fix;
                filter.inject_gnss(
                    &Pos { 
                        x: (state.gps_fix.pos.x.value as f32).with_units(), 
                        y: (state.gps_fix.pos.y.value as f32).with_units(),
                        z: (state.gps_fix.pos.z.value as f32).with_units()
                    },
                    Instant::now().as_micros() as u32
                );
            }
        } 

        mag_inject_counter += 1;
        if(mag_inject_counter % 10 == 0){
            filter.inject_magn(&sirin.data.magnetometer, Instant::now().as_micros() as u32);   
        }

        if let Ok(pressure) = sirin.data.baro.pressure {
            filter.inject_baro(approx_pressure_altitude(pressure.convert()) - initial_altitude, Instant::now().as_micros() as u32);
        }

        //Calculate DataState
        let datastate = SirinDataState{
            data: sirin.data.clone(),
            mode: state.mode.clone(),
            altitude: filter.position().z,
            apogee: state.apogee.clone(),
            gps_fix: state.gps_fix.clone(),
            pos: filter.position(),
            vel: filter.velocity(),
            rot_quaternion: filter.attitude(),
            voltage: sirin.match_sense.read(),
        };

        /*info!("Calculate barometric altitude");
        if let Ok(pressure) = sirin.data.baro.pressure {
            let measured_altitude = approx_pressure_altitude(pressure.convert());
            state.altitude = measured_altitude - initial_altitude;
            if state.altitude.value > max_altitude.value {
                max_altitude = state.altitude;
            }
        }
        //info!("Altitude calculated");*/

        let accel_mag_squared = if let Ok(accel) = &sirin.data.imu.accel {
            let x_f64: MicroGs<f64> = (accel.x.value as f64).with_units();
            let x: Gs<f64> = x_f64.convert();
            let y_f64: MicroGs<f64> = (accel.y.value as f64).with_units();
            let y: Gs<f64> = y_f64.convert();
            let z_f64: MicroGs<f64> = (accel.z.value as f64).with_units();
            let z: Gs<f64> = z_f64.convert();
            Some(x * x + y * y + z * z)
        } else {
            None
        };

        match state.mode {
            SirinMode::Standby => {
                if state.altitude.value > altitude_threshold
                    || accel_mag_squared.is_some_and(|accel| accel.value > accel_threshold.value)
                    || desired_mode == Some(SirinMode::Flight)
                {
                    sirin.led_white.set_high();
                    state.mode = SirinMode::Flight;
                    info!("Entered flight mode...");
                    FLASH_LOGGING_ENABLED.store(true, Ordering::Relaxed);
                    launched_at = Some(Instant::now());
                }
            },
            SirinMode::Flight => {
                //Log a DataState packet every 100 milliseconds
                OUT_CHANNEL.publish_immediate(IoPacket::new(
                    IoChannel::Flash, OutPacket::LogEntry(
                        LogEntry::new(
                            sirin.data.time,
                            Log::DataState(datastate.clone())
                        )
                    )
                ));

                dur = launched_at.map(|t| Instant::now() - t);

                //Check apogee: altitude must be below the max by apogee_error
                //for THRESHOLD_REQUIRED of the last THRESHOLD_WINDOW measurements
                apogee_check.push(max_altitude.value > state.altitude.value + apogee_error);
                if state.apogee.is_none() && apogee_check.passes(config::THRESHOLD_REQUIRED) {
                    state.apogee = Some(max_altitude);

                    //Deploy apo parachute
                    if !apo_deployed && dur.is_some_and(|d| d > Duration::from_secs(timeout)) {
                        Sirin::deploy_chute_apo(&mut sirin.parachute_apo);
                        OUT_CHANNEL.publish_immediate(IoPacket::new(
                            IoChannel::Flash,
                            OutPacket::DeployedApoAt(sirin.data.time.value)
                        ));
                        apo_deployed = true;
                    }

                    state.mode = SirinMode::Descent;
                    info!("Exiting flight mode...");
                    info!("Entered descent mode...");
                }
            },
            SirinMode::Descent => {
                // Maintain active flight duration update
                dur = launched_at.map(|t| Instant::now() - t);
                sirin.led_white.set_low();
                sirin.led_blue.set_high();

                // Log DataState packet during descent
                OUT_CHANNEL.publish_immediate(IoPacket::new(
                    IoChannel::Flash,
                    OutPacket::LogEntry(LogEntry::new(
                        sirin.data.time,
                        Log::DataState(datastate.clone()),
                    )),
                ));

                //Deploy main parachute
                if state.apogee.is_some() {
                    if state.altitude.value < main_deployment_altitude {
                        if !main_deployed {
                            Sirin::deploy_chute_main(&mut sirin.parachute_main);
                            OUT_CHANNEL.publish_immediate(IoPacket::new(
                    IoChannel::Flash, OutPacket::DeployedMainAt(sirin.data.time.value)
                            ));
                            main_deployed = true;
                        }
                    }
                }

                //Landed once velocity is below threshold for THRESHOLD_REQUIRED of the
                //last THRESHOLD_WINDOW measurements. Ticks without a valid velocity count as misses.
                let total_vel: f64 = (state.nominal.vel.x.powf(2.0) + state.nominal.vel.y.value.powf(2.0) + state.nominal.vel.z.powf(2.0)).sqrt() as f64;
                landed_check.push(
                    velocity_valid
                        && total_vel.is_finite()
                        && total_vel < config::LANDED_VELOCITY_THRESHOLD_M_S
                );
                if landed_check.passes(config::THRESHOLD_REQUIRED) {
                    desired_mode = Some(SirinMode::Landed);
                    info!("Exiting descent mode...");
                }

                if let Some(launched_at) = launched_at {
                    dur = Some(Instant::now() - launched_at);
                    if dur.unwrap() > Duration::from_secs(config::FLIGHT_TIMEOUT_S) {
                        desired_mode = Some(SirinMode::Landed);
                        info!("Flight duration exceeded, exiting descent mode...");
                    }
                }

                if desired_mode == Some(SirinMode::Landed) {
                    sirin.led_white.set_low();
                    state.mode = SirinMode::Landed;
                    FLASH_LOGGING_ENABLED.store(false, Ordering::Relaxed);
                }
                
            },
            SirinMode::Landed => {
                sirin.led_blue.set_low();
            }
        }

        if let Some(mode) = desired_mode {
            state.mode = mode;
            desired_mode = None;
        }

        //info!("Broadcast");
        //TODO: This might be double logging to flash during flight
        //broadcast_log(sirin.data.time, Log::Data(sirin.data.clone()));
        //broadcast_log(sirin.data.time, Log::State(state.clone()));

        //Broadcast over USB at the rate set in config
        if config::USB_BROADCASTING_ALLOWED{
            usb_ticks_since_broadcast += 1;
            if usb_ticks_since_broadcast >= period_ticks(config::USB_BROADCAST_HZ) {
                usb_ticks_since_broadcast = 0;
                OUT_CHANNEL.publish_immediate(IoPacket::new(
                    IoChannel::Usb, OutPacket::LogEntry(LogEntry::new(
                        sirin.data.time,
                        Log::DataState(datastate.clone())
                    ))
                ));
            }
        }

        //Broadcast over LoRa at the rate set for the current mode
        ticks_since_broadcast += 1;
        if ticks_since_broadcast >= period_ticks(broadcast_rate_hz(state.mode)) {
            ticks_since_broadcast = 0;
            OUT_CHANNEL.publish_immediate(IoPacket::new(
                IoChannel::ToLoRa, OutPacket::LogEntry(LogEntry::new(
                    sirin.data.time,
                    Log::DataState(datastate.clone())
                ))
            ));
        }
    }
}

//TODO: Move all the code below into a more permanent location

// Broadcast rate for each flight mode
fn broadcast_rate_hz(mode: SirinMode) -> f64 {
    match mode {
        SirinMode::Standby => config::STANDBY_BROADCAST_HZ,
        SirinMode::Flight => config::FLIGHT_BROADCAST_HZ,
        SirinMode::Descent => config::DESCENT_BROADCAST_HZ,
        SirinMode::Landed => config::LANDED_BROADCAST_HZ,
    }
}

// Loop ticks between broadcasts at "rate_hz"
fn period_ticks(rate_hz: f64) -> u32 {
    let ticks = config::LOOP_RATE_HZ / rate_hz;
    ((ticks + 0.5) as u32).max(1)
}
// Ring buffer and impl to account for outliers in apogee / landing detection 
struct ThresholdWindow<const N: usize> {
    history: [bool; N],
    next: usize,
    filled: usize,
}

impl<const N: usize> ThresholdWindow<N> {
    const fn new() -> Self {
        Self { history: [false; N], next: 0, filled: 0 }
    }

    fn push(&mut self, met: bool) {
        self.history[self.next] = met;
        self.next = (self.next + 1) % N;
        self.filled = (self.filled + 1).min(N);
    }

    fn passes(&self, required: usize) -> bool {
        self.filled == N && self.history.iter().filter(|&&met| met).count() >= required
    }
}