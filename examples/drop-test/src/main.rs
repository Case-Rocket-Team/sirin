#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin, sync::atomic::Ordering, u16};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use lsm6dsv_spi::{SflpOdr};
use defmt::{debug, info, println, warn, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::{Duration, Instant, Ticker, Timer, TICK_HZ};
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::{ReadRfm9, Rfm9};
use w25qx::W25Q;
use {defmt_rtt as _, panic_probe as _};
use sirin::{Radio, Sirin, error::SirinError, flash::Flash, gps::{GPS_FIX, gps_task, gps_task_with_nav_db}, io::{EKF_DEBUG, FLASH_LOGGING_ENABLED, IN_CHANNEL, OUT_CHANNEL, broadcast, broadcast_log, flash_io_task, log_to_flash, radio_io_task, send_packet, send_packet_reliable,set_flash_logging_enabled, set_usb_broadcasting_enabled, set_usb_exporting, try_receive_packet, usb_input_task, usb_output_task}, packet::{SirinDataState, GpsFixType, InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page, SirinData, SirinState}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, subsystems::measure_sirin, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, Meters, MetersPerSecond2, MicroGs, WithUnits}};
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
    
    // Stays in Standby for the whole test
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
    // No radio task: nothing is sent over LoRa
    //sirin.spawner.spawn(radio_io_task(&sirin.config, &mut sirin.radio)).unwrap();
    sirin.spawner.spawn(usb_input_task(&mut sirin.usb.read_ep)).unwrap();
    sirin.spawner.spawn(usb_output_task(&mut sirin.usb.write_ep)).unwrap();

    let mut flash = Mutex::new(&mut sirin.flash);
    let flash_static: &'static Mutex<&'static mut Flash> = unsafe { transmute_into_static(&mut flash) };
    sirin.spawner.spawn(flash_io_task(flash_static)).unwrap();
    sirin.spawner.spawn(gps_task_with_nav_db(&mut sirin.gps_rx, &mut sirin.gps_tx, flash_static)).unwrap();

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

    // Drop test sequence (replaces the flight modes)
    let mut phase = Phase::WaitingForGps;
    let mut phase_started = Instant::now();
    // Set once the first 3D fix reaches the filter
    let mut gps_locked = false;
    info!("Drop test: waiting for a 3D GPS fix (blue LED blinking)");


    // Kalman filter init
    let ekf_settings = Ekf_settings {
        accel_var_f2ps4: 0.001272,
        accel_bias_var_f2ps4: 1e-5,
        gyro_var_rad2ps2: 3.668276077e-7,
        gyro_bias_var_rad2ps2: 1e-10,        // σ ≈ 0.00105 rad/s (≈ 0.06 °/s)
        baro_var_ft2: 100.0,              // σ = 10 ft (≈ 3 m
        magn_covar_gauss2: [                // σ = 3.2 mG (x, y), 4.1 mG (z)
            0.025 * 0.025, 0.0, 0.0,
            0.0, 0.025 * 0.025, 0.0,
            0.0, 0.0, 0.025 * 0.025,
        ],
        magn_bias_var_gauss2: 1e-6,
        gnss_covar_ft2: [                   // unused while you don't inject GPS
            18.0, 0.0, 0.0,
            0.0, 18.0, 0.0,
            0.0, 0.0, 200.0,
        ],
    };

    let t_us = Instant::now().as_micros() as u32;
    let mut inject_counter = 0;
    // Last raw magnetometer reading so each sample is injected only once
    let mut last_mag: Option<(i16, i16, i16)> = None;
    // When the board last became still
    let mut calm_since: Option<Instant> = None;

    let mut filter = Ekf::new(&ekf_settings, &gravity, &gyro_bias, &mag_reference, t_us).unwrap();
    // The accel's offset drifts ~10-20 mg between runs, so start the filter's bias
    // from this run's still-board gravity reading instead of zero
    match filter.seed_accel_bias_from_gravity(&gravity) {
        Some(err_g) => info!("Accel bias seeded: reads {} mg off 1 g at rest", err_g * 1000.0),
        None => warn!("Accel bias not seeded: gravity reading too far from 1 g"),
    }

    // Converts GPS ECEF coordinates to the local east/north/up frame
    const MAG_DECLINATION_DEG: f32 = 0.0;
    let (north_cos, north_sin) = {
        let (mx, my, _) = filter.world_magn_north_gauss();
        let h = ComplexField::sqrt(mx * mx + my * my);
        let (c, s) = (mx / h, my / h);
        let d = MAG_DECLINATION_DEG * (PI / 180.0);
        let (cd, sd) = (ComplexField::cos(d), ComplexField::sin(d));
        (c * cd - s * sd, s * cd + c * sd)
    };
    // Pad ECEF position
    let mut gps_pad: Option<([i32; 3], [[f64; 3]; 3])> = None;

    loop {

        //info!("Handle input packets");
        while let Ok(io_packet) = try_receive_packet() {
            info!("Received packet: {:?}", Debug2Format(&io_packet));
            match io_packet.packet {
                // Passive parachute: never drive the pyro pins
                InPacket::DeployMain | InPacket::DeployApo => {
                    warn!("Drop test: ignoring parachute deploy command");
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
                InPacket::SetMode(_) => {
                    // No flight modes in the drop test
                    warn!("Drop test: ignoring SetMode, staying in Standby");
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
                    // Look up by list position, the same numbering QueryFlights reports
                    // (the deque's physical slots stop matching once old flights are removed)
                    let Some(header) = flash.flight_headers.iter().nth(index as usize).cloned() else {
                        //TODO: Fix error
                        //send_packet(io_packet.reply(OutPacket::Error(PacketError::FlightNotFound(index))));
                        warn!("Flight {} not found ({} flights stored)", index, flash.flight_headers.len());
                        // Reply anyway so the host's export ends instead of waiting forever
                        send_packet_reliable(io_packet.reply(OutPacket::Ok)).await;
                        continue;
                    };

                    info!("Reading flight {} (header slot {})", index, header.index);

                    // Drop live USB data (and any already queued) until the export is done
                    set_usb_exporting(true);

                    // Wait for USB to take each log so none are dropped
                    let mut iter = flash.read_logs(&header);
                    let mut host_reading = true;
                    while let Some(log) = iter.next().await {
                        //info!("Sent log: {}", Debug2Format(&log));
                        if !send_packet_reliable(io_packet.reply(log?)).await {
                            warn!("Host stopped reading, export of flight {} abandoned", index);
                            host_reading = false;
                            break;
                        }
                    }

                    //info!("Done writing logs.")
                    if host_reading {
                        send_packet_reliable(io_packet.reply(OutPacket::Ok)).await;
                    }
                    set_usb_exporting(false);
                    continue;
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

        // After logging, handling packets is the only thing left to do
        if phase == Phase::Done {
            continue;
        }

        //info!("Measure Sirin data");
        sirin.data = measure_sirin(
            &mut sirin.baro,
            &mut sirin.imu,
            &mut sirin.high_g_imu,
            &mut sirin.magnetometer
        ).await;

        filter.inject_imu(&sirin.data.imu, Instant::now().as_micros() as u32);

        if let Some(fix) = GPS_FIX.try_take() {
            let new_3d_fix = fix.fix_type == GpsFixType::Fix3d;
            state.gps_fix = fix;
            // Only inject a new 3D fix
            if new_3d_fix {
                gps_locked = true;
                let f = &state.gps_fix;
                let ecef = [f.pos.x.value, f.pos.y.value, f.pos.z.value];
                let (pad, r) = *gps_pad.get_or_insert_with(|| {
                    let lat = f.lat * (core::f64::consts::PI / 180.0);
                    let lon = f.lon * (core::f64::consts::PI / 180.0);
                    let (sl, cl) = (ComplexField::sin(lat), ComplexField::cos(lat));
                    let (so, co) = (ComplexField::sin(lon), ComplexField::cos(lon));
                    (ecef, [
                        [-so, co, 0.0],             // east
                        [-sl * co, -sl * so, cl],   // north
                        [cl * co, cl * so, sl],     // up
                    ])
                });
                // Subtract the pad while still in integer cm
                let d = [0, 1, 2].map(|i| (ecef[i] - pad[i]) as f64 * 0.01);
                let [e, n, u] = r.map(|row| row[0] * d[0] + row[1] * d[1] + row[2] * d[2]);
                let (e, n) = (e as f32, n as f32);
                // East/north -> filter world x/y
                let x = e * north_sin + n * north_cos;
                let y = -e * north_cos + n * north_sin;
                filter.inject_gnss(
                    &Pos { x: x.with_units(), y: y.with_units(), z: (u as f32).with_units() },
                    Instant::now().as_micros() as u32
                );
            }
        } 

        inject_counter += 1;

        // Only inject magnetometer readings when the board is nearly still and the reading is new
        const MAG_MAX_GYRO_UDPS: f32 = 25_000_000.0;
        const MAG_MAX_ACCEL_ERR_UG: f32 = 100_000.0;
        // The board must stay still this long before readings are injected. Mag averages over a timeframe so
        // an extended duration is needed to obtain real values
        const MAG_SETTLE_MS: u64 = 750;
        if let Ok(m) = &sirin.data.magnetometer.mag {
            let new_mag = last_mag != Some((m.x, m.y, m.z));
            last_mag = Some((m.x, m.y, m.z));

            let calm = match (&sirin.data.imu.angular_vel, &sirin.data.imu.accel) {
                (Ok(w), Ok(a)) => {
                    let w2 = [w.x.value, w.y.value, w.z.value].iter().map(|&v| (v as f32) * (v as f32)).sum::<f32>();
                    let a2 = [a.x.value, a.y.value, a.z.value].iter().map(|&v| (v as f32) * (v as f32)).sum::<f32>();
                    let a_lo = 1_000_000.0 - MAG_MAX_ACCEL_ERR_UG;
                    let a_hi = 1_000_000.0 + MAG_MAX_ACCEL_ERR_UG;
                    w2 < MAG_MAX_GYRO_UDPS * MAG_MAX_GYRO_UDPS && a2 > a_lo * a_lo && a2 < a_hi * a_hi
                }
                _ => false,
            };
            if !calm {
                calm_since = None;
            } else if calm_since.is_none() {
                calm_since = Some(Instant::now());
            }
            let settled = calm_since.is_some_and(|t| Instant::now() - t >= Duration::from_millis(MAG_SETTLE_MS));

            if new_mag && settled {
                filter.inject_magn(&sirin.data.magnetometer, Instant::now().as_micros() as u32);
            }
        }
        if let Ok(pressure) = sirin.data.baro.pressure {
            if inject_counter % 1 == 0{
                filter.inject_baro(approx_pressure_altitude(pressure.convert()) - initial_altitude, Instant::now().as_micros() as u32);
            }
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

        // Drop test sequence in place of the flight modes (state.mode stays Standby)
        let elapsed = Instant::now() - phase_started;
        let blink_on = (elapsed.as_millis() % config::BLINK_PERIOD_MS) < config::BLINK_PERIOD_MS / 2;
        match phase {
            Phase::WaitingForGps => {
                sirin.led_blue.set_level(if blink_on { Level::High } else { Level::Low });
                if gps_locked {
                    sirin.led_blue.set_low();
                    phase = Phase::Settling;
                    phase_started = Instant::now();
                    info!("Drop test: GPS locked, settling for {} s (white LED blinking)", config::SETTLE_S);
                }
            }
            Phase::Settling => {
                sirin.led_white.set_level(if blink_on { Level::High } else { Level::Low });
                if elapsed >= Duration::from_secs(config::SETTLE_S) {
                    sirin.led_white.set_high();
                    FLASH_LOGGING_ENABLED.store(true, Ordering::Relaxed);
                    phase = Phase::Logging;
                    phase_started = Instant::now();
                    info!("Drop test: logging to flash for {} s (white LED solid) - DROP NOW", config::LOG_S);
                }
            }
            Phase::Logging => {
                if elapsed >= Duration::from_secs(config::LOG_S) {
                    // Entries already queued still get written
                    FLASH_LOGGING_ENABLED.store(false, Ordering::Relaxed);
                    sirin.led_white.set_low();
                    sirin.led_blue.set_low();
                    phase = Phase::Done;
                    info!("Drop test: done, logging stopped. Only handling packets now.");
                } else {
                    log_to_flash(OutPacket::LogEntry(LogEntry::new(
                        sirin.data.time,
                        Log::DataState(datastate.clone()),
                    )));
                }
            }
            Phase::Done => {}
        }
    }
}

// Drop test sequence, run once per boot
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    // Filter running, waiting for the first 3D GPS fix
    WaitingForGps,
    // Letting the filter's biases settle before logging
    Settling,
    // Logging every DataState to flash
    Logging,
    // Finished; only incoming packets are handled
    Done,
}
