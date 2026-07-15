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
use sirin::{Radio, Sirin, error::SirinError, flash::Flash, gps::{GPS_FIX, gps_task}, io::{FLASH_LOGGING_ENABLED, IN_CHANNEL, OUT_CHANNEL, broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_flash_logging_enabled, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task}, packet::{SirinDataState, GpsFixType, InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page, SirinData, SirinState}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, subsystems::measure_sirin, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, Meters, MetersPerSecond2, MicroGs, WithUnits}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{PubSubBehavior, Publisher, Subscriber}};
use sirin_shared::{mode::SirinMode, physics::approx_pressure_altitude, time::AbsoluteTimeReference};
use sirin::song::SongDiscriminant;
use nalgebra as na;
use na::{Matrix3, Matrix6, Vector3, UnitQuaternion, Rotation3};

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
    /*

    FOR IREC ROCKET - CHECK TO ENSURE THESE VALUES ARE CODED:
    DO NOT PUSH CODE WITH THESE VALUES SIGNIFICANTLY CHANGED
    accel_threshold = 10G * 10G
    altitude_threshold = 20m
    main_deployment_altitude = 457.2m (1500ft)
    flight_duration = 600s
    apogee_error = 1m
    timeout = 25s

     */

    let accel_threshold: Gs<f64> = (10.0 * 10.0).with_units(); //In Gs squared
    let altitude_threshold = 20.0; //In meters
    let main_deployment_altitude= 457.2; //In meters
    let flight_duration = 1000; //In seconds
    let apogee_error = 4.0; //In meters
    let timeout = 25; //In seconds

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
    let initial_altitude = altitude_array[50].with_units();
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

    info!("Start main");

    //Update the loop every 500 ms
    let mut ticker = Ticker::every(Duration::from_millis(100));

    let mut launched_at = None;
    let mut dur: Option<Duration> = None;
    let mut max_altitude: Meters<f64> = 0.0.with_units();

    let mut desired_mode = None;

    let mut i = 0;

    loop {
        i += 1;
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
                    info!("Waiting until flash is ready...");
                    flash.w25q.until_ready().await?;
                    info!("Finished chip erase.");
                    send_packet(io_packet.reply(OutPacket::Ok));

                    Timer::after_millis(500).await;

                    panic!("Reboot");
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
        //state.nominal = kalman_filter(&mut self, &prev_reading);

        //Calculate DataState
        let datastate = SirinDataState{
            data: sirin.data.clone(),
            mode: state.mode.clone(),
            altitude: state.altitude.clone(),
            apogee: state.apogee.clone(),
            gps_fix: state.gps_fix.clone(),
            pos: state.nominal.pos.clone(),
            vel: state.nominal.vel.clone(),
            rot_quaternion: state.nominal.rot_quaternion.clone()
        };

        //info!("Calculate barometric altitude");
        if let Ok(pressure) = sirin.data.baro.pressure {
            let measured_altitude = approx_pressure_altitude(pressure.convert());
            state.altitude = measured_altitude - initial_altitude;
            if state.altitude.value > max_altitude.value {
                max_altitude = state.altitude;
            }
        }
        //info!("Altitude calculated");

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
                    sirin.led.set_high();
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

                /*OUT_CHANNEL.publish_immediate(IoPacket::new(
                    IoChannel::Flash, OutPacket::LogEntry(
                        LogEntry::new(
                            sirin.data.time,
                            Log::State(state.clone())
                        )
                    )
                ));*/

                //Check apogee, deploy apo parachute
                if let None = state.apogee {
                    if max_altitude.value > state.altitude.value + apogee_error{
                        state.apogee = Some(max_altitude);
                        match dur{
                            Some(duration) => {
                                if duration > Duration::from_secs(timeout){
                                    if !apo_deployed{
                                        //Timer::after_millis(1000).await;
                                        Sirin::deploy_chute_apo(&mut sirin.parachute_apo);
                                        OUT_CHANNEL.publish_immediate(IoPacket::new(
                                        IoChannel::Flash, 
                                        OutPacket::DeployedApoAt(sirin.data.time.value)
                                        ));
                                        apo_deployed = true;
                                    }
                                }
                            },
                            None => {}
                        }
                    }
                }

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

                //Timeout after designated time
                if let Some(launched_at) = launched_at {
                    dur = Some(Instant::now() - launched_at);
                    if dur.unwrap() > Duration::from_secs(flight_duration) {
                        desired_mode = Some(SirinMode::Landed);
                        info!("Exiting flight mode...");
                    }
                }

                if desired_mode == Some(SirinMode::Landed) {
                    sirin.led.set_low();
                    state.mode = SirinMode::Landed;
                    FLASH_LOGGING_ENABLED.store(false, Ordering::Relaxed);
                }
            },
            SirinMode::Landed => {
                
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

        //TODO: Make State now exceed the MAX_OUTPACKET_SIZE
        OUT_CHANNEL.publish_immediate(IoPacket::new(
            IoChannel::Usb, OutPacket::LogEntry(LogEntry::new(
                sirin.data.time,
                Log::DataState(datastate.clone())
            ))
        ));

        /* 
        OUT_CHANNEL.publish_immediate(IoPacket::new(
            IoChannel::Usb, OutPacket::LogEntry(LogEntry::new(
                sirin.data.time,
                Log::Data(sirin.data.clone())
            ))
        ));
        */

        //info!("Try get GPS fix");
        if let Some(fix) = GPS_FIX.try_take() {
            //if fix.fix_type != GpsFixType::NoFix {
                state.gps_fix = fix;
            //}
        }

        //Alternates sending State and Data packets every half second
        //buffer overflow error, this is me trying to mitigate it since there is no time to 
        //info!("Transmit data");
        if i % 5 == 0  {
            OUT_CHANNEL.publish_immediate(IoPacket::new(
                IoChannel::ToLoRa, OutPacket::LogEntry(LogEntry::new(
                    sirin.data.time,
                    Log::DataState(datastate)
                ))
            ));
            //println!("State broadcasted!");
        }


        /* Uncomment if you ever want to broadcast raw data over LoRa for whatever reason
        if i % 5 == 0{
            OUT_CHANNEL.publish_immediate(IoPacket::new(
                IoChannel::ToLoRa, OutPacket::LogEntry(LogEntry::new(
                    sirin.data.time,
                    Log::Data(sirin.data.clone())
                ))
            ));
            //println!("Data broadcasted!");
        }
        */


        //info!("Final state altitude: {}", state.altitude.value);
        //info!("Done with GPS");
    }
}