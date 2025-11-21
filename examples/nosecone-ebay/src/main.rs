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
use sirin::{Radio, Sirin, error::SirinError, flash::Flash, gps::{GPS_FIX, gps_task}, io::{FLASH_LOGGING_ENABLED, IN_CHANNEL, OUT_CHANNEL, broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task}, packet::{GpsFixType, InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page, SirinData, SirinState}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, subsystems::measure_sirin, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, Meters, MetersPerSecond2, MicroGs, WithUnits}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{PubSubBehavior, Publisher, Subscriber}};
use sirin_shared::{mode::SirinMode, physics::approx_pressure_altitude, time::AbsoluteTimeReference};
use sirin::song::SongDiscriminant;

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
    let accel_threshold: Gs<f64> = (10.0).with_units(); //In Gs
    let altitude_threshold = 33.0; //In meters
    let main_deployment_altitude= 1500.0; //In meters
    let flight_duration = 100; //In seconds
    let apogee_error = 7.0; //In meters

    let mut apo_deployed = false;
    let mut main_deployed = false;
    

    let mut state = SirinState::default();

    let mut initial_altitude = approx_pressure_altitude(sirin.baro.read().await?.pressure.convert());
    let mut altitude_array: [f64; 100] = [0.0; 100];
    for i in 0..100 {
        let altitude = approx_pressure_altitude(sirin.baro.read().await?.pressure.convert());
        altitude_array[i] = altitude.value;
        Timer::after_millis(25).await;
    }
    
    altitude_array.sort_unstable_by(|a, b | a.partial_cmp(b).unwrap());
    initial_altitude = altitude_array[50].with_units();
   

    info!("Initial altitude: {}", initial_altitude.value);

    sirin.spawner.spawn(radio_io_task(&sirin.config, &mut sirin.radio)).unwrap();
    sirin.spawner.spawn(usb_input_task(&mut sirin.usb.read_ep)).unwrap();
    sirin.spawner.spawn(usb_output_task(&mut sirin.usb.write_ep)).unwrap();
    sirin.spawner.spawn(gps_task(&mut sirin.gps_rx, &mut sirin.gps_tx)).unwrap();

    let mut i = 0;
    
    let mut flash = Mutex::new(&mut sirin.flash);
    sirin.spawner.spawn(flash_io_task(unsafe {
        transmute_into_static(&mut flash)
    })).unwrap();

    info!("Start main");

    let mut ticker = Ticker::every(Duration::from_millis(500));

    let mut launched_at = None;
    let mut max_altitude: Meters<f64> = 0.0.with_units();

    let mut desired_mode = None;

    loop {
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
                    if duration_since_epoch().is_some() {
                        continue;
                    }

                    // Subtract current uptime from time since boot
                    let ms_since_epoch = reference.ms_since_epoch - Instant::now().as_millis();

                    set_duration_since_epoch(Duration::from_millis(ms_since_epoch));
                    flash.lock().await.set_absolute_time_reference(AbsoluteTimeReference { ms_since_epoch }).await?;

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
                        send_packet(io_packet.reply(OutPacket::Error(PacketError::FlightNotFound(index))));
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
                    //info!("Starting chip erase...");
                    flash.w25q.chip_erase().await?;
                    flash.w25q.until_ready().await?;
                    //info!("Finished chip erase.");
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

        //info!("Calculate altitude");
        info!("Initial altitude: {}", initial_altitude.value);
        if let Ok(pressure) = sirin.data.baro.pressure {
            let measured_altitude = approx_pressure_altitude(pressure.convert());
            info!("Measured altitude: {}", measured_altitude.value);
            state.altitude = measured_altitude - initial_altitude;
            info!("Relative altitude: {}", state.altitude.value);
            //info!("Estimated altitude: {}m", state.altitude.value);

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
                    FLASH_LOGGING_ENABLED.store(true, Ordering::Relaxed);
                    launched_at = Some(Instant::now());
                }
            },
            SirinMode::Flight => {
                OUT_CHANNEL.publish_immediate(IoPacket::new(
                    IoChannel::Flash, OutPacket::LogEntry(
                        LogEntry::new(
                            sirin.data.time,
                            Log::Data(sirin.data.clone())
                        )
                    )
                ));

                //Check apogee, deploy apo parachute
                if let None = state.apogee {
                    if max_altitude.value > state.altitude.value + apogee_error {
                        state.apogee = Some(max_altitude);
                        if !apo_deployed{
                            Sirin::deploy_chute_apo(&mut sirin.parachute_apo);
                            OUT_CHANNEL.publish_immediate(IoPacket::new(
                        IoChannel::Flash, 
                        OutPacket::DeployedApoAt(sirin.data.time.value)
                            ));
                            apo_deployed = true;
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
                    let dur = Instant::now() - launched_at;
                    if dur > Duration::from_secs(flight_duration) {
                        desired_mode = Some(SirinMode::Landed)
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
        broadcast_log(sirin.data.time, Log::Data(sirin.data.clone()));

        //info!("Try get GPS fix");
        if let Some(fix) = GPS_FIX.try_take() {
            //if fix.fix_type != GpsFixType::NoFix {
                state.gps = fix;
            //}
        }

        //info!("Transmit data");
        
        if i % 10 == 0 {
            OUT_CHANNEL.publish_immediate(IoPacket::new(
                IoChannel::Flash, OutPacket::LogEntry(LogEntry::new(
                    sirin.data.time,
                    Log::Data(sirin.data.clone())

                ))
            ));
            OUT_CHANNEL.publish_immediate(IoPacket::new(
                IoChannel::ToLoRa, OutPacket::LogEntry(LogEntry::new(
                    sirin.data.time,
                    Log::Data(sirin.data.clone())
                ))
            ));
        }
        info!("Final state altitude: {}", state.altitude.value);
        //info!("Done with GPS");


        
        //i = i.wrapping_add(1);

    }
}

// TODO: airbreaks
/*#[task]
async fn kalman(
    mut event_sub: Subscriber<'static, CriticalSectionRawMutex, Event, 100, 4, 4>
) {
    loop {
        let event = event_sub.next_message_pure().await;

        match event {
            Event::Measurement(measurement) => {
                match measurement {
                    Measurement::Baro(bmp3_readout) => todo!(),
                    Measurement::ImuAccel(accel) => todo!(),
                    Measurement::ImuAngularVel(angular_vel) => todo!(),
                }
            },
        }

        // Example: call a C function from sirin-c Rust crate
        // Edit sirin-c crate and c project to add more functions
        sirin_c::cmsis_dsp_sin(f32::consts::PI / 2.0);
    }
}*/