#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32, f64::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::{debug, info, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::{ReadRfm9, Rfm9};
use w25qx::W25Q;
use {defmt_rtt as _, panic_probe as _};
use sirin::{error::SirinError, flash::Flash, io::{broadcast, radio_io_task, send_packet, try_receive_packet, usb_input_task, usb_output_task, IN_CHANNEL}, packet::{InPacket, IoChannel, IoPacket, OutPacket, PacketError}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, EcefPos, State, Vel}, subsystems::SirinData, uunit::WithUnits, Radio, Sirin};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{Publisher, Subscriber}};
use sirin_shared::mode::SirinMode;
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

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

async fn main_task(sirin: &'static mut Sirin) -> Result<(), SirinError> {
    let mut i: u32 = 0;

    //sirin.spawner.spawn(radio_io_task(&mut sirin.radio)).unwrap();
    sirin.spawner.spawn(usb_input_task(&mut sirin.usb.read_ep)).unwrap();
    sirin.spawner.spawn(usb_output_task(&mut sirin.usb.write_ep)).unwrap();

    let mut state: State;
    let mut mode = SirinMode::Standby;

    loop {
        while let Ok(io_packet) = try_receive_packet() {
            match io_packet.packet {
                InPacket::Null => {},
                InPacket::DumpFlash => {
                    todo!()
                }
                InPacket::QueryConfig => {
                    send_packet(IoPacket::new(
                        io_packet.channel,
                        OutPacket::Config(sirin.config.clone())
                    ));
                }
                InPacket::SetConfig(config) => {
                    info!("Updating the config to {:?}", Debug2Format(&config));

                    sirin.flash.save_config(&config).await.unwrap();
                    cortex_m::peripheral::SCB::sys_reset();
                }
                InPacket::QueryMode => {
                    send_packet(io_packet.reply(OutPacket::Mode(mode)));
                }
                InPacket::SetMode(m) => {
                    mode = m;
                }
            }
        }

        Timer::after_millis(100).await;

        sirin.led.set_high();

        sirin.data = SirinData::measure(
            &mut sirin.baro,
            &mut sirin.imu,
            &mut sirin.high_g_imu
        ).await;

        // TODO: Replace with call to C code
        unsafe {
            let mut _state: MaybeUninit<State> = MaybeUninit::uninit();
            run_kalman_filter(&mut _state, &sirin.data);
            state = _state.assume_init();
        }

        if i % 10 == 0 {
            // Do logging
            broadcast(OutPacket::State(state));
        }

        i = i.wrapping_add(1);

        sirin.led.set_low();
    }
}

unsafe fn run_kalman_filter(state: *mut MaybeUninit<State>, data: *const SirinData) {
    let accel = (*data).imu.accel.as_ref().unwrap();

    state.write(MaybeUninit::new(State {
        pos: EcefPos {
            x: 0.0.with_units(),
            y: 0.0.with_units(),
            z: 0.0.with_units(),
        },
        vel: Vel {
            x: 0.0.with_units(),
            y: 0.0.with_units(),
            z: 0.0.with_units(),
        },
        accel: Accel {
            x: (accel.x.value as f64).with_units(),
            y: (accel.y.value as f64).with_units(),
            z: (accel.z.value as f64).with_units(),
        },
        altitude: 0.0.with_units()
    }));
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