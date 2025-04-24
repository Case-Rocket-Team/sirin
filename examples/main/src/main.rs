#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32, f64::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::{debug, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::Timer;
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::ReadRfm9;
use {defmt_rtt as _, panic_probe as _};
use sirin::{event::Event, flash_logger::FlashLogger, song::{FromSong, OutPacket, SongSize}, state::{Accel, EcefPos, State, Vel}, subsystems::SirinData, uunit::WithUnits, Sirin};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::{Publisher, Subscriber}};

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

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

async fn main_task(mut sirin: &'static mut Sirin) {
    let mut i: u32 = 0;

    // going to change this later
    let mut state: State;

    loop {
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
            // let log = LogData::State(state);

            // TODO: figure out how to do this without another task while also not
            // freezing up the main task. Maybe break up erasing into a separate function?
            if !sirin.flash.is_busy().await.is_ok_and(|b| b) {
                // TODO: what should we do with this error? It's not like we can log it...
                // let _ = flash_logger.log(&mut sirin.flash, &log).await;
            }
        }

        call_user_code(&mut sirin);

        i = i.wrapping_add(1);
    }
}

#[allow(unused)]
fn call_user_code(sirin: &mut Sirin) {
    // Dummy function to ensure that we can get a mutable borrow of Sirin
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