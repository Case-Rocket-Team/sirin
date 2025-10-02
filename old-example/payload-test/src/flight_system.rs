
#![no_std]
#![no_main]

mod flight_computer;
use flight_computer::FlightComputer;

use cortex_m_rt::entry;
use cortex_m::Peripherals as CorePeripherals;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    delay::Delay,
    gpio::{Output, PushPull},
};

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    let mut delay = Delay::new(cp.SYST, clocks);

    let gpioa = dp.GPIOA.split();
    let screw_switch = gpioa.pa0.into_pull_down_input();
    let mut led = gpioa.pa5.into_push_pull_output();

    while screw_switch.is_low().unwrap() {

    }

    let mut fc = FlightComputer::new(dp, clocks);
    fc.initialize_all(&mut delay);
    fc.enter_standby();

    for _ in 0..5 {
        led.set_high().unwrap();
        delay.delay_ms(200_u16);
        led.set_low().unwrap();
        delay.delay_ms(200_u16);
    }

    loop {}
}
