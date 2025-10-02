use embedded_hal::blocking::delay::DelayMs;
use stm32h7xx_hal::gpio::{Output, PushPull, Pin};
use stm32h7xx_hal::pwm::PwmChannel;

pub struct ServoController {
    pwm: PwmChannel<'static, stm32f4xx_hal::pac::TIM2>,
}

impl ServoController {
    pub fn init<D: DelayMs<u16>>(&mut self, delay: &mut D) {
        // setup PWM timers here
        self.pwm.enable();
        delay.delay_ms(100);
    }

    pub fn unfold_sheet(&mut self) {
        // Set PWM duty cycle to control servo angle
        let max_duty = self.pwm.get_max_duty();
        self.pwm.set_duty(max_duty / 10); 
    }
}

pub struct Camera {
    pin: Pin<'B', 0, Output<PushPull>>, 
}

impl Camera {
    pub fn init(&mut self) {
        self.pin.set_low();
    }

    pub fn start_recording(&mut self) {
        self.pin.set_high();
    }

    pub fn stop_recording(&mut self) {
        self.pin.set_low();
    }
}