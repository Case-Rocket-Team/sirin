use embedded_hal::blocking::delay::DelayMs;

pub struct PiezoSensor;

impl PiezoSensor {
    pub fn init(&mut self) {
        
    }

    pub fn begin_logging<PIN, ADC>(
        &mut self,
        adc: &mut Adc<ADC>,
        pin: &mut PIN,
        sd: &mut SdLogger,
        clock: &mut InternalClock,
    ) where
        PIN: Channel<ADC, ID = u8> + Analog,
        ADC: stm32f4xx_hal::adc::Instance,
    {
        let mut ema = 0.0_f32;
        let alpha = 0.1;
        let start_time = clock.get_time_ms();
        loop {
            let now = clock.get_time_ms();
            if now - start_time > 120_000 {
                break;
            }
            let raw: u16 = adc.read(pin).unwrap();
            let raw_f32 = raw as f32;
            ema = alpha * raw_f32 + (1.0 - alpha) * ema;
            let line = format!("{},{},{}\n", now, raw, ema);
            sd.log_data(&line);
            clock.delay_ms(10);
        }
    }

}

pub struct ImuSensor;

pub struct AccelData {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl ImuSensor {
    pub fn init<D: DelayMs<u16>>(&mut self, _delay: &mut D) -> Result<(), ()> {
        // Init IMU via I2C
        Ok(())
    }

    pub fn get_accel(&mut self) -> Result<AccelData, ()> {
        // Read acceleration
        Ok(AccelData { x: 0.0, y: 0.0, z: 35.0 }) // Stub
    }
}
