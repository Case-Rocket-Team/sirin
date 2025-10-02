
mod actuators;
mod clock;
mod sensors;
mod storage;

use actuators::{ServoController, Camera};
use sensors::{PiezoSensor, ImuSensor};
use clock::InternalClock;
use storage::SdLogger;
use embedded_hal::blocking::delay::DelayMs;



pub struct FlightComputer {
    servos: ServoController,
    camera: Camera,
    piezo: PiezoSensor,
    clock: InternalClock,
    sd: SdLogger,
    imu: ImuSensor,
}

impl FlightComputer {
    pub fn new(
        servos: ServoController,
        camera: Camera,
        piezo: PiezoSensor,
        clock: InternalClock,
        sd: SdLogger,
        imu: ImuSensor,
    ) -> Self {
        Self {
            servos,
            camera,
            piezo,
            clock,
            sd,
            imu,
        }
    }

    pub fn initialize_all<D: DelayMs<u16>>(&mut self, delay: &mut D) {
        self.servos.init(delay);
        self.camera.init();
        self.piezo.init();
        self.clock.init();
        self.sd.init().unwrap();
        self.imu.init(delay).unwrap();
    }

    pub fn enter_standby(&mut self) {
        let mut count = 0;

        loop {
            let accel = self.imu.get_accel().unwrap();

            if accel.z > 30.0 {
                count += 1;
            } else {
                count = 0;
            }

            self.clock.delay_ms(50);

            if count >= 10 {
                break;
            }
        }

        self.start_experiment();
    }

    fn start_experiment(&mut self) {
        self.camera.start_recording();
        self.servos.unfold_sheet();
        self.piezo.begin_logging(&mut self.sd, &self.clock);

        self.camera.stop_recording();
        

    }

//     fn finish_logging<T: embedded_sdmmc::BlockDevice>(
//         controller: &mut Controller<T>,
//         volume: &Volume,
//         filename: &str,
//     ) {
//         let root_dir = controller.open_root_dir(volume).unwrap();
//         let mut file = controller.open_file_in_dir(volume, &root_dir, filename, Mode::ReadWriteAppend).unwrap();
    
//         controller.flush_file(&mut file).unwrap(); 
    
//         controller.close_file(file).unwrap();      
//     }
}
