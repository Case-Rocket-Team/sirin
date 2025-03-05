use binary_layout::prelude::*;
use serde::{Serialize, Deserialize};
use postcard::{from_bytes, to_eio};
use uunit::*;


binary_layout!(baro_measurement, LittleEndian, {
    pressure: f64,
    temperature: f64,
});
binary_layout!(imu1_measurement, LittleEndian, {
    accel_x: i32,
    accel_y: i32,
    accel_z: i32,
    gyro_pitch: i32,
    gyro_roll: i32,
    gyro_yaw: i32,
});
binary_layout!(imu2_measurement, LittleEndian, {
    accel_x: i32,
    accel_y: i32,
    accel_z: i32,
});

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
struct BaroMeasurement {
    pressure: f64,
    temperature: f64,
}



fn write_measurement(data: &mut [u8]) {
    let mut view = baro_measurement::View::new(data);
    let val = view.pressure().read();

    let test_pressure: f64 = 1.237;
    let test_temperature: f64 = 122.37;

    let output = to_eio(&BaroMeasurement {
        pressure: test_pressure,
        temperature: test_temperature,
    }).unwrap();

}
