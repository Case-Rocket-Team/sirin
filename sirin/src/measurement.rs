use serde::{Serialize, Deserialize};
use embedded_io::{Write, Read};
use postcard::{from_eio, to_eio};
use uunit::*;
use defmt::debug;


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

#[derive(Serialize, Deserialize, Debug, PartialEq)]
struct BaroMeasurement {
    pressure: f64,
    temperature: f64,
}



pub fn write_measurement() {

    let test_pressure: f64 = 1.9937;
    let test_temperature: f64 = 122.37;
    
    let mut buffer: [u8; 32] = [0; 32];
    let mut writer: &mut [u8] = &mut buffer;
    let ser = to_eio(&true, &mut writer).unwrap();

    to_eio(&BaroMeasurement {
        pressure: test_pressure,
        temperature: test_temperature,
    }, ser).unwrap();


    debug!("serialized: {}", buffer);
    let mut reader: &[u8] = &buffer;
    
    let output: BaroMeasurement = from_eio((reader, &mut buffer)).unwrap();

}
