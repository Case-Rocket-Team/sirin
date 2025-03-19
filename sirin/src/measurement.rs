use serde::{Serialize, Deserialize};
use postcard::{from_bytes, to_eio};
use uunit::*;

#[derive(Serialize, Deserialize, Debug)]
struct BaroMeasurement {
    pressure: f64,
    temperature: f64,
}

fn write_measurement() {
    todo!()
}
