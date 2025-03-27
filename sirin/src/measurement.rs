use bmp3::Bmp3Readout;
use serde::{Serialize, Deserialize};
use embedded_io::{Write, Read};
use postcard::{from_eio, to_eio};
use uunit::*;
use defmt::debug;
use crate::Sirin;


#[derive(Debug, Clone, Serialize, Deserialize)]
#[repr(u8)]
pub enum Measurement {
    Baro(Bmp3Readout)
}
