use core::{error::Error, fmt::Debug, future::Future, ops::{Deref, DerefMut}};

use bmp3::Bmp3;
use defmt::Str;
use embedded_hal::spi::ErrorKind as SpiErrorKind;
use h3lis::H3lis;
use lsm6dso::{Accel, AngularVel, Lsm6dso};
use paste::paste;
use rfm9x::Rfm9x;
use sirin_macros::Measurement;
use snafu::prelude::*;
use uunit::{Celsius, Pascals, WithUnits};
use w25q::W25Q;

use crate::spi::SpiDev;

pub trait Subsystem {
    const NAME: &str;
    const PART: &str;

    fn selfcheck(&mut self) -> impl Future<Output = Result<(), SubsystemError>>;
}

pub trait Instrument: Subsystem {
    type Data: Debug + Clone;

    fn measure(&mut self) -> impl Future<Output = Self::Data>;
}

pub trait Measurement {
    fn unmeasured() -> Self;
}

#[derive(Debug, Clone, Snafu)]
pub enum SubsystemError {
    #[snafu(display("{error_msg}"))]
    SanityCheckFailed{
        error_msg: &'static str,
        // lazy but w/e -- just convert all numeric types into f64
        // making a different type for each numeric/making the entire error enum
        // generic is too much of a pita.
        value: Option<f64>
    },
    #[snafu(display("Error in SPI bus: {error}"))]
    SpiError{ error: SpiErrorKind },
    #[snafu(display("Not measured -- call .measure()"))]
    NotYetMeasured
}

macro_rules! sanity_check {
    ($test:expr => $($pattern:tt)*) => {{
        const ERR_MSG: &str = concat!("Sanity check failed: ", stringify!($test), " not in ", stringify!($($pattern)*));

        let value = $test;

        match value {
            $($pattern)* => Ok(value),
            _ => SanityCheckFailedSnafu {
                error_msg: ERR_MSG,
                value: Some(value as f64)
            }.fail()
        }
    }};
}

macro_rules! sanity_check_uunit {
    ($test:expr => $($pattern:tt)*) => {{
        const ERR_MSG: &str = concat!("Sanity check failed: ", stringify!($test), " not in ", stringify!($($pattern)*));

        let value = $test;

        match value.value {
            $($pattern)* => Ok(value),
            _ => SanityCheckFailedSnafu {
                error_msg: ERR_MSG,
                value: Some(value.value as f64)
            }.fail()
        }
    }};
}

impl Error for SubsystemError {}

impl From<SpiErrorKind> for SubsystemError {
    fn from(value: SpiErrorKind) -> Self {
        Self::SpiError { error: value }
    }
}

impl Subsystem for Bmp3<SpiDev> {
    const NAME: &str = "Barometer";
    const PART: &str = "BMP388";

    async fn selfcheck(&mut self) -> Result<(), SubsystemError> {
        Ok(())
    }
}

#[derive(Debug, Clone, Measurement)]
pub struct BaroData {
    pressure: Result<Pascals<f64>, SubsystemError>,
    temperature: Result<Celsius<f64>, SubsystemError>
}

impl Instrument for Bmp3<SpiDev> {
    type Data = BaroData;

    async fn measure(&mut self) -> Self::Data {
        let readout = self.read().await;

        match readout {
            Ok(readout) => {
                Self::Data {
                    pressure: sanity_check_uunit!(readout.pressure => 0.0..),
                    temperature: sanity_check_uunit!(readout.temperature => -300.0..300.0)
                }
            },
            Err(err) => {
                Self::Data {
                    pressure: Err(err.into()),
                    temperature: Err(err.into())
                }
            }
        }
    }
}

impl Subsystem for W25Q<SpiDev> {
    const NAME: &str = "Flash";
    const PART: &str = "W25Q32";

    async fn selfcheck(&mut self) -> Result<(), SubsystemError> {
        let device_id = self.read_device_id().await?;

        sanity_check!(device_id => 21)?;
        Ok(())
    }
}

impl Subsystem for Lsm6dso<SpiDev> {
    const NAME: &str = "IMU";
    const PART: &str = "LSM6DSO32";

    async fn selfcheck(&mut self) -> Result<(), SubsystemError> {
        let manufacturer_id = self.read_manufacturer_id().await?;

        sanity_check!(manufacturer_id => 108)?;
        Ok(())
    }
}

#[derive(Debug, Clone, Measurement)]
pub struct ImuData {
    pub accel: Result<Accel, SubsystemError>,
    pub angular_vel: Result<AngularVel, SubsystemError>
}

impl Instrument for Lsm6dso<SpiDev> {
    type Data = ImuData;

    async fn measure(&mut self) -> Self::Data {
        Self::Data {
            accel: self.accel().await.map_err(|e| e.into()),
            angular_vel: self.angular_vel().await.map_err(|e| e.into())
        }
    }
}

impl Subsystem for H3lis<SpiDev> {
    const NAME: &str = "High G IMU";
    const PART: &str = "H3LIS";

    async fn selfcheck(&mut self) -> Result<(), SubsystemError> {
        let manufacturer_id = self.manufacturer_id().await?;

        sanity_check!(manufacturer_id => 50)?;
        Ok(())
    }
}

#[derive(Debug, Clone, Measurement)]
pub struct HighGImuData {
    // TODO: Put units on this!
    accel: Result<(i32, i32, i32), SubsystemError>
}

impl Instrument for H3lis<SpiDev> {
    type Data = HighGImuData;

    async fn measure(&mut self) -> Self::Data {
        Self::Data {
            accel: self.acceleration().await.map_err(|e| e.into())
        }
    }
}

impl Subsystem for Rfm9x<SpiDev> {
    const NAME: &str = "LoRa Radio";
    const PART: &str = "RFM9x";

    async fn selfcheck(&mut self) -> Result<(), SubsystemError> {
        let radio_version = self.read_version().await.unwrap();

        sanity_check!(radio_version => 18)?;
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct SirinData {
    pub baro: BaroData,
    pub imu: ImuData,
    pub high_g_imu: HighGImuData
}

impl Measurement for SirinData {
    fn unmeasured() -> Self {
        Self {
            baro: BaroData::unmeasured(),
            imu: ImuData::unmeasured(),
            high_g_imu: HighGImuData::unmeasured()
        }
    }
}

impl SirinData {
    pub async fn measure(
        baro: &mut Bmp3<SpiDev>,
        imu: &mut Lsm6dso<SpiDev>,
        high_g_imu: &mut H3lis<SpiDev>
    ) -> Self {
        // TODO: join futures?
        Self {
            baro: baro.measure().await,
            imu: imu.measure().await,
            high_g_imu: high_g_imu.measure().await
        }
    }
}