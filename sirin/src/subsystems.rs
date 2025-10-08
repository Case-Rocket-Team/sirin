use core::{error::Error, fmt::Debug, future::Future, ops::{Deref, DerefMut}};

use bmp3::Bmp3;
use defmt::Str;
use embassy_time::Instant;
use embedded_hal::spi::ErrorKind as SpiErrorKind;
use h3lis::H3lis;
use lsm6dso_spi::{Accel, AngularVel, Lsm6dso};
use paste::paste;
use rfm9::Rfm9;
use sirin_macros::Measurement;
use sirin_shared::packet::{BaroData, HighGImuData, ImuData, Measurement, SirinData, SubsystemError, Vec3};
use snafu::prelude::*;
use uunit::{Celsius, Milliseconds, Pascals, WithUnits};
use w25qx::W25Q;
use lis3mdl::Lis3mdl;

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

/*
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
}*/

macro_rules! sanity_check {
    ($test:expr => $($pattern:tt)*) => {{
        const ERR_MSG: &str = concat!("Sanity check failed: ", stringify!($test), " not in ", stringify!($($pattern)*));

        let value = $test;

        Result::<_, SpiErrorKind>::Ok(value)
    }};
}

macro_rules! sanity_check_uunit {
    ($test:expr => $($pattern:tt)*) => {{
        const ERR_MSG: &str = concat!("Sanity check failed: ", stringify!($test), " not in ", stringify!($($pattern)*));

        let value = $test;

        Ok(value)
    }};
}

impl Subsystem for Bmp3<SpiDev> {
    const NAME: &str = "Barometer";
    const PART: &str = "BMP388";

    async fn selfcheck(&mut self) -> Result<(), SubsystemError> {
        Ok(())
    }
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

impl Instrument for Lsm6dso<SpiDev> {
    type Data = ImuData;

    async fn measure(&mut self) -> Self::Data {
        Self::Data {
            accel: self.accel().await.map(|accel| Vec3 { x: accel.x, y: accel.y, z: accel.z }).map_err(|e| e.into()),
            angular_vel: self.angular_vel().await.map(|vel| Vec3 { x: vel.x_pitch, y: vel.y_roll, z: vel.z_yaw }).map_err(|e| e.into())
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

impl Instrument for H3lis<SpiDev> {
    type Data = HighGImuData;

    async fn measure(&mut self) -> Self::Data {
        match self.magnetic().await.map_err(|e| e.into()) {
            Ok(accel) => {
                Self::Data {
                    accel: Ok(Vec3 {
                        x: accel.0,
                        y: accel.1,
                        z: accel.2
                    })
                }
            },
            Err(e) => {
                Self::Data {
                    accel: Err(e)
                }
            }
        }

    }
}

impl Subsystem for Lis3mdl<SpiDev> {
    const NAME: &str = "Magnetometer";
    const PART: &str = "LIS3MDL";

    async fn selfcheck(&mut self) -> Result<(), SubsystemError> {
        let manufacturer_id = self.manufacturer_id().await?;

        sanity_check!(manufacturer_id => 50)?;
        Ok(())
    }
}

impl Instrument for Lis3mdl<SpiDev> {
    type Data = MagnetometerData;

    async fn measure(&mut self) -> Self::Data {
        Self::Data{
            mag: self.magnetic().await.map(|mag| Vec3 { x: mag.x, y: mag.y, z: mag.z }).map_err(|e| e.into()),
            temp: self.temp().await.map(|t: i16| t as i16).map_err(|e| e.into())
        }
        
    }
}

impl Subsystem for Rfm9<SpiDev> {
    const NAME: &str = "LoRa Radio";
    const PART: &str = "Rfm9";

    async fn selfcheck(&mut self) -> Result<(), SubsystemError> {
        let radio_version = self.read_version().await.unwrap();

        sanity_check!(radio_version => 18)?;
        Ok(())
    }
}

pub async fn measure_sirin(
    baro: &mut Bmp3<SpiDev>,
    imu: &mut Lsm6dso<SpiDev>,
    high_g_imu: &mut H3lis<SpiDev>,
    magnetometer: &mut Lis3mdl<SpiDev>
) -> SirinData {
    //TODO: join futures?
    //TODO: Measure magnetometer data here as well
    SirinData {
        time: (Instant::now().as_millis() as u32).with_units(),
        baro: baro.measure().await,
        imu: imu.measure().await,
        high_g_imu: high_g_imu.measure().await,
        magnetometer: magnetometer.measure().await
    }
}