use core::ops::Div;
use crate::{config::{CallsignBuf, SirinConfig, SirinId}, mode::SirinMode, song::{magic::MagicU8, maybe_unwritten_max_bytes::MaybeUnwrittenMaxBytes, *}, state::NominalState, time::AbsoluteTimeReference};
use derive_more::Display;
use sirin_macros::*;
use embedded_hal::spi::ErrorKind as SpiErrorKind;

pub const MAX_OUT_PACKET_SIZE: usize = 256;

#[macro_export]
/// Generate a null-terminated string of fixed length
macro_rules! byte_array_str {
    ($len: expr, $bytestring: expr) => {
        {
            let mut bytes = [0u8; $len];
            let str = $bytestring;
            bytes[0..(str.len())].copy_from_slice(str);
            bytes
        }
    };
}

pub use byte_array_str;
use snafu::Snafu;
use uunit::{Celsius, Meters, MetersPerSecond, MicroGs, Milliseconds, Pascals, Quantity, UnitMicrodegrees, UnitSeconds, WithUnits};

#[derive(Debug)]
pub enum ByteArrayStrError {
    /// Str is too long for the buffer
    TooLong
}

pub trait ByteArrayStr: Sized {
    /// Returns a byte array as `&str`, truncated before the first non-ascii or null byte.
    fn as_str(&self) -> &str;
    fn null_terminate(&self) -> &[u8];
    fn from_str(str: &str) -> Result<Self, ByteArrayStrError>;
}

impl <const SIZE: usize> ByteArrayStr for [u8; SIZE] {
    fn as_str(&self) -> &str {
        for i in 0..self.len() {
            if  self[i] == 0 || !self[i].is_ascii() {
                unsafe {
                    return core::str::from_utf8_unchecked(&self[0..i])
                }
            }
        }

        unsafe {
            return core::str::from_utf8_unchecked(self)
        }
    }

    fn null_terminate(&self) -> &[u8] {
        let mut len = 0;
        while len < self.len() && self[len] != 0 {
            len += 1;
        }
    
        &self[..len]
    }

    fn from_str(str: &str) -> Result<Self, ByteArrayStrError> {
        let mut buf = [0; SIZE];

        if str.len() > SIZE {
            Err(ByteArrayStrError::TooLong)
        } else {
            buf[0..str.len()].copy_from_slice(str.as_bytes());
            Ok(buf)
        }
    }
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[song(discriminant(OutPacketType = u8))]
pub enum OutPacket {
    Null,
    Ok,
    Error(PacketError),
    Config(SirinConfig),
    Mode(SirinMode),
    FlightStart(u8),
    LogEntry(LogEntry),
    FlightHeader(Page<FlightHeader>),
    State(SirinState),
    DeployedApoAt(u32),
    DeployedMainAt(u32)
}

#[derive(Debug, Clone, SongSize, FromSong, ToSong)]
pub struct SirinState {
    pub mode: SirinMode,
    pub gps: GpsFix,
    pub altitude: Meters<f64>,
    pub apogee: Option<Meters<f64>>
}

impl Default for SirinState {
    fn default() -> Self {
        Self {
            mode: SirinMode::Standby,
            gps: GpsFix::default(),
            altitude: 0.0.with_units(),
            apogee: None
        }
    }
}

#[derive(Debug, Clone, SongSize, FromSong, ToSong)]
pub struct Vec3<T: SongSize + FromSong + ToSong> {
    pub x: T,
    pub y: T,
    pub z: T
}

pub type EcefPos<T> = Vec3<Meters<T>>;
pub type EcefVel<T> = Vec3<MetersPerSecond<T>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, SongSize, FromSong, ToSong)]
#[repr(u8)]
pub enum GpsFixType {
    NoFix = 0,
    FixPrediction,
    Fix2d,
    Fix3d,
    FixDifferential,
    TimeOnlyFix
}

#[derive(Debug, Clone, SongSize, FromSong, ToSong)]
pub struct GpsFix {
    pub time: Milliseconds<u32>,
    pub satellites: u8,
    pub almanac: u8,
    pub ephemerides: u8,
    pub healthy_satellites: u8,
    pub fix_type: GpsFixType,
    pub pos: Vec3<Meters<f64>>,
    pub vel: Vec3<MetersPerSecond<f32>>,
}

impl Default for GpsFix {
    fn default() -> Self {
        GpsFix {
            time: 0u32.with_units(),
            satellites: 0,
            almanac: 0,
            ephemerides: 0,
            healthy_satellites: 0,
            fix_type: GpsFixType::NoFix,
            pos: Vec3 {
                x: 0.0.with_units(),
                y: 0.0.with_units(),
                z: 0.0.with_units()
            },
            vel: Vec3 {
                x: 0.0f32.with_units(),
                y: 0.0f32.with_units(),
                z: 0.0f32.with_units()
            },
        }
    }
}

#[derive(Debug, Clone, SongSize, FromSong, ToSong)]
pub struct Page<T: SongSize + ToSong + FromSong> {
    pub index: u16,
    pub data: T
}

impl <T: SongSize + ToSong + FromSong> Page<T> {
    pub fn new(index: u16, data: T) -> Self {
        Page {
            index,
            data
        }
    }
}

#[derive(Debug, Display, Clone, PartialEq, Eq, SongSize, ToSong, FromSong)]
#[song(discriminant(PacketErrorType = u8))]
pub enum PacketError {
    #[display("The packet type {_1:?} is not supported over {_0:?}.")]
    PacketNotSupportedOverChannel(IoChannel, InPacketType),

    #[display("Flight #{_0} could not be found.")]
    FlightNotFound(u16),
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]

pub struct FlashPageDump {
    addr: u32,
    data: [u8; 256]
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[song(discriminant(InPacketType = u8))]
pub enum InPacket {
    Null,
    Ping,
    SetTime(AbsoluteTimeReference),
    Reboot,
    QueryConfig,
    SetConfig(SirinConfig),
    QueryMode,
    SetMode(SirinMode),
    QueryFlights,
    ReadFlight(u16),
    Tail(bool),
    EraseFlash(MagicU8<0xA8>),
    DeployMain,
    DeployApo,
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct LogEntry {
    pub time: Milliseconds<u32>,
    pub log: Log
}

impl LogEntry {
    pub fn new(time: Milliseconds<u32>, log: Log) -> Self {
        Self {
            time,
            log
        }
    }
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
#[song(discriminant(LogDataType = u8))]
pub enum Log {
    //State(NominalState),
    State(SirinState),
    Data(SirinData),
    BarometricAltitude(Meters<f64>),
    GpsNmea([u8; 200])
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct SirinData {
    pub time: Milliseconds<u32>,
    pub baro: BaroData,
    pub imu: ImuData,
    pub high_g_imu: HighGImuData,
    pub magnetometer: MagnetometerData
}

pub trait Measurement {
    fn unmeasured() -> Self;
}

impl Measurement for SirinData {
    fn unmeasured() -> Self {
        Self {
            time: 0u32.with_units(),
            baro: BaroData::unmeasured(),
            imu: ImuData::unmeasured(),
            high_g_imu: HighGImuData::unmeasured(),
            magnetometer: MagnetometerData::unmeasured()
        }
    }
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong, Measurement)]
#[allow(dead_code)]
pub struct BaroData {
    pub pressure: Result<Pascals<f64>, SubsystemError>,
    pub temperature: Result<Celsius<f64>, SubsystemError>
}

type MicrodegreesPerSecond<T> = Quantity<T, <UnitMicrodegrees as Div<UnitSeconds>>::Output>;
#[derive(Debug, Clone, SongSize, ToSong, FromSong, Measurement)]
pub struct ImuData {
    pub accel: Result<Vec3<MicroGs<i32>>, SubsystemError>,
    pub angular_vel: Result<Vec3<MicrodegreesPerSecond<i64>>, SubsystemError>
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong, Measurement)]
#[allow(dead_code)]
pub struct HighGImuData {
    // TODO: Put units on this!
    pub accel: Result<Vec3<i32>, SubsystemError>
}

#[derive(Debug, Clone, SongSize, ToSong, FromSong, Measurement)]
#[allow(dead_code)]
pub struct MagnetometerData {
    pub mag: Result<Vec3<i16>, SubsystemError>,
    pub temp: Result<i16, SubsystemError>
}

// TODO: maybe change to `derive_more` crate and remove snafu
#[derive(Debug, Clone, Copy, Snafu, SongSize, ToSong, FromSong)]
#[repr(u8)]
pub enum SubsystemError {
    #[snafu(display("Sanity check failed"))]
    SanityCheckFailed/*{
        error_msg: &'static str,
        // lazy but w/e -- just convert all numeric types into f64
        // making a different type for each numeric/making the entire error enum
        // generic is too much of a pita.
        value: Option<f64>
    }*/,
    #[snafu(display("Error in SPI bus"))]
    SpiError,
    #[snafu(display("Not measured -- call .measure()"))]
    NotYetMeasured
}

impl From<SpiErrorKind> for SubsystemError {
    fn from(value: SpiErrorKind) -> Self {
        SubsystemError::SpiError
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, SongSize, ToSong, FromSong)]
#[repr(u8)]
pub enum IoChannel {
    Broadcast = 0,
    Usb,
    ToLoRa,
    FromLoRa,
    Flash,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IoPacket<P: SongSize + ToSong + FromSong> {
    pub channel: IoChannel,
    pub packet: P
}

impl <P: SongSize + ToSong + FromSong> IoPacket<P> {
    pub fn new(channel: IoChannel, packet: P) -> Self {
        Self {
            channel,
            packet
        }
    }

    /// TODO: implement full request-response with packet ids
    pub fn reply<R: SongSize + ToSong + FromSong>(&self, packet: R) -> IoPacket<R> {
        IoPacket::new(self.channel, packet)
    }
}
impl core::error::Error for SubsystemError {}

#[derive(Debug, Clone, SongSize, ToSong, FromSong)]
pub struct RadioPacket<P: SongSize + ToSong + FromSong> {
    pub id: SirinId,
    pub callsign: CallsignBuf,
    pub packet: P
}

impl <P: SongSize + ToSong + FromSong> RadioPacket<P> {
    pub fn new(config: &SirinConfig, packet: P) -> Self {
        let callsign = config.callsign.clone();

        Self {
            id: config.id,
            callsign,
            packet
        }
    }
}

#[derive(Clone, Debug, SongSize, ToSong, FromSong)]
pub struct FlightHeader {
    /// Tracking byte -- when this flight is overwritten in the cyclic flash, this byte is zeroed out.
    /// This field must be first!
    pub status: FlightHeaderStatus,

    /// Address of the start of flight logs, without offset.
    addr: [u8; 3],

    pub time_reference: MaybeUnwrittenMaxBytes<AbsoluteTimeReference>,

    /// Time of the flight in terms of ticks since boot
    pub timestamp: u64
}

impl FlightHeader {
    pub fn new(addr: u32, time_reference: Option<AbsoluteTimeReference>, timestamp: u64) -> Self {
        let arr = addr.to_le_bytes();

        Self {
            status: FlightHeaderStatus::Valid,
            addr: [arr[0], arr[1], arr[2]],
            time_reference: MaybeUnwrittenMaxBytes(time_reference),
            timestamp
        }
    }

    pub fn data_addr(&self) -> u32 {
        let arr = [self.addr[0], self.addr[1], self.addr[2], 0];
        u32::from_le_bytes(arr)
    }
}

#[derive(Clone, Copy, Debug, SongSize, ToSong, FromSong, PartialEq, Eq)]
#[repr(u8)]
pub enum FlightHeaderStatus {
    // Initial value of NOR flash is 0xFF
    Null = 0xFF,
    Valid = 0x01,
    Overwritten = 0x00
}