use sirin_shared::{packet::FlightHeader, song::{ConstSongSize, FromSong}};
use static_assertions::const_assert;
use w25qx::W25Q;

use crate::{error::SirinError, spi::SpiDev};

const SECTOR_SIZE: u32 = 4096;
const PAGE_SIZE: u32 = 256;
const FLIGHT_HEADER_COUNT: u32 = 128;
const FLIGHT_HEADERS_PER_SECTOR: u32 = SECTOR_SIZE / (FlightHeader::SONG_SIZE as u32);

const CONFIG_START_SECTOR: u32 = 0;
const HEADERS_START_SECTOR: u32 = CONFIG_START_SECTOR + 2;
const HEADERS_SECTORS_COUNT: u32 = FLIGHT_HEADER_COUNT.div_ceil(FLIGHT_HEADERS_PER_SECTOR);
const LOGS_START_SECTOR: u32 = HEADERS_START_SECTOR + HEADERS_SECTORS_COUNT;

pub struct FlashFlightHeader {
    pub index: usize,
    pub header: FlightHeader,
}

pub struct Flash {
    pub w25q: W25Q<SpiDev>
}

/// provides `i * size` except taking into account the rule that no object of size `size`
/// should be written across indices
fn sector_aware_index(i: u32, size: u32) -> u32 {
    debug_assert!(size <= SECTOR_SIZE); 
    let items_per_sector: u32 = SECTOR_SIZE / size;

    let sector_num = i / items_per_sector;
    SECTOR_SIZE * sector_num + size * i
}

impl Flash {
    /// Assumes that the item we're searching for is not written across sectors,
    /// but might be written across pages.
    fn binary_search<T>() -> Result<(u32, T), SirinError>
    where T: Ord + FromSong + ConstSongSize
}