#![doc = include_str!("README.md")]

use core::{future::{poll_fn, Future, PollFn}, mem::MaybeUninit, ops::{Add, Neg, Rem, Sub}};

use cyclic::{AppendResult, CyclicFlashSection};
use defmt::{error, info, println, trace, Debug2Format, Display2Format};
use embassy_time::{Instant, Timer};
use sirin_shared::{packet::{FlightHeader, FlightHeaderStatus, LogEntry, OutPacket, MAX_OUT_PACKET_SIZE}, song::{maybe_unwritten_max_bytes::MaybeUnwrittenMaxBytes, FromSong, SongSize, ToSong}, time::AbsoluteTimeReference};
use sirin_shared::song::ConstSongSize;
use static_assertions::const_assert;
use w25qx::W25Q;

use crate::{deque::Deque, error::SirinError, spi::SpiDev, time::absolute_time_reference, SirinConfig};

mod cyclic;

const FLASH_SIZE: u32 = 16777216;
const SECTOR_SIZE: u32 = 4096;
const FLIGHT_HEADER_COUNT: usize = 256;

// GPS navigation database (UBX-MGA-DBD dump), stored in two A/B slots at the
// end of flash so a power cut mid-save never loses the previous copy.
const GPS_NAV_DB_SLOT_SIZE: u32 = 8 * SECTOR_SIZE;
const GPS_NAV_DB_REGION_START: u32 = FLASH_SIZE - 2 * GPS_NAV_DB_SLOT_SIZE;
// Data starts on a page boundary (w25q.write() needs page-aligned addresses)
const GPS_NAV_DB_DATA_OFFSET: u32 = 256;
pub const GPS_NAV_DB_MAX_LEN: usize = (GPS_NAV_DB_SLOT_SIZE - GPS_NAV_DB_DATA_OFFSET) as usize;
const GPS_NAV_DB_MAGIC: u32 = u32::from_le_bytes(*b"GDB1");
const GPS_NAV_DB_HEADER_LEN: usize = 32;

const_assert!(FLIGHT_HEADER_COUNT / 2 < SECTOR_SIZE as usize / FlightHeader::SONG_SIZE);

pub struct Flash {
    pub w25q: W25Q<SpiDev>,
    flight_data: CyclicFlashSection,
    pub flight_headers: Deque<FlashFlightHeader, FLIGHT_HEADER_COUNT>,
    is_second_config: bool,
}

#[derive(Clone, Debug)]
pub struct FlashFlightHeader {
    pub index: usize,
    pub header: FlightHeader,
}

impl Flash {
    /// NB. Must call `init()`` after calling `new()`
    #[allow(static_mut_refs)]
    pub(crate) unsafe fn new(w25q: W25Q<SpiDev>) -> Self {
        Self {
            w25q,
            flight_data: CyclicFlashSection::new(4 * SECTOR_SIZE, GPS_NAV_DB_REGION_START),
            flight_headers: Deque::new(),
            is_second_config: true,
        }
    }

    pub(crate) async fn debug(&mut self) -> Result<(), SirinError> {
        let reg1 = self.w25q.read_status_reg1().await.unwrap();
        let reg2 = self.w25q.read_status_reg2().await.unwrap();
        let reg3 = self.w25q.read_status_reg3().await.unwrap();

        info!("W25Q32 Status Registers: 1[{:b}], 2[{:b}], 3[{:b}]", reg1, reg2, reg3);

        let mut buf = [0u8; 512];

        self.w25q.read(0, &mut buf).await?;
        println!("First sector: {:x}", buf[0..256]);

        self.w25q.read(4096, &mut buf).await?;
        println!("Second sector: {:x}", buf[0..256]);

        Ok(())
    }

    pub(crate) async fn init(&mut self) -> Result<SirinConfig, SirinError> {
        self.flight_data.init(&mut self.w25q).await?;

        self.debug().await?;

        // Read config
        let mut buf = [0u8; 512];
        self.is_second_config = true;

        self.w25q.read(4096, &mut buf).await?;

        if buf[0] != 0x77 {
            self.is_second_config = false;
            self.w25q.read(0, &mut buf).await?;
        }

        if buf[0] != 0x77 {
            trace!("Couldn't read config. Filling in with default.");
            let config = SirinConfig::default();
            self.save_config(&config).await?;

            return Ok(config)
        }

        // First byte will just be 0x77 to specify that it isn't erased
        let config = SirinConfig::from_song(&buf[1..]).unwrap_or_default();

        println!("{}", Display2Format(&config));

        match self.init_flight_headers().await {
            Ok(()) => {},
            Err(_) => {
                error!("Failed to init flight headers. Erasing.");
                self.flight_headers = Deque::new();
                self.erase_flight_headers().await?;
            }
        }

        self.new_flight().await?;

        Ok(config)
    }

    async fn init_flight_headers(&mut self) -> Result<(), SirinError> {
        for i in 0..FLIGHT_HEADER_COUNT {
            let mut buf = [0u8; FlightHeader::SONG_SIZE];
            self.w25q.read(flight_header_index_to_addr(i), &mut buf).await?;
            
            if buf[0] == FlightHeaderStatus::Valid as u8 {
                // the `set` method will take care of setting up the deque
                // for us
                self.flight_headers.set(i, FlashFlightHeader {
                    index: i,
                    header: FlightHeader::from_song(&buf)?
                }).map_err(|_| SirinError::CorruptedData)?;
            }
        }

        info!("Read these flight headers: {:#}", Debug2Format(&self.flight_headers));

        Ok(())
    }

    pub async fn erase_flight_headers(&mut self) -> Result<(), SirinError> {
        info!("Erasing flight headers");
        self.w25q.checked_erase_sector(SECTOR_SIZE * 2).await?;
        self.w25q.checked_erase_sector(SECTOR_SIZE * 3).await?;
        Ok(())
    }

    async fn invalidate_flight_header(&mut self, index: usize) -> Result<(), SirinError> {
        info!("Invalidating flight header {}", index);
        self.w25q.write(flight_header_index_to_addr(index), &[0]).await?;
        Ok(())
    }

    pub async fn new_flight(&mut self) -> Result<&FlightHeader, SirinError> {
        let header = FlashFlightHeader {
            index: self.flight_headers.back().map(|h| (h.index + 1) % FLIGHT_HEADER_COUNT).unwrap_or(0),
            header: FlightHeader::new(
                self.flight_data.cursor(),
                absolute_time_reference(),
                Instant::now().as_ticks()
            )
        };

        info!("New flight header: {:?}", Debug2Format(&header));

        let addr = flight_header_index_to_addr(header.index);

        if addr % SECTOR_SIZE == 0 {
            info!("Erasing flight header sector {}", addr);
            self.w25q.erase_sector(addr).await?;
            self.w25q.until_ready().await?;
            let mut buf = [0; 256];
            self.w25q.read(addr, &mut buf).await?;
            info!("Erased sector: [{}]", buf);
        }

        let mut buf = [0u8; FlightHeader::SONG_SIZE];
        assert!(header.header.song_size() == FlightHeader::SONG_SIZE);
        header.header.to_song(&mut buf).unwrap();
        self.w25q.checked_write(addr, &buf).await?;
        info!("Wrote flight header to {}", addr);

        if self.flight_headers.is_full() {
            self.flight_headers.pop_front();
        }

        self.flight_headers.push_back(header).unwrap();

        Ok(&self.flight_headers.back().unwrap().header)
    }

    pub fn active_flight_header(&self) -> &FlashFlightHeader {
        self.flight_headers.back().unwrap()
    }

    pub async fn set_absolute_time_reference(&mut self, reference: AbsoluteTimeReference) -> Result<(), SirinError> {
        let header = self.flight_headers.back_mut().unwrap();
        header.header.time_reference = MaybeUnwrittenMaxBytes(Some(reference.clone()));
        let mut buf = [0; FlightHeader::SONG_SIZE];
        header.header.to_song(&mut buf)?;

        self.w25q.write(
            flight_header_index_to_addr(header.index),
            &buf
        ).await?;

        Ok(())
    }

    pub async fn log(&mut self, packet: &OutPacket) -> Result<(), SirinError> {
        let mut data = [0u8; MAX_OUT_PACKET_SIZE];
        packet.to_song(&mut data)?;
        let result = self.flight_data.append(&mut self.w25q, &data[0..packet.song_size()]).await?;

        //info!("Wrote absolute address {}: {:x}", result.addr, &data[0..packet.song_size()]);

        // Check if we just overwrote an old log. If we did, invalidate it.
        if let Some(sector) = result.sector_erased {
            loop {
                if self.flight_headers.len() <= 1 {
                    break;
                }

                // use a code block to drop the borrow on the deque so we can pop later
                let (index, addr) = {
                    let Some(FlashFlightHeader { index, header }) = self.flight_headers.front() else {
                        break;
                    };

                    (*index, header.data_addr() + self.flight_data.region_start)
                };

                // check if it's inside the erased sector
                if sector <= addr && addr < sector + SECTOR_SIZE {
                    self.flight_headers.pop_front();
                    self.invalidate_flight_header(index).await?;
                } else {
                    break;
                }
            }
        }

        Ok(())
    }

    pub fn read_logs(&mut self, header: &FlashFlightHeader) -> FlashFlightPacketsIterator<'_> {
        let start = header.header.data_addr();
        let size = self.flight_data.data_subregion_size;

        // Stop where the next flight's data starts
        let end = match self.flight_headers.get((header.index + 1) % FLIGHT_HEADER_COUNT) {
            Some(next) if !self.flight_headers.back().is_some_and(|b| b.index == header.index) => {
                let next_addr = next.header.data_addr();
                if next_addr >= start {
                    // Equal means this flight logged nothing
                    next_addr
                } else {
                    // Next flight wrapped around the cyclic buffer
                    next_addr + size
                }
            }
            _ => start + size,
        };

        FlashFlightPacketsIterator {
            flash: self,
            rel_addr: start,
            rel_end_addr: end,
            is_done: false
        }
    }

    pub async fn save_config(&mut self, config: &SirinConfig) -> Result<(), SirinError> {
        let mut buf = [0u8; 512];
        buf[0] = 0x77;
        // TODO remove unwrap
        config.to_song(&mut buf[1..]).unwrap();

        if self.is_second_config {
            self.w25q.checked_erase_sector(0).await?;
            self.w25q.checked_write(0, &buf[0..config.song_size() + 1]).await?;
            self.w25q.checked_erase_sector(4096).await?;
        } else {
            self.w25q.checked_erase_sector(4096).await?;
            self.w25q.checked_write(4096, &buf[0..config.song_size() + 1]).await?;
            self.w25q.checked_erase_sector(0).await?;
        }

        Ok(())
    }

    async fn read_gps_nav_db_header(&mut self, slot: u32) -> Result<Option<GpsNavDbHeader>, SirinError> {
        let mut buf = [0u8; GPS_NAV_DB_HEADER_LEN];
        self.w25q.read(gps_nav_db_slot_addr(slot), &mut buf).await?;
        Ok(GpsNavDbHeader::from_bytes(&buf))
    }

    /// Load the newest valid GPS navigation database into `buf`.
    /// Returns its header, or `None` if no valid copy is stored.
    pub async fn load_gps_nav_db(&mut self, buf: &mut [u8]) -> Result<Option<GpsNavDbHeader>, SirinError> {
        let mut slots = [
            (0, self.read_gps_nav_db_header(0).await?),
            (1, self.read_gps_nav_db_header(1).await?),
        ];
        // Newest first
        slots.sort_unstable_by_key(|(_, h)| core::cmp::Reverse(h.map(|h| h.seq)));

        for (slot, header) in slots {
            let Some(header) = header else { continue };
            let len = header.len as usize;
            if len > buf.len() {
                continue;
            }
            self.w25q.read(gps_nav_db_slot_addr(slot) + GPS_NAV_DB_DATA_OFFSET, &mut buf[..len]).await?;
            if fnv1a(&buf[..len]) == header.checksum {
                return Ok(Some(header));
            }
            error!("GPS nav DB slot {} failed its checksum", slot);
        }

        Ok(None)
    }

    /// Save a GPS navigation database, overwriting the older of the two slots.
    pub async fn save_gps_nav_db(&mut self, data: &[u8], frames: u32, pos: GpsNavDbPos) -> Result<(), SirinError> {
        if data.len() > GPS_NAV_DB_MAX_LEN {
            return Err(SirinError::CorruptedData);
        }

        let a = self.read_gps_nav_db_header(0).await?;
        let b = self.read_gps_nav_db_header(1).await?;
        let (slot, newest_seq) = match (a, b) {
            (Some(a), Some(b)) if a.seq >= b.seq => (1, a.seq),
            (Some(_), Some(b)) => (0, b.seq),
            (Some(a), None) => (1, a.seq),
            (None, Some(b)) => (0, b.seq),
            (None, None) => (0, 0),
        };
        let addr = gps_nav_db_slot_addr(slot);

        // Erase only the sectors this copy needs (the header sector is always included)
        let sectors = (GPS_NAV_DB_DATA_OFFSET + data.len() as u32).div_ceil(SECTOR_SIZE);
        for i in 0..sectors {
            self.w25q.erase_sector(addr + i * SECTOR_SIZE).await?;
        }

        // Data first, header last, so a half-written slot never looks valid
        self.w25q.write(addr + GPS_NAV_DB_DATA_OFFSET, data).await?;
        let header = GpsNavDbHeader {
            seq: newest_seq.wrapping_add(1),
            len: data.len() as u32,
            frames,
            checksum: fnv1a(data),
            pos,
        };
        self.w25q.write(addr, &header.to_bytes()).await?;
        self.w25q.until_ready().await?;

        info!("Saved GPS nav DB: slot {}, seq {}, {} frames, {} bytes", slot, header.seq, frames, data.len());
        Ok(())
    }
}

/// Last known position, saved alongside the GPS navigation database
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct GpsNavDbPos {
    /// Latitude in 1e-7 degrees
    pub lat_e7: i32,
    /// Longitude in 1e-7 degrees
    pub lon_e7: i32,
    /// Height above the WGS84 ellipsoid in centimetres
    pub alt_cm: i32,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct GpsNavDbHeader {
    pub seq: u32,
    pub len: u32,
    pub frames: u32,
    pub checksum: u32,
    pub pos: GpsNavDbPos,
}

impl GpsNavDbHeader {
    fn to_bytes(&self) -> [u8; GPS_NAV_DB_HEADER_LEN] {
        let words = [
            GPS_NAV_DB_MAGIC, self.seq, self.len, self.frames, self.checksum,
            self.pos.lat_e7 as u32, self.pos.lon_e7 as u32, self.pos.alt_cm as u32,
        ];
        let mut out = [0u8; GPS_NAV_DB_HEADER_LEN];
        for (chunk, word) in out.chunks_exact_mut(4).zip(words) {
            chunk.copy_from_slice(&word.to_le_bytes());
        }
        out
    }

    fn from_bytes(buf: &[u8; GPS_NAV_DB_HEADER_LEN]) -> Option<Self> {
        let w = |i: usize| u32::from_le_bytes(buf[i * 4..i * 4 + 4].try_into().unwrap());
        if w(0) != GPS_NAV_DB_MAGIC || w(2) as usize > GPS_NAV_DB_MAX_LEN {
            return None;
        }
        Some(Self {
            seq: w(1),
            len: w(2),
            frames: w(3),
            checksum: w(4),
            pos: GpsNavDbPos { lat_e7: w(5) as i32, lon_e7: w(6) as i32, alt_cm: w(7) as i32 },
        })
    }
}

fn gps_nav_db_slot_addr(slot: u32) -> u32 {
    GPS_NAV_DB_REGION_START + slot * GPS_NAV_DB_SLOT_SIZE
}

fn fnv1a(data: &[u8]) -> u32 {
    data.iter().fold(0x811c_9dc5u32, |h, b| (h ^ *b as u32).wrapping_mul(0x0100_0193))
}

pub struct FlashFlightPacketsIterator<'a> {
    flash: &'a mut Flash,

    // Not including offset!
    rel_addr: u32,

    // Exclusive end of this flight's data, not including offset. May exceed the
    // data region size when the flight wraps around; `rel_addr` is unwrapped too.
    rel_end_addr: u32,

    is_done: bool
}

impl <'a> FlashFlightPacketsIterator<'a> {
    pub async fn next(&mut self) -> Option<Result<OutPacket, SirinError>> {
        let size = self.flash.flight_data.data_subregion_size;
        let offset = self.flash.flight_data.region_start;

        let mut buf = [0; MAX_OUT_PACKET_SIZE];

        loop {
            // Checked every pass so a jump to the next sector can't run past the end
            if self.is_done || self.rel_addr >= self.rel_end_addr {
                self.is_done = true;
                return None;
            }

            //info!("Ran iter loop");
            let addr = offset + (self.rel_addr % size);
            match self.flash.w25q.read(addr, &mut buf).await {
                Ok(..) => {},
                Err(e) => return Some(Err(e.into()))
            }

            if buf[0] == 0xFE {
                // Go to next sector
                info!("Going to next sector");
                self.rel_addr = (self.rel_addr / SECTOR_SIZE + 1) * SECTOR_SIZE;
                continue;
            } else if buf[0] == 0xFF {
                // We're done reading.
                self.is_done = true;
                return None
            } else if buf[0] == 0x00 {
                Timer::after_millis(150).await;
                info!("Buf: {:x}", &buf);
                panic!("Couldn't read flight data, addr: {}", addr)
            }

            let log = match OutPacket::from_song(&buf) {
                Ok(p) => p,
                Err(e) => {
                    self.is_done = true;
                    return Some(Err(e.into()))
                }
            };

            self.rel_addr += log.song_size() as u32;
            return Some(Ok(log))
        }
        
    }
}

fn flight_header_index_to_addr(i: usize) -> u32 {
    let i = i as u32;
    let headers_per_sector = FLIGHT_HEADER_COUNT as u32 / 2;

    2 * SECTOR_SIZE + i / headers_per_sector * SECTOR_SIZE + (i as u32 % headers_per_sector) * FlightHeader::SONG_SIZE as u32
}