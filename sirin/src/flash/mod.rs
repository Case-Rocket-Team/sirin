#![doc = include_str!("README.md")]

use core::{mem::MaybeUninit, ops::{Add, Neg, Rem, Sub}};

use cyclic::{AppendResult, CyclicFlashSection};
use defmt::{println, trace, Display2Format};
use embassy_time::Instant;
use heapless::Deque;
use sirin_shared::{packet::{FlightHeader, FlightHeaderStatus, OutPacket, MAX_OUT_PACKET_SIZE}, song::{FromSong, SongSize, ToSong}, time::AbsoluteTimeReference};
use sirin_shared::song::ConstSongSize;
use static_assertions::const_assert;
use w25qx::W25Q;

use crate::{error::SirinError, spi::SpiDev, time::absolute_time_reference, SirinConfig};

mod cyclic;

const FLASH_SIZE: u32 = 16777216;
const SECTOR_SIZE: u32 = 4096;
const FLIGHT_HEADER_COUNT: usize = 256;

const_assert!(FLIGHT_HEADER_COUNT / 2 < SECTOR_SIZE as usize / FlightHeader::SONG_SIZE);

pub struct Flash {
    pub w25q: W25Q<SpiDev>,
    flight_data: CyclicFlashSection,
    flight_headers: Deque<FlashFlightHeader, FLIGHT_HEADER_COUNT>,
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
            flight_data: CyclicFlashSection::new(4 * SECTOR_SIZE, FLASH_SIZE),
            flight_headers: Deque::new(),
            is_second_config: true,
        }
    }

    pub(crate) async fn debug(&mut self) -> Result<(), SirinError> {
        let mut buf = [0u8; 512];

        self.w25q.read_data(0, &mut buf).await?;
        println!("First sector: {:x}", buf[0..256]);

        self.w25q.read_data(4096, &mut buf).await?;
        println!("Second sector: {:x}", buf[0..256]);

        Ok(())
    }

    pub(crate) async fn init(&mut self) -> Result<SirinConfig, SirinError> {
        self.flight_data.init(&mut self.w25q).await?;

        self.debug().await?;

        // Read config
        let mut buf = [0u8; 512];
        self.is_second_config = true;

        self.w25q.read_data(4096, &mut buf).await?;

        if buf[0] != 0x77 {
            self.is_second_config = false;
            self.w25q.read_data(0, &mut buf).await?;
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
                let _ = self.erase_flight_headers().await;
            }
        }

        Ok(config)
    }

    pub fn flight_headers_deque(&self) -> &Deque<FlashFlightHeader, FLIGHT_HEADER_COUNT> {
        &self.flight_headers
    }

    pub fn flight_headers(&self) -> impl ExactSizeIterator<Item = &FlightHeader> {
        self.flight_headers_deque().into_iter().map(|h| &h.header)
    }

    async fn read_flight_header(&mut self, index: usize) -> Result<FlightHeader, SirinError> {
        let mut buf = [0u8; FlightHeader::SONG_SIZE];
        self.w25q.read_data(flight_header_index_to_addr(index), &mut buf).await?;
        // todo
        Ok(FlightHeader::from_song(&buf)?)
    }

    async fn read_flight_header_status(&mut self, index: usize) -> Result<FlightHeaderStatus, SirinError> {
        let mut buf = [0u8; 1];
        self.w25q.read_data(flight_header_index_to_addr(index), &mut buf).await?;
        // todo
        Ok(FlightHeaderStatus::from_song(&buf).unwrap_or(FlightHeaderStatus::Overwritten))
    }

    async fn init_flight_headers(&mut self) -> Result<(), SirinError> {
        let mut start = None;

        // TODO: optimize. We can read out multiple headers in a single read
        // instead of using so many. Each read has 4 bytes overhead

        // We need to add these the the deque in order, so first scan for the start.
        let mut prev = self.read_flight_header_status(0).await?;
        for i in 1..=FLIGHT_HEADER_COUNT {
            let curr = self.read_flight_header_status(i % FLIGHT_HEADER_COUNT).await?;

            if prev != FlightHeaderStatus::Valid
                && curr == FlightHeaderStatus::Valid
            {
                // This is the start!
                start = Some(i % FLIGHT_HEADER_COUNT);
                break;
            }

            prev = curr;
        }

        let start = start.unwrap_or(0);

        for i in 0..FLIGHT_HEADER_COUNT {
            let header = self.read_flight_header((start + i) % FLIGHT_HEADER_COUNT).await?;

            if header.status != FlightHeaderStatus::Valid {
                // We've reached the end of valid headers.
                break;
            }

            let _ = self.flight_headers.push_back(FlashFlightHeader {
                index: (start + i) % FLIGHT_HEADER_COUNT,
                header
            });
        }

        Ok(())
    }

    pub async fn erase_flight_headers(&mut self) -> Result<(), SirinError> {
        self.w25q.sector_erase(SECTOR_SIZE * 2).await?;
        self.w25q.sector_erase(SECTOR_SIZE * 3).await?;
        Ok(())
    }

    async fn invalidate_flight_header(&mut self, index: usize) -> Result<(), SirinError> {
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

        let addr = flight_header_index_to_addr(header.index);

        if addr % SECTOR_SIZE == 0 {
            self.w25q.sector_erase(addr).await?;
        }

        let mut buf = [0u8; FlightHeader::SONG_SIZE];
        header.header.to_song(&mut buf).unwrap();
        self.w25q.write(addr, &buf).await?;

        if self.flight_headers.is_full() {
            self.flight_headers.pop_front();
        }

        self.flight_headers.push_back(header).unwrap();

        Ok(&self.flight_headers.back().unwrap().header)
    }

    pub async fn log(&mut self, packet: &OutPacket) -> Result<(), SirinError> {
        let mut data = [0u8; MAX_OUT_PACKET_SIZE];
        packet.to_song(&mut data).unwrap();
        let AppendResult { sector_erased, .. } = self.flight_data.append(&mut self.w25q, &data).await?;

        // Check if we just overwrote an old log. If we did, invalidate it.
        if let Some(sector) = sector_erased {
            loop {
                // use a code block to drop the borrow on the deque so we can pop later
                let index = {
                    let Some(FlashFlightHeader { index, .. }) = self.flight_headers.front() else {
                        break;
                    };

                    *index
                };

                let addr = flight_header_index_to_addr(index);

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

    pub async fn save_config(&mut self, config: &SirinConfig) -> Result<(), SirinError> {
        let mut buf = [0u8; 512];
        buf[0] = 0x77;
        // TODO remove unwrap
        config.to_song(&mut buf[1..]).unwrap();

        if self.is_second_config {
            self.w25q.sector_erase(0).await?;
            self.w25q.write(0, &buf[0..config.song_size() + 1]).await?;
            self.w25q.sector_erase(4096).await?;
        } else {
            self.w25q.sector_erase(4096).await?;
            self.w25q.write(4096, &buf[0..config.song_size() + 1]).await?;
            self.w25q.sector_erase(0).await?;
        }

        Ok(())
    }
}

fn flight_header_index_to_addr(i: usize) -> u32 {
    let i = i as u32;
    let headers_per_sector = FLIGHT_HEADER_COUNT as u32 / 2;

    2 * SECTOR_SIZE + i / headers_per_sector * SECTOR_SIZE + (i as u32 % headers_per_sector) * FlightHeader::SONG_SIZE as u32
}

// The fact the numbers are near their u32 max makes this difficult.
/// a minus b, accounting for the fact that it may be wrapped around
fn wrapping_difference(a: u32, b: u32, max: u32) -> i32 {
    if a >= b {
        // two possibilities
        let x = a - b;
        let y = max - a + b;

        if x < y {
            x as i32
        } else {
            -(y as i32)
        }
    } else {
        -wrapping_difference(b, a, max)
    }
}