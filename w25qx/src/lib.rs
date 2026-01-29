#![cfg_attr(not(test), no_std)]

#[allow(unused_imports)]
use core::{future::{poll_fn, Future}, task::Poll};
#[allow(unused_imports)]
use embedded_hal_async::{digital::Wait, spi::ErrorKind};
use spi_handle::SpiHandle;
use embedded_hal_async::spi::SpiBus;

mod yield_now;

pub mod mock;

#[cfg(test)]
mod test;

#[cfg(test)]
fn main() {}

pub struct W25Q<S: SpiHandle>{
    spi: S
}

impl <S: SpiHandle> W25Q<S>{
    pub fn new(spi: S) -> Self {
        Self {
            spi
        }
    }

    pub async fn read_status_reg1(&mut self) -> Result<u8, ErrorKind> {
        let mut spi = self.spi.select().await;
        let mut out = [0u8];
        spi.write(&[0x05]).await?;
        spi.read(&mut out).await?;
        Ok(out[0])
    }

    pub async fn read_status_reg2(&mut self) -> Result<u8, ErrorKind> {
        let mut spi = self.spi.select().await;
        let mut out = [0u8];
        spi.write(&[0x35]).await?;
        spi.read(&mut out).await?;
        Ok(out[0])
    }

    pub async fn read_status_reg3(&mut self) -> Result<u8, ErrorKind> {
        let mut spi = self.spi.select().await;
        let mut out = [0u8];
        spi.write(&[0x15]).await?;
        spi.read(&mut out).await?;
        Ok(out[0])
    }

    pub async fn is_busy(&mut self) -> Result<bool, ErrorKind> {
        Ok(self.read_status_reg1().await? % 2 == 1)
    }

    /// Returns a future that completes when the chip is ready (not BUSY)
    pub async fn until_ready(&mut self) -> Result<(), ErrorKind> {
        while self.is_busy().await? {
            yield_now::yield_now().await;
        }

        Ok(())
    }

    pub async fn write_enable(&mut self) -> Result<(), ErrorKind> {
        let mut spi = self.spi.select().await;
        spi.write(&[0x06]).await?;

        Ok(())
    }

    pub async fn prepare_write(&mut self) -> Result<(), ErrorKind> {
        self.until_ready().await?;
        self.write_enable().await?;

        Ok(())
    }

    pub async fn page_program(&mut self, addr: u32, buf: &[u8]) -> Result<u32, ErrorKind> {
        self.prepare_write().await?;

        let mut spi = self.spi.select().await;
        spi.write(&[0x02]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        spi.write(buf).await?;
        Ok(buf.len() as u32)
    }

    /// Write `buf` to the flash, possibly spanning multiple pages
    pub async fn write(&mut self, addr: u32, buf: &[u8]) -> Result<u32, ErrorKind> {
        let addr = addr as usize;
        let mut i = 0;

        while i < buf.len() {
            let page_end = (addr + 256) / 256 * 256;
            
            let section_size = (page_end - addr).min(buf.len() - i);
            self.page_program((addr + i) as u32, &buf[i..(i + section_size)]).await?;

            i += section_size;
        }

        Ok(buf.len() as u32)
    }

    /// For debugging, check to see if what you wrote is what's on the chip. This will panic if you
    /// accidentally make an invalid write.
    pub async fn checked_write(&mut self, addr: u32, buf: &[u8]) -> Result<u32, ErrorKind> {
        let mut buf2 = [0; 512];
        self.read(addr, &mut buf2).await?;

        if !buf2[0..buf.len()].iter().all(|b| *b == 0xFF) {
            panic!("Bytes at {} are not erased!\nData: {:x?}",
                addr,
                &buf2[0..buf.len()],
            )
        }

        self.write(addr, buf).await?;
        self.read(addr, &mut buf2).await?;

        if *buf != buf2[0..buf.len()] {
            panic!("Bytes at {} are not the same as what was written!\nData: {:x?}\nWritten: {:x?}",
                addr,
                &buf2[0..buf.len()],
                buf
            )
        }

        Ok(addr)
    }

    pub async fn erase_sector(&mut self, addr: u32) -> Result<(), ErrorKind>{
        self.prepare_write().await?;
        let mut spi = self.spi.select().await;
        spi.write(&[0x20]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        Ok(())
    }

    /// Will panic if the sector isn't actually erased.
    pub async fn checked_erase_sector(&mut self, addr: u32) -> Result<(), ErrorKind> {
        if addr % 4096 != 0 {
            panic!("Addr {} is not divisible by 4096!", addr);
        }

        self.erase_sector(addr).await?;

        let mut buf = [0xFF; 256];

        for i in 0..(4096/256) {
            self.read(addr + 256 * i, &mut buf).await?;
            if !buf.iter().all(|b| *b == 0xFF) {
                panic!("Sector {} was not erased!", addr)
            }
        }

        Ok(())
    }

    pub async fn read(&mut self, addr: u32, words: &mut[u8]) -> Result<(), ErrorKind>{
        self.until_ready().await?;

        let mut spi = self.spi.select().await;
        spi.write(&[0x03]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        spi.read(words).await?;
        Ok(())
    }

    pub async fn chip_erase(&mut self) -> Result<(), ErrorKind> {
        self.prepare_write().await?;
        
        let mut spi = self.spi.select().await;
        spi.write(&[0x60]).await?;

        Ok(())
    }

    pub async fn erase_64kb_block(&mut self, addr:u32) -> Result<(), ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0xD8]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        Ok(())
    }

    pub async fn erase_32kb_block(&mut self, addr:u32) -> Result<(), ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0x52]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        Ok(())
    }

    pub async fn suspend(&mut self) -> Result<(), ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0x75]).await?;
        Ok(())
    }

    pub async fn resume(&mut self) -> Result<(), ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0x7A]).await?;
        Ok(())
    }

    pub async fn read_device_id(&mut self) -> Result<u8, ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0x90]).await?;
        spi.write(&[0,0,0]).await?;
        let mut array: [u8; 2] = [0; 2];
        spi.read(&mut array).await?;
        Ok(array[1])
    }
}