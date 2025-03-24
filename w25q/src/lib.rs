#![no_std]

use core::{future::{poll_fn, Future}, task::Poll};

use embedded_hal_async::{digital::Wait, spi::ErrorKind};
use spi_handle::SpiHandle;
use embedded_hal_async::spi::SpiBus;
use yield_now::yield_now;

mod yield_now;

pub struct W25Q<S: SpiHandle>{
    spi: S
}

impl <S: SpiHandle> W25Q<S>{
    pub fn new(spi: S) -> Self {
        Self {
            spi
        }
    }

    pub async fn read_status_reg_1(&mut self) -> Result<u8, ErrorKind> {
        let mut spi = self.spi.select().await;
        let mut out = [0u8];
        spi.write(&[0x05]).await?;
        spi.read(&mut out).await?;
        Ok(out[0])
    }

    pub async fn is_busy(&mut self) -> Result<bool, ErrorKind> {
        Ok(self.read_status_reg_1().await? % 2 == 1)
    }

    /// Returns a future that completes when the chip is ready (not BUSY)
    pub async fn until_ready(&mut self) -> Result<(), ErrorKind> {
        while self.is_busy().await? {
            yield_now::yield_now().await;
        }

        Ok(())
    }

    //write enable
    pub async fn write_enable(&mut self) -> Result<(), ErrorKind> {
        let mut spi = self.spi.select().await;
        spi.write(&[0x06]).await?;

        Ok(())
    }

    // TODO: mutex/lockout to handle concurrent pages?
    pub async fn prepare_write(&mut self) -> Result<(), ErrorKind> {
        self.until_ready().await?;
        self.write_enable().await?;

        Ok(())
    }

    //page program 02h
    pub async fn page(&mut self, addr: u32, words: &[u8]) -> Result<(), ErrorKind> {
        self.prepare_write().await?;

        let mut spi = self.spi.select().await;
        spi.write(&[0x02]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        spi.write(words).await?;
        Ok(())
    }

    /// sector erase 20h
    pub async fn sector_erase(&mut self, addr: u32) -> Result<(), ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0x20]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        Ok(())
    }

    //read data 03h
    pub async fn read_data(&mut self, addr: u32, words: &mut[u8]) -> Result<(), ErrorKind>{
        self.until_ready().await?;

        let mut spi = self.spi.select().await;
        spi.write(&[0x03]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        spi.read(words).await?;
        Ok(())
    }

    //all non-fast read, writes, erases should be implemented
    //chip erase
    pub async fn chip_erase(&mut self) -> Result<(), ErrorKind>{
        self.prepare_write().await?;
        
        let mut spi = self.spi.select().await;
        spi.write(&[0x60]).await?;
        // TODO: await erase
        Ok(())
    }

    //block erase 64KB D8h
    pub async fn block_64kb_erase(&mut self, addr:u32) -> Result<(), ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0xD8]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        Ok(())
    }

    //block erase 32KB 52h
    pub async fn block_32kb_erase(&mut self, addr:u32) -> Result<(), ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0x52]).await?;
        spi.write(&[((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, ((addr & 0xFF) as u8)]).await?;
        Ok(())
    }

    //erase/program suspend 75h
    pub async fn suspend(&mut self) -> Result<(), ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0x75]).await?;
        Ok(())
    }

    //erase/program resume 7Ah
    pub async fn resume(&mut self) -> Result<(), ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0x7A]).await?;
        Ok(())
    }

    //read device_id 90h
    pub async fn read_device_id(&mut self) -> Result<u8, ErrorKind>{
        let mut spi = self.spi.select().await;
        spi.write(&[0x90]).await?;
        spi.write(&[0,0,0]).await?;
        let mut array: [u8; 2] = [0; 2];
        spi.read(&mut array).await?;
        Ok(array[1])
    }

    /*
    TODO: Fix
    
    //read manufacturer id --returns a constant EFh
    pub fn read_manufacturer_id(&mut self) -> u16{
        0xEF
    }*/
}   

pub struct Logger {
    current_addr: u32
}

impl Logger {
    pub fn new() -> Self {
        Logger {
            current_addr: 0
        }
    }
    pub async fn log_partial_page(&mut self, flash: &mut W25Q<impl SpiHandle>, msg: &[u8]) -> Result<(), ErrorKind> {
        flash.page(self.current_addr, msg).await?;
        self.current_addr += msg.len() as u32;
        Ok(())
    }
    pub async fn log(&mut self, flash: &mut W25Q<impl SpiHandle>, msg: &[u8]) -> Result<(), ErrorKind> {
        while flash.is_busy().await? {
            yield_now().await;
        }
        for i in 0..((msg.len()/4096) + 1){
            flash.sector_erase(self.current_addr + (i as u32)*(4096)).await.unwrap();
        }
        let start = self.current_addr as usize;
        let end = self.current_addr as usize + msg.len();
        let mut msg_i = 0;
        let first_full_page = ((self.current_addr as usize / 256) + 1) * 256;
        self.log_partial_page(flash, &msg[0..(first_full_page - start)]).await?;
        msg_i = first_full_page - start;
        while ((self.current_addr as usize / 256) + 1) * 256 <= end {
            self.log_partial_page(flash, &msg[msg_i..(msg_i + 256)]).await?;
            msg_i += 256;
        }
        self.log_partial_page(flash, &msg[msg_i..]).await?;
        Ok(())
    }

    pub async fn erase(&mut self, flash: &mut W25Q<impl SpiHandle>) -> Result<(), ErrorKind> {
        let next_sector = ((self.current_addr as usize / 4096) + 1) * 4096;
        flash.sector_erase(next_sector as u32).await?;
        Ok(())
    }

    pub async fn reset_address(&mut self) -> Result<(), ErrorKind> {
        self.current_addr = 0;
        Ok(())
    }
}