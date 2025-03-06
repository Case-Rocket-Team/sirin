//#![feature(error_in_core)]
//#![feature(associated_type_defaults)]
#![no_std]
use core::{mem::MaybeUninit, panic, ptr::addr_of_mut};
use bmp3::{Bmp3};
use embassy_executor::{Executor, Spawner};
use embassy_stm32::{ gpio::{Level, Output, Speed}, spi as em_spi, time::mhz, Config, Peripherals };
use gpio::GpioPins;
use rfm9x::{ReadRfm9x, Rfm9x};
use w25q::W25Q;
use lsm6dso::Lsm6dso;
use h3lis::H3lis;
use spi::{Spi, SpiConfig, SpiConfigStruct, SpiDev, SpiInstance, WithSpiHandle};
use defmt::debug;
use bmp3::Bmp3Readout;
pub mod spi;
pub mod delay;
pub mod gpio;
pub mod sync;
pub mod triplet;

pub struct Sirin {
    pub spawner: Spawner,
    pub spi1: SpiInstance,
    pub spi2: SpiInstance,
    pub gpio: GpioPins,
    pub baro: Bmp3<SpiDev>,
    pub flash: W25Q<SpiDev>,
    pub imu: Lsm6dso<SpiDev>,
    pub highg_imu: H3lis<SpiDev>,
    pub radio: Rfm9x<SpiDev>,
    //pub gps: S1315F8,
}

impl Sirin {
    /// Initializing Sirin is a PITA bc it is a self-referential struct
    #[inline]
    pub async fn init(
        sirin: &'static mut MaybeUninit<Sirin>,
        spawner: Spawner
    ) -> &'static mut Sirin {
        unsafe {
            macro_rules! ptr {
                (sirin . $field: ident) => {
                    addr_of_mut!((*sirin.as_mut_ptr()).$field)
                };
            }

            *ptr!(sirin.spawner) = spawner;
            let mut config = Config::default();
            {
                use embassy_stm32::rcc::*;
                config.rcc.hsi = Some(HSIPrescaler::DIV1);
                config.rcc.csi = true;
                config.rcc.pll1 = Some(Pll {
                    source: PllSource::HSI,
                    prediv: PllPreDiv::DIV4,
                    mul: PllMul::MUL50,
                    divp: Some(PllDiv::DIV2),
                    divq: Some(PllDiv::DIV8), // used by SPI3. 100Mhz.
                    divr: None,
                });
                config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
                config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
                config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
                config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
                config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
                config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
                config.rcc.voltage_scale = VoltageScale::Scale1;
            }

            let p = embassy_stm32::init(config);
            let mut spi_config = em_spi::Config::default();
            spi_config.frequency = mhz(1);

            let spi1: *mut SpiInstance = ptr!(sirin.spi1);
            spi1.write(spi::SpiInstance::new(SpiConfigStruct {
                spi: p.SPI1,
                sck: p.PA5,
                miso: p.PA6,
                mosi: p.PA7,
                dma_tx: p.DMA1_CH3,
                dma_rx: p.DMA1_CH2,
                config: spi_config
            }));

            let spi2: *mut SpiInstance = ptr!(sirin.spi2);
            spi2.write(spi::SpiInstance::new(SpiConfigStruct {
                spi: p.SPI2,
                sck: p.PB13,
                miso: p.PB14,
                mosi: p.PB15,
                dma_tx: p.DMA1_CH5,
                dma_rx: p.DMA1_CH4,
                config: spi_config
            }));

            let gpio: *mut GpioPins = ptr!(sirin.gpio);
            gpio.write(GpioPins {
                p1: p.PA4,
                p2: p.PC4,
                p3: p.PC5,
                p4: p.PB0,
                p5: p.PB1,
                p6: p.PB2,
                p7: p.PE7,
                p8: p.PE8,
                p9: p.PE9,
                p10: p.PE10,
                p11: p.PD7,
                p12: p.PD6,
                p13: p.PD5,
                p14: p.PD4,
                p15: p.PD3,
                p16: p.PD1,
                p17: p.PD0,
                p18: p.PC12,
                p19: p.PC11,
                p20: p.PC10,
            });

            let baro_ptr: *mut Bmp3<SpiDev> = ptr!(sirin.baro);
            let baro_cs = Output::new(p.PA2, Level::High, Speed::High);
            let baro_future = Bmp3::new((*spi1).handle(baro_cs));

            let radio_ptr: *mut Rfm9x<SpiDev> = ptr!(sirin.radio);
            let radio_cs = Output::new(p.PC8, Level::High, Speed::High);
            radio_ptr.write(Rfm9x::new((*spi2).handle(radio_cs)));

            let flash_ptr: *mut W25Q<SpiDev> = ptr!(sirin.flash);
            let flash_cs = Output::new(p.PD2, Level::High, Speed::High);
            flash_ptr.write(W25Q::new((*spi2).handle(flash_cs)));

            let imu_ptr: *mut Lsm6dso<SpiDev> = ptr!(sirin.imu);
            let imu_cs = Output::new(p.PE11, Level::High, Speed::High);
            imu_ptr.write(Lsm6dso::new((*spi1).handle(imu_cs)));

            let highg_imu_ptr: *mut H3lis<SpiDev> = ptr!(sirin.highg_imu);
            let highg_imu_cs = Output::new(p.PE13,Level::High, Speed::High);
            highg_imu_ptr.write(H3lis::new((*spi1).handle(highg_imu_cs)));

            
            // TODO: JOIN FUTURES, AWAIT
            baro_ptr.write(baro_future.await.unwrap());
            (*radio_ptr).init().await.unwrap();
            (*radio_ptr).use_high_power().await.unwrap();
            (*flash_ptr).chip_erase().await.unwrap();
            (*imu_ptr).setup().await.unwrap();
            (*highg_imu_ptr).setup().await.unwrap();

            let sirin: &'static mut _ = sirin.assume_init_mut();
            sirin
        }
    }

    /*

    When converting this whole thing to structs people cut parts of this instead of copying and idk if we still need it so heres a massive comment

    pub async fn self_check(&mut self) -> Selfcheck {
        /*Bounds assume Sirin is not violently accelerating or rotating, 
        and is at or close to 1k foot altitude and within 50F to 95F.
        Adjust bounds if elsewhere*/

        //Check all the h3lis stuff here
        
        

        //Check all the lsm6 (imu) stuff here
        debug!("\nIMU:");
        let imu_id = self.imu.read_manufacturer_id().await.unwrap();
        debug!("Manufacturer ID: {}", imu_id);
        let imu_active = match imu_id{
            108 => true,
            _ => false
        };
        let accel = self.imu.accel().await.unwrap();
        let imu_accel_check: bool = match accel{
            (-16_000_000..=16_000_000, -16_000_000..=16_000_000, -16_000_000..=16_000_000) => true,
            _ => false
        };
        debug!("Instantaneous Acceleration: {} (μg)", accel);
        let gyro = self.imu.gyro().await.unwrap();
        let imu_gyro_check: bool = match gyro{
            (-360_000_000..=360_000_000, -360_000_000..=360_000_000, -360_000_000..=360_000_000) => true,
            _ => false
        };
        debug!("Instantaneous Gyroscope: {} (μdps)", gyro);

        //Check all the baro stuff here
        debug!("\nBaro:");
        

        //Check all the radio stuff here
        debug!("\nRadio:");
        let radio_num = self.radio.version().await.unwrap();
        let radio_active: bool = match radio_num {
            18 => true,
            _ => false
        };
        debug!("Radio Version: {}", radio_num);
        
        //Check all the flash stuff here
        debug!("\nFlash:");
        let flash_active = match self.flash.read_device_id().await.unwrap(){
            21 => true,
            _ => false
        };
        debug!("Manufacturer ID: {}", self.flash.read_device_id().await.unwrap());
        let mut array: [u8; 4] = [0, 0, 0, 0];
        let mut input_array: [u8; 4] = [18, 22, 99, 1];
        debug!("Testing Flash Write: Array '[18, 22, 99, 1]' should print below");
        self.flash.page(100, &mut input_array).await.unwrap();
        self.flash.read_data(100, &mut array).await.unwrap();
        debug!("{}", array);
        let flash_write: bool = match array {
            [18, 22, 99, 1] => true,
            _ => false
        };
    
        debug!("\nOverall Operation Status: 
        H3LIS Active? {}
        H3LIS Acceleration: {}
        LSM6 Active? {}
        LSM6 Acceleration: {} 
        LSM6 Gyroscope: {} 
        BMP3 Pressure: {} 
        BMP3 Temperature: {}
        RFM9X Active? {} 
        W25Q Active? {}
        W25Q Read/Write? {}",
        h3lis_active, 
        h3lis_accel_check, 
        imu_active,
        imu_accel_check, 
        imu_gyro_check, 
        baro_pressure_check, 
        baro_temperature_check,
        radio_active,
        flash_active,
        flash_write);

        [h3lis_active, 
        h3lis_accel_check, 
        imu_active,
        imu_accel_check, 
        imu_gyro_check, 
        baro_pressure_check, 
        baro_temperature_check,
        radio_active,
        flash_active, 
        flash_write]
    } */
}

/*h3lis_active, 
        h3lis_accel_check, 
        imu_active,
        imu_accel_check, 
        imu_gyro_check, 
        baro_pressure_check, 
        baro_temperature_check,
        radio_active,
        flash_active, 
        flash_write*/

pub struct Selfcheck {
    pub baro: BaroSelfcheck,
    pub radio: RadioSelfcheck,
    pub flash: FlashSelfcheck,
    pub imu: ImuSelfcheck,
    pub highg_imu: HighgImuSelfcheck,
}

impl Selfcheck {
    pub fn result(&self) -> Result<(),()> {
        if self.baro.pressure_check.is_ok()
            && self.baro.temperature_check.is_ok()
            && self.radio.radio_active.is_ok()
            && self.flash.active_check.is_ok()
            && self.flash.read_write_check.is_ok()
            && self.imu.accel_check.is_ok()
            && self.imu.gyro_check.is_ok()
            && self.highg_imu.accel_check.is_ok()
            && self.highg_imu.active_check.is_ok()
        {
            Ok(())
        } else {
            Err(())
        }
    }
    pub async fn selfcheck(sirin: &mut Sirin) -> Self {
        Self {
            baro: BaroSelfcheck::selfcheck(sirin).await,
            flash: FlashSelfcheck::selfcheck(sirin).await,
            imu: ImuSelfcheck::selfcheck(sirin).await,
            highg_imu: HighgImuSelfcheck::selfcheck(sirin).await,
            radio: RadioSelfcheck::selfcheck(sirin).await
        }
    }
}

pub struct BaroSelfcheck {
    pub pressure_check: Result<(),()>,
    pub temperature_check: Result<(),()>,
}

impl BaroSelfcheck {
    pub async fn selfcheck(sirin: &mut Sirin) -> Self {
        let baro_data = sirin.baro.read().await.unwrap();
        let pressure_check = match baro_data.pressure.value {
            90_000.0..=110_000.0 => Ok(()),
            _ => Err(())
        };
        debug!("Pressure: {} (Pa)", baro_data.pressure.value);
        
        let temperature_check = match baro_data.temperature.value {
            10.0..=35.0 => Ok(()),
            _ => Err(())
        };
        debug!("Temperature: {} (C)", baro_data.temperature.value);

        Self {
            pressure_check,
            temperature_check
        }
    }
}

pub struct FlashSelfcheck {
    pub active_check: Result<(),()>,
    pub read_write_check: Result<(),()>,
}
    
impl FlashSelfcheck {
    pub async fn selfcheck(sirin: &mut Sirin) -> Self {
        let active_check = match sirin.flash.read_device_id().await.unwrap(){
            21 => Ok(()),
            _ => Err(())
        };

        debug!("Manufacturer ID: {}", sirin.flash.read_device_id().await.unwrap());
        let mut array: [u8; 4] = [0, 0, 0, 0];
        let mut input_array: [u8; 4] = [18, 22, 99, 1];
        debug!("Testing Flash Write: Array '[18, 22, 99, 1]' should print below");
        sirin.flash.page(100, &mut input_array).await.unwrap();
        sirin.flash.read_data(100, &mut array).await.unwrap();
        debug!("{}", array);

        let read_write_check = match array {
            [18, 22, 99, 1] => Ok(()),
            _ => Err(())
        };

        Self {
            active_check,
            read_write_check
        }
    }
}

pub struct ImuSelfcheck {
    pub active_check: Result<(), ()>,
    pub accel_check: Result<(), ()>,
    pub gyro_check: Result<(), ()>
}

impl ImuSelfcheck {
    pub async fn selfcheck(sirin: &mut Sirin) -> Self {
        let imu_id = sirin.imu.read_manufacturer_id().await.unwrap();
        debug!("Manufacturer ID: {}", imu_id);
        let active_check = match imu_id{
            108 => Ok(()),
            _ => Err(())
        };

        let accel = sirin.imu.accel().await.unwrap();
        debug!("Instantaneous Acceleration: {} (μg)", accel);
        let accel_check = match accel{
            (-16_000_000..=16_000_000, -16_000_000..=16_000_000, -16_000_000..=16_000_000) => Ok(()),
            _ => Err(())
        };
        
        let gyro = sirin.imu.gyro().await.unwrap();
        debug!("Instantaneous Gyroscope: {} (μdps)", gyro);
        let gyro_check = match gyro {
            (-360_000_000..=360_000_000, -360_000_000..=360_000_000, -360_000_000..=360_000_000) => Ok(()),
            _ => Err(())
        };

        Self{
            active_check,
            accel_check,
            gyro_check
        }
    }
}

pub struct HighgImuSelfcheck {
    pub active_check: Result<(),()>,
    pub accel_check: Result<(),()>,
}

impl HighgImuSelfcheck {
    pub async fn selfcheck(sirin: &mut Sirin) -> Self {
        debug!("H3LIS:");
        let h3lis_id = sirin.highg_imu.manufacturer_id().await.unwrap();
        debug!("Manufacturer ID: {} ",h3lis_id);

        let active_check = match h3lis_id {
            50 => Ok(()),
            _ => Err(())
        };
        let accel = sirin.highg_imu.acceleration().await.unwrap();
        debug!("Instantaneous Acceleration: {} (μg)", accel);

        let accel_check = match accel {
            (-16_000_000..=16_000_000, -16_000_000..=16_000_000, -16_000_000..=16_000_000) => Ok(()),
            _ => Err(())
        };

        Self {
            accel_check,
            active_check
        }
    }
}

struct RadioSelfcheck {
    pub radio_active: Result<(), ()>,
}

impl RadioSelfcheck {
    pub async fn selfcheck(sirin: &mut Sirin) -> Self {
        let radio_num = sirin.radio.version().await.unwrap();
        let radio_active = match radio_num {
            18 => Ok(()),
            _ => Err(())
        };
        Self {
            radio_active
        }
    }
}