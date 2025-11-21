use core::fmt::Debug;

use crate::{
    error::SirinError,
    io::{broadcast, broadcast_log},
    usb,
};
use defmt::{error, info, println, warn, Debug2Format};
use embassy_executor::{raw, task};
use embassy_stm32::{
    mode::Async,
    pac::Interrupt::PVD_AVD,
    sai::B,
    usart::{self, RingBufferedUartRx, Uart, UartTx},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Instant, Timer};
use serde::de;
use sirin_shared::{
    packet::{self, GpsFix, GpsFixType, Log, OutPacket, Vec3},
    song::FromSong,
};
use ublox::{
    cfg_gnss::CfgGnssBuilder,
    cfg_inf::CfgInfBuilder,
    cfg_nav5::CfgNav5Builder,
    cfg_prt::{
        CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode,
        UartPortId,
    },
    proto31::Proto31,
    FixedBuffer,
};
use ublox::{proto31::*, GnssFixType, Parser, Position, UbxPacket, Velocity};
use uunit::{Meters, MetersPerSecond, WithUnits};

pub static GPS_FIX: Signal<CriticalSectionRawMutex, GpsFix> = Signal::new();

#[task]
pub async fn gps_task(
    gps_rx: &'static mut RingBufferedUartRx<'static>,
    gps_tx: &'static mut UartTx<'static, Async>,
) {
    let mut fix = GpsFix::default();
    //Create packet parser
    let mut packet_parser: Parser<FixedBuffer<512>, Proto31> = ublox::Parser::new_fixed();
    //Create buffer to read into from RingBuffer
    let mut bytes_from_ring_buf = [0u8; 128];

    //Task loop
    loop {
        info!("New gps loop iteration");
        match gps_impl(gps_rx, &mut fix, &mut packet_parser, gps_tx, &mut bytes_from_ring_buf).await {
            Err(err) => error!("GPS Error: {}", Debug2Format(&err)),
            Ok(_) => {}
        };
    }
}

pub async fn read(
    gps_rx: &mut RingBufferedUartRx<'static>,
    buf: &mut [u8],
) -> Result<(), SirinError> {
    let len = buf.len();
    let mut i = 0;
    loop {
        i += gps_rx.read(&mut buf[i..]).await?;

        if i >= len {
            return Ok(());
        }
    }
}

pub async fn gps_impl(
    gps_rx: &mut RingBufferedUartRx<'static>,
    fix: &mut GpsFix,
    packet_parser: &mut Parser<FixedBuffer<512>, Proto31>,
    gps_tx: &mut UartTx<'static, Async>,
    bytes: &mut [u8],
) -> Result<(), SirinError> {
    //Read bytes from GPS RingBuffer until Bytes is full
    read(gps_rx, bytes).await?;

    //Copy bytes into the parser
    let mut iterator = packet_parser.consume_ubx( bytes);

    //Attempt to construct packets from whatever bytes the parser has
    while let Some(packet) = iterator.next() {
        match packet {
            Ok(UbxPacket::Proto31(packet)) => {
                match packet {
                    PacketRef::NavPvt(nav_pvt_packet) => {
                        //info!("Got version message: nav_pvt_packet");
                        let has_time: bool;
                        let has_posvel: bool;

                        match nav_pvt_packet.fix_type() {
                            GnssFixType::TimeOnlyFix => {
                                fix.fix_type = GpsFixType::TimeOnlyFix;
                                has_time = true;
                                has_posvel = false;
                            }
                            GnssFixType::GPSPlusDeadReckoning => {
                                fix.fix_type = GpsFixType::FixDifferential;
                                has_time = true;
                                has_posvel = true;
                            }
                            GnssFixType::NoFix => {
                                fix.fix_type = GpsFixType::NoFix;
                                has_time = false;
                                has_posvel = false;
                            }
                            GnssFixType::DeadReckoningOnly => {
                                fix.fix_type = GpsFixType::FixPrediction;
                                has_time = true;
                                has_posvel = false;
                            }
                            GnssFixType::Fix2D => {
                                fix.fix_type = GpsFixType::Fix2d;
                                has_time = true;
                                has_posvel = true;
                            }
                            GnssFixType::Fix3D => {
                                fix.fix_type = GpsFixType::Fix3d;
                                has_time = true;
                                has_posvel = true;
                            }
                            _ => {
                                fix.fix_type = GpsFixType::NoFix;
                                has_time = false;
                                has_posvel = false;
                            }
                        }
                        //Update GPS fix
                        if has_posvel {
                            fix.pos = Vec3 {
                                x: nav_pvt_packet.longitude().with_units(),
                                y: nav_pvt_packet.latitude().with_units(),
                                z: nav_pvt_packet.height_msl().with_units(),
                            };
                            fix.vel = Vec3 {
                                x: nav_pvt_packet.vel_east().with_units(),
                                y: nav_pvt_packet.vel_north().with_units(), 
                                z: nav_pvt_packet.vel_down().with_units()
                            };
                        }

                        if has_time {
                            //TODO: figure out what to do here
                            fix.satellites = nav_pvt_packet.num_satellites();
                        }
                        
                        //Make new gps fix available to main task
                        GPS_FIX.signal(fix.clone());
                    }
                    PacketRef::AckAck(_raw_packet) => {
                        info!("Got message: AckAck");
                        //info!("{}", raw_packet.as_bytes());
                    }
                    PacketRef::AckNak(_raw_packet) => {
                        info!("Got message: AckNak");
                        //info!("{}", raw_packet.as_bytes());
                    }
                    PacketRef::MonRf(mon_rf_packet) => {
                        for block in mon_rf_packet.blocks(){
                            info!("Block: {:?}", Debug2Format(&block));
                        }
                    }
                    PacketRef::NavSat(nav_sat_packet) => {
                        for nav_sat_info in nav_sat_packet.svs(){
                            info!("Nav Sat Info: {:?}", Debug2Format(&nav_sat_info));   
                        }
                    }
                    _ => {
                        //info!("Other packet not listed");
                    }
                }
            }
            //Could not parse packet
            Err(e) => {
                return Err(SirinError::GpsPacketParseError(e))
            }
        }
    }

    Ok(())
}