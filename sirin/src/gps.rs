


use core::fmt::Debug;

use crate::{
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
    packet::{self, EcefPos, GpsDop, GpsFix, GpsFixType, Log, Vec3},
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
    let mut buf = [0u8; 128];

    //Task loop
    loop {
        info!("New gps loop iteration");
        match gps_impl(
            gps_rx,
            gps_tx,
            &mut packet_parser,
            &mut buf,
            &mut fix,
        ).await {
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
#[allow(unused)]
pub async fn gps_impl(
    gps_rx: &mut RingBufferedUartRx<'static>,
    gps_tx: &mut UartTx<'static, Async>,
    packet_parser: &mut Parser<FixedBuffer<512>, Proto31>,
    buf: &mut [u8],
    fix: &mut GpsFix,
) -> Result<(), SirinError> {
    //Read bytes from GPS RingBuffer until Bytes is full
    read(gps_rx, buf).await?;

    //Copy bytes into the parser
    let mut iterator = packet_parser.consume_ubx( buf);

    //Attempt to construct packets from whatever bytes the parser has
    while let Some(packet) = iterator.next() {
        match packet {
            Ok(UbxPacket::Proto31(packet)) => {
                match packet {
                    PacketRef::NavPosEcef(p) => {
                        fix.itow = p.itow();

                        // Not actually meters, I think.
                        fix.pos.x.value = p.ecef_x_meters_raw();
                        fix.pos.y.value = p.ecef_y_meters_raw();
                        fix.pos.z.value = p.ecef_z_meters_raw();
                        fix.pos_acc.value = p.p_acc_meters_raw();
                    },
                    _ => {}
                    /*PacketRef::NavPvt(nav_pvt_packet) => {
                        //info!("Got version message: nav_pvt");
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
                            fix.horizontal_accuracy = nav_pvt_packet.horizontal_accuracy().with_units();
                            fix.vertical_accuracy = nav_pvt_packet.vertical_accuracy().with_units();
                            fix.pos_dop = nav_pvt_packet.pdop();
                        }

                        if has_time {
                            fix.time = 1u64.with_units();
                            fix.satellites = nav_pvt_packet.num_satellites();
                        }
                        
                        //Make new gps fix available to main task
                        GPS_FIX.signal(fix.clone());
                        info!("GPS fix sent to main!");
                    }
                    PacketRef::AckAck(_raw_packet) => {
                        info!("Got message: AckAck");
                        //info!("{}", raw_packet.as_bytes());
                    }
                    PacketRef::AckNak(_raw_packet) => {
                        info!("Got message: AckNak");
                        //info!("{}", raw_packet.as_bytes());
                    }
                    PacketRef::NavDop(dop_packet) => {
                        info!("Got message: nav_dop");
                        info!("NavDop: {:?}", Debug2Format(&dop_packet));
                        dop.time_of_week_millis = dop_packet.itow().with_units();
                        dop.geometric_dop = dop_packet.geometric_dop();
                        dop.position_dop = dop_packet.position_dop();
                        dop.time_dop = dop_packet.time_dop();
                        dop.vertical_dop = dop_packet.vertical_dop();
                        dop.horizontal_dop = dop_packet.horizontal_dop();
                        dop.northing_dop = dop_packet.northing_dop();
                        dop.easting_dop = dop_packet.easting_dop();
                        //New GPS DOP available
                        GPS_DOP.signal(dop.clone());
                    }
                    PacketRef::MonRf(mon_rf_packet) => {
                        for block in mon_rf_packet.blocks(){
                            info!("Got message: mon_rf");
                            info!("Block: {:?}", Debug2Format(&block));
                            GPS_RF.signal(rf.clone());
                        }
                    }
                    PacketRef::NavSat(nav_sat_packet) => {
                        for nav_sat_info in nav_sat_packet.svs(){
                            info!("Got message: nav_sat");
                            info!("Nav Sat Info: {:?}", Debug2Format(&nav_sat_info));
                            GPS_SAT.signal(sat.clone());
                        }
                    }
                    _ => {
                        //info!("Other packet not listed");
                    }*/
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