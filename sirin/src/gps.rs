use core::fmt::Debug;
use crate::{error::SirinError,io::{broadcast, broadcast_log},usb,};
use defmt::{error, info, println, warn, Debug2Format};
use embassy_executor::{raw, task};
use embassy_stm32::{mode::Async,pac::Interrupt::PVD_AVD,sai::B,usart::{self, RingBufferedUartRx, Uart, UartTx},};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Instant, Timer};
use serde::de;
use sirin_shared::{packet::{self, EcefPos, GpsDop, GpsFix, GpsFixType, Log, OutPacket, Vec3},song::FromSong,};
use uunit::{Meters, MetersPerSecond, WithUnits};
use ublox::{
    nav_pvt::proto27_31::{NavPvt, NavPvtRef}, proto31::*, rxm_rawx::RxmRawx,
    cfg_inf::{CfgInf, CfgInfMask, CfgInfBuilder}, cfg_msg::CfgMsgSinglePortBuilder, cfg_rate::{CfgRate, CfgRateBuilder},
    cfg_gnss::CfgGnssBuilder, cfg_nav5::CfgNav5Builder, mon_rf::MonRf, nav_dop::NavDop, nav_sat::NavSat, nav_pos_ecef::NavPosEcef,
    cfg_prt::{CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode,UartPortId,},
    UbxPacketMeta, UbxProtocol, proto31::Proto31, FixedBuffer, GnssFixType, Parser, Position, UbxPacket, Velocity
};

pub static GPS_FIX: Signal<CriticalSectionRawMutex, GpsFix> = Signal::new();

pub async fn gps_init(gps_uart: &mut Uart<'_, Async>) {
    //Construct GPS config packets
    //UART config for GPS
    let port_config_packet = CfgPrtUartBuilder {
        portid: UartPortId::Uart1,
        reserved0: 0,
        tx_ready: 0,
        mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
        baud_rate: 9600,
        in_proto_mask: InProtoMask::UBLOX,
        out_proto_mask: OutProtoMask::UBLOX,
        flags: 0,

        reserved5: 0,
    };
    //Navigation Mode config
    let mut nav_mode_config = CfgNav5Builder::default();
    nav_mode_config.dyn_model = ublox::cfg_nav5::NavDynamicModel::AirborneWithLess4gAcceleration;
    nav_mode_config.fix_mode = ublox::cfg_nav5::NavFixMode::Auto2D3D;
    //GPS measurement and calculation rate
    let gps_update_config = CfgRateBuilder {
        measure_rate_ms: 100,
        nav_rate: 1,
        time_ref: ublox::cfg_rate::AlignmentToReferenceTime::Utc,
    };
    //Navigation message config (Position/Velocity/Time)
    let nav_msg_config = CfgMsgSinglePortBuilder {
        msg_class: NavPvt::CLASS,
        msg_id: NavPvt::ID,
        rate: 1,
    };
    //DOP message config (Dilution of precession)
    let nav_dop_config = CfgMsgSinglePortBuilder {
        msg_class: NavDop::CLASS,
        msg_id: NavDop::ID,
        rate: 1,
    };
    //RF message config ()
    let rf_msg_config = CfgMsgSinglePortBuilder {
        msg_class: MonRf::CLASS,
        msg_id: MonRf::ID,
        rate: 5,
    };
    //Satelite message config ()
    let satelite_msg_config = CfgMsgSinglePortBuilder {
        msg_class: NavSat::CLASS,
        msg_id: NavSat::ID,
        rate: 5,
    };
    let nav_ecef_config = CfgMsgSinglePortBuilder {
        msg_class: NavPosEcef::CLASS,
        msg_id: NavPosEcef::ID,
        rate: 1,
    };

    //Send GPS config packets
    let gps_config_delay = 50u64;
    gps_uart.write(&port_config_packet.into_packet_bytes()).await.unwrap();
    Timer::after_millis(gps_config_delay).await;
    gps_uart.write(&nav_mode_config.into_packet_bytes()).await.unwrap();
    Timer::after_millis(gps_config_delay).await;
    gps_uart.write(&gps_update_config.into_packet_bytes()).await.unwrap();
    Timer::after_millis(gps_config_delay).await;
    gps_uart.write(&nav_msg_config.into_packet_bytes()).await.unwrap();
    Timer::after_millis(gps_config_delay).await;
    gps_uart.write(&nav_dop_config.into_packet_bytes()).await.unwrap();
    Timer::after_millis(gps_config_delay).await;
    gps_uart.write(&rf_msg_config.into_packet_bytes()).await.unwrap();
    Timer::after_millis(gps_config_delay).await;
    gps_uart.write(&satelite_msg_config.into_packet_bytes()).await.unwrap();
    Timer::after_millis(gps_config_delay).await;
    gps_uart.write(&nav_ecef_config.into_packet_bytes()).await.unwrap();
    Timer::after_millis(gps_config_delay).await;
}

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
        //info!("New gps loop iteration");
        match gps_impl(gps_rx, gps_tx, &mut packet_parser, &mut buf, &mut fix).await {
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
    let mut iterator = packet_parser.consume_ubx(buf);

    //Attempt to construct packets from whatever bytes the parser has
    //println!("Attempting to run gps loop...");
    while let Some(packet) = iterator.next() {
        //println!("Packet received from gps!!! not broken!!!");
        match packet {
            Ok(UbxPacket::Proto31(packet)) => {
                //println!("Packet found!");
                //println!("{:?}", Debug2Format(&packet));
                match packet {
                    PacketRef::NavPosEcef(p) => {
                        //println!("Fix found!");
                        fix.itow = p.itow();
                        // Not actually meters, I think.
                        fix.pos.x.value = p.ecef_x_meters_raw();
                        fix.pos.y.value = p.ecef_y_meters_raw();
                        fix.pos.z.value = p.ecef_z_meters_raw();
                        fix.pos_acc.value = p.p_acc_meters_raw();
                    }
                    PacketRef::NavPvt(p) => {
                        fix.fix_type = match p.fix_type() {
                            GnssFixType::TimeOnlyFix => GpsFixType::TimeOnlyFix,
                            GnssFixType::GPSPlusDeadReckoning => {
                                GpsFixType::FixDifferential
                            }
                            GnssFixType::NoFix => GpsFixType::NoFix,
                            GnssFixType::DeadReckoningOnly => {
                                GpsFixType::FixPrediction
                            }
                            GnssFixType::Fix2D => GpsFixType::Fix2d,
                            GnssFixType::Fix3D => GpsFixType::Fix3d,
                            _ => GpsFixType::NoFix,
                        };
                        fix.itow = p.itow();
                        fix.lon = p.longitude();
                        fix.lat = p.latitude();
                        fix.satellites = p.num_satellites();
                        fix.vel = Vec3 {
                            x: ((p.vel_north() * 100.0) as i32).with_units(),
                            y: ((p.vel_east() * 100.0) as i32).with_units(),
                            z: ((p.vel_down() * 100.0) as i32).with_units(),
                        };
                        fix.vel_acc =
                            ((p.speed_accuracy() * 100.0) as u32).with_units();
                        // Signal on PVT only. NAV-POSECEF updates the same
                        // accumulator, but previously emitted a second,
                        // partially populated fix with no quality metadata.
                        GPS_FIX.signal(fix.clone());
                    }
                    _ => {} /*PacketRef::NavPvt(nav_pvt_packet) => {
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
                                        x: nav_pvt_packet.vel_north().with_units(),
                                        y: nav_pvt_packet.vel_east().with_units(),
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
            Err(e) => return Err(SirinError::GpsPacketParseError(e)),
        }
    }

    Ok(())
}
