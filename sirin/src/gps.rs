use defmt::{error, info, println, warn, Debug2Format};
use embassy_executor::task;
use embassy_stm32::{mode::Async, pac::Interrupt::PVD_AVD, usart::{self, RingBufferedUartRx, Uart, UartTx}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Instant;
use sirin_shared::{packet::{self, GpsFix, GpsFixType, Log, OutPacket, Vec3}, song::FromSong};
use uunit::{Meters, MetersPerSecond, WithUnits};
use crate::{error::SirinError, io::{broadcast, broadcast_log}, usb};
use ublox::{FixedBuffer, cfg_nav5::CfgNav5Builder, cfg_prt::{CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode, UartPortId}, proto31::Proto31};
use ublox::{Parser,UbxPacket,proto31::PacketRef,GnssFixType,Position,Velocity};

pub(crate) type Proto = ublox_device::ublox::proto31::Proto31;
pub static GPS_FIX: Signal<CriticalSectionRawMutex, GpsFix> = Signal::new();

#[task]
pub async fn gps_task(
    gps_rx: &'static mut RingBufferedUartRx<'static>,
    gps_tx: &'static mut UartTx<'static, Async>
) {
    let mut fix = GpsFix::default();
    
    //Send GPS setup packet(s)
    let port_config_packet: &[u8] = &CfgPrtUartBuilder {
        portid: UartPortId::Uart1,
        reserved0: 0,
        tx_ready: 0,
        mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
        baud_rate: 9600,
        in_proto_mask: InProtoMask::all(),
        out_proto_mask: OutProtoMask::UBLOX,
        flags: 0,
        reserved5: 0,
    }.into_packet_bytes();
    //let nav_mode_config_packet: &[u8] = &CfgNav5Builder{}.into_packet_bytes();
    gps_tx.write(port_config_packet).await.unwrap();
    //gps_tx.write(nav_mode_config_packet).await.unwrap();

    //Create packet parser
    let mut packet_parser:Parser<FixedBuffer<512>,Proto31> = ublox::Parser::new_fixed();
    
    //Task loop
    loop {
        match gps_impl(gps_rx, &mut fix, &mut packet_parser).await {
            Err(err) => error!("GPS Error: {}", Debug2Format(&err)),
            Ok(_) => {}
        };
    }   
}

pub async fn read(
    gps_rx: &mut RingBufferedUartRx<'static>,
    buf: &mut [u8]
) -> Result<(), SirinError> {
    let mut len = buf.len();
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
    packet_parser: &mut Parser<FixedBuffer<512>, Proto31>
) -> Result<(), SirinError> {
    loop {
        //GPS_FIX.signal(fix.clone());
        //Read 32 bytes from RingBuffer at a time
        let mut bytes = [0u8; 32];
        read(gps_rx, &mut bytes).await?;

        //Copy those 32 bytes to Parser internal buffer
        let mut iterator = packet_parser.consume_ubx(&bytes);

        loop {
            //Try to read next packet from Parser internal buffer
            match iterator.next() {
                Some(Ok(UbxPacket::Proto31(packet))) => {
                    match packet{
                        PacketRef::MonVer(mon_ver_packet) => {
                            
                        },
                        PacketRef::NavPvt(nav_pvt_packet) => {
                            let has_time = nav_pvt_packet.fix_type() == GnssFixType::Fix3D
                                || nav_pvt_packet.fix_type() == GnssFixType::GPSPlusDeadReckoning
                                || nav_pvt_packet.fix_type() == GnssFixType::TimeOnlyFix;
                            let has_posvel = nav_pvt_packet.fix_type() == GnssFixType::Fix3D
                                || nav_pvt_packet.fix_type() == GnssFixType::GPSPlusDeadReckoning;

                            if has_posvel {
                                let pos: Position = nav_pvt_packet.into();
                                let vel: Velocity = nav_pvt_packet.into();
                                fix.pos = Vec3 { 
                                    x: pos.lon,
                                    y: pos.lat, 
                                    z: pos.alt
                                }
                            }

                            if has_time {
                                
                            }
                            //new gps fix available
                            GPS_FIX.signal(fix.clone());
                        },
                        PacketRef::EsfRaw(raw_packet) => {
                            //println!("Got raw message: {raw:?}");
                            
                        },
                        _ => {
                            //println!("{packet_ref:?}");
                            
                        },
                    }
                },
                Some(Ok(UbxPacket::Proto23(packet))) => {
                    match packet{
                        PacketRef::MonVer(mon_ver_packet) => {
                            
                        },
                        PacketRef::NavPvt(nav_pvt_packet) => {
                            let has_time = nav_pvt_packet.fix_type() == GnssFixType::Fix3D
                                || nav_pvt_packet.fix_type() == GnssFixType::GPSPlusDeadReckoning
                                || nav_pvt_packet.fix_type() == GnssFixType::TimeOnlyFix;
                            let has_posvel = nav_pvt_packet.fix_type() == GnssFixType::Fix3D
                                || nav_pvt_packet.fix_type() == GnssFixType::GPSPlusDeadReckoning;

                            if has_posvel {
                                let pos: Position = nav_pvt_packet.into();
                                fix.pos = Vec3 { 
                                    x: nav_pvt_packet.longitude().with_units(),
                                    y: nav_pvt_packet.latitude().with_units(), 
                                    z: nav_pvt_packet }
                            }

                            if has_time {
                                
                            }
                            //new gps fix available
                            GPS_FIX.signal(fix.clone());
                        },
                        PacketRef::EsfRaw(raw_packet) => {
                            //println!("Got raw message: {raw:?}");
                            
                        },
                        _ => {
                            //println!("{packet_ref:?}");
                            
                        },
                    }
                },
                Some(Err(e)) => {
                    
                },
                None => {
                    
                }
            }
        }
    }
    /*let mut start_byte_0 = [0u8; 1];
        read(gps, &mut start_byte_0).await?;

        if start_byte_0[0] != 0xA0 {
            warn!("Bad start byte 0: {:x}", start_byte_0[0]);
            continue;
        }

        let mut start_byte_1 = [0u8; 1];
        read(gps, &mut start_byte_1).await?;

        if start_byte_1[0] != 0xA1 {
            warn!("Bad start byte 1: {:x}", start_byte_1[0]);
            continue;
        }
        let mut payload_length = [0u8; 2];
        read(gps, &mut payload_length).await?;
        let payload_length = u16::from_be_bytes(payload_length);

        let mut payload = [0u8; 65536];
        read(gps, &mut payload[0..payload_length as usize]).await?;

        let mut checksum = [0u8; 1];
        read(gps, &mut checksum).await?;

        let mut footer = [0u8; 2];
        read(gps, &mut footer).await?;

        if footer[0] != 0x0D || footer[1] != 0x0A {
            warn!("The footer is incorrect");
        }

        if payload.len() == 0 {
            continue;
        }

        let message_id = payload[0];

        match message_id {
            0xDF => {
                info!("Received 0xDF fix packet");

                println!("Payload: {:x}", payload);

                if payload.len() < 48  {
                    warn!("GPS Fix response is too short!");
                    continue;
                }

                let fix_type = match payload[2] {
                    0x00 => GpsFixType::NoFix,
                    0x01 => GpsFixType::FixPrediction,
                    0x02 => GpsFixType::Fix2d,
                    0x03 => GpsFixType::Fix3d,
                    0x04 => GpsFixType::FixDifferential,
                    id => {
                        error!("Invalid fix type ({:x})", id);
                        return Err(SirinError::CorruptedData);
                    }
                };

                info!("fix status: {} ({})", Debug2Format(&fix_type), payload[2]);

                if fix_type == GpsFixType::NoFix {
                    return Ok(());
                }

                let mut pos_x_arr = [0; 8];
                pos_x_arr.copy_from_slice(&payload[13..21]);
                let pos_x = f64::from_be_bytes(pos_x_arr);

                let mut pos_y_arr = [0; 8];
                pos_y_arr.copy_from_slice(&payload[21..29]);
                let pos_y = f64::from_be_bytes(pos_y_arr);

                let mut pos_z_arr = [0; 8];
                pos_z_arr.copy_from_slice(&payload[29..37]);
                let pos_z = f64::from_be_bytes(pos_z_arr);

                let mut vel_x_arr = [0; 4];
                vel_x_arr.copy_from_slice(&payload[37..41]);
                let vel_x = f32::from_be_bytes(vel_x_arr);

                let mut vel_y_arr = [0; 4];
                vel_y_arr.copy_from_slice(&payload[45..49]);
                let vel_y = f32::from_be_bytes(vel_y_arr);

                let mut vel_z_arr = [0; 4];
                vel_z_arr.copy_from_slice(&payload[49..53]);
                let vel_z = f32::from_be_bytes(vel_z_arr);
                
                fix.time = (Instant::now().as_millis() as u32).with_units();
                fix.fix_type = fix_type;
                fix.pos = Vec3 {
                    x: pos_x.with_units(),
                    y: pos_y.with_units(),
                    z: pos_z.with_units(),
                };
                fix.vel = Vec3 {
                    x: vel_x.with_units(),
                    y: vel_y.with_units(),
                    z: vel_z.with_units()
                };
                info!("Fix: {:#?}", Debug2Format(&fix))
            },
            0xDE => {
                let nsvs = payload[2];
                let chsize = 8;

                let mut almanac_count = 0;
                let mut ephemeris_count = 0;
                let mut healthy_count = 0;

                for i in 0..nsvs {
                    let offset = 3 + (chsize as usize * i as usize);
                    let chid = payload[offset];
                    let svid = payload[offset + 1];
                    let status = payload[offset + 2];

                    let almanac = status & 0b001 > 0;
                    let ephemeris = status & 0b010 > 0;
                    let healthy = status & 0b100 > 0;

                    if almanac {
                        almanac_count += 1;
                    }

                    if ephemeris {
                        ephemeris_count += 1;
                    }

                    if healthy {
                        healthy_count += 1;
                    }

                    info!(
                        "GPS (chid: {}, svid: {}): [{}] almanac, [{}] ephemeris, [{}] healthy", 
                        chid, svid, almanac, ephemeris, healthy
                    )
                }
                info!("{} satellites total", nsvs);
                fix.satellites = nsvs;
                fix.almanac = almanac_count;
                fix.ephemerides = ephemeris_count;
                fix.healthy_satellites = healthy_count;
            },
            id => {
                info!("Received message with ID {:x}, len {}", message_id, payload_length)
            } 
        }*/
}