use core::{cell::Cell, fmt::Debug, ptr::addr_of_mut, sync::atomic::Ordering};
use crate::{error::SirinError, flash::{Flash, GpsNavDbPos, GPS_NAV_DB_MAX_LEN}, io::{broadcast, broadcast_log, FLASH_LOGGING_ENABLED}, sync::Mutex, usb,};
use embassy_futures::join::join;
use defmt::{error, info, println, warn, Debug2Format};
use embassy_executor::{raw, task};
use embassy_stm32::{mode::Async,pac::Interrupt::PVD_AVD,sai::B,usart::{self, RingBufferedUartRx, Uart, UartTx},};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use serde::de;
use sirin_shared::{packet::{self, EcefPos, GpsDop, GpsFix, GpsFixType, Log, OutPacket, Vec3},song::FromSong,};
use uunit::{Meters, MetersPerSecond, WithUnits};
use ublox::{
    nav_pvt::{proto27_31::{NavPvt, NavPvtRef}, common::NavPvtFlags}, proto31::*, rxm_rawx::RxmRawx,
    cfg_inf::{CfgInf, CfgInfMask, CfgInfBuilder}, cfg_msg::CfgMsgSinglePortBuilder, cfg_rate::{CfgRate, CfgRateBuilder},
    cfg_gnss::CfgGnssBuilder, cfg_nav5::CfgNav5Builder, mon_rf::MonRf, nav_dop::NavDop, nav_sat::NavSat, nav_pos_ecef::NavPosEcef,
    cfg_prt::{CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode,UartPortId,},
    UbxPacketMeta, UbxProtocol, proto31::Proto31, FixedBuffer, GnssFixType, Parser, Position, UbxPacket, Velocity
};

pub static GPS_FIX: Signal<CriticalSectionRawMutex, GpsFix> = Signal::new();

/// Factory baud rate the module returns to on every power-up (no backup power)
const GPS_BAUD_DEFAULT: u32 = 9600;
/// Baud rate we run the link at; 9600 can't carry the configured output
const GPS_BAUD_FAST: u32 = 115_200;

pub async fn gps_init(gps_uart: &mut Uart<'_, Async>) {
    //Construct GPS config packets
    //UART config for GPS
    let port_config_packet = CfgPrtUartBuilder {
        portid: UartPortId::Uart1,
        reserved0: 0,
        tx_ready: 0,
        mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
        baud_rate: GPS_BAUD_FAST,
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
    // Measure rate at 10Hz so the GPS searches for satellites more thoroughly per update
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
    // Send the baud change at both speeds so it works whether the module is at
    // its 9600 default (after power-up) or still at 115200 (after a reflash
    // without power loss). The copy sent at the wrong speed is ignored.
    let port_config_bytes = port_config_packet.into_packet_bytes();
    for baud in [GPS_BAUD_FAST, GPS_BAUD_DEFAULT] {
        gps_uart.set_baudrate(baud).unwrap();
        gps_uart.write(&port_config_bytes).await.unwrap();
        gps_uart.flush().await.unwrap(); // last byte fully shifted out before changing speed
        Timer::after_millis(100).await;  // module applies the new port settings
    }
    gps_uart.set_baudrate(GPS_BAUD_FAST).unwrap();
    Timer::after_millis(100).await;
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
    gps_run(gps_rx, gps_tx, None).await
}

/// Same as `gps_task`, but also saves the receiver's navigation database to flash
/// and restores it on boot, since the module has no backup power.
#[task]
pub async fn gps_task_with_nav_db(
    gps_rx: &'static mut RingBufferedUartRx<'static>,
    gps_tx: &'static mut UartTx<'static, Async>,
    flash: &'static Mutex<&'static mut Flash>,
) {
    gps_run(gps_rx, gps_tx, Some(flash)).await
}

async fn gps_run(
    gps_rx: &'static mut RingBufferedUartRx<'static>,
    gps_tx: &'static mut UartTx<'static, Async>,
    flash: Option<&'static Mutex<&'static mut Flash>>,
) {
    let mut fix = GpsFix::default();
    // iTOW of the last NavPvt, used to send one GPS_FIX per epoch
    let mut pvt_itow = u32::MAX;
    let mut tracker = NavDbTracker::default();

    //Create packet parser
    let mut packet_parser: Parser<FixedBuffer<512>, Proto31> = ublox::Parser::new_fixed();
    //Create buffer to read into from RingBuffer
    let mut buf = [0u8; 128];

    // Only this task touches NAV_DB_BUF
    #[allow(static_mut_refs)]
    let mut capture = DbdCapture::new(unsafe { &mut *addr_of_mut!(NAV_DB_BUF) });
    let mut saver = NavDbSaver::default();

    // Push the saved database back into the receiver, reading GPS output meanwhile
    if let Some(flash) = flash {
        let loaded = flash.lock().await.load_gps_nav_db(capture.buf).await;
        match loaded {
            Ok(Some(header)) => {
                info!("Restoring GPS nav DB: {} frames, {} bytes", header.frames, header.len);
                let done = Cell::new(false);
                let data = &capture.buf[..header.len as usize];
                let writer = async {
                    send_nav_db(gps_tx, data, header.pos).await;
                    done.set(true);
                };
                let reader = async {
                    while !done.get() {
                        if let Err(err) = gps_impl(gps_rx, &mut packet_parser, &mut buf, &mut fix, &mut pvt_itow, &mut tracker, None).await {
                            error!("GPS Error: {}", Debug2Format(&err));
                        }
                    }
                };
                join(writer, reader).await;
                info!("GPS nav DB restored");
            }
            Ok(None) => info!("No saved GPS nav DB, cold starting"),
            Err(err) => error!("Failed to read GPS nav DB: {}", Debug2Format(&err)),
        }
    }

    //Task loop
    loop {
        //info!("New gps loop iteration");
        let collecting = saver.poll_sent.is_some();
        match gps_impl(gps_rx, &mut packet_parser, &mut buf, &mut fix, &mut pvt_itow, &mut tracker, collecting.then_some(&mut capture)).await {
            Err(err) => {
                error!("GPS Error: {}", Debug2Format(&err));
                // packet_parser = ublox::Parser::new_fixed(); // clear stuck/desynced state
            }
            Ok(_) => {}
        };

        if let Some(flash) = flash {
            saver.tick(gps_tx, flash, &tracker, &mut capture).await;
        }
    }
}

const UBX_CLASS_MGA: u8 = 0x13;
const UBX_ID_MGA_DBD: u8 = 0x80;
const UBX_ID_MGA_INI: u8 = 0x40;

/// Wait this long after a continuous good 3D fix starts before the first save,
/// so the receiver has downloaded orbit data for most visible satellites
const NAV_DB_FIRST_SAVE_AFTER: Duration = Duration::from_secs(120);
/// Time between later saves (orbit data is only good for a few hours)
const NAV_DB_SAVE_INTERVAL: Duration = Duration::from_secs(5 * 60);
/// The dump is complete once no MGA-DBD frame has arrived for this long
const NAV_DB_QUIET: Duration = Duration::from_secs(2);
/// Give up on a dump that hasn't finished in this long
const NAV_DB_TIMEOUT: Duration = Duration::from_secs(30);
/// Minimum satellites in the fix before saving
const NAV_DB_MIN_SATELLITES: u8 = 6;
/// Position accuracy sent with MGA-INI-POS_LLH. Kept loose because the board
/// may be powered up somewhere other than where the database was saved.
const NAV_DB_POS_ACC_CM: u32 = 100_000 * 100;
/// Pause between restored frames so the receiver's input buffer keeps up
const NAV_DB_FRAME_GAP_MS: u64 = 5;

static mut NAV_DB_BUF: [u8; GPS_NAV_DB_MAX_LEN] = [0u8; GPS_NAV_DB_MAX_LEN];

/// Tracks the fix quality the saver needs, updated from NavPvt
#[derive(Default)]
pub struct NavDbTracker {
    last_pos: Option<GpsNavDbPos>,
    good_fix_since: Option<Instant>,
}

/// Collects MGA-DBD frames as complete UBX frames
pub struct DbdCapture<'a> {
    buf: &'a mut [u8],
    len: usize,
    frames: u32,
    overflowed: bool,
    last_frame: Option<Instant>,
}

impl<'a> DbdCapture<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0, frames: 0, overflowed: false, last_frame: None }
    }

    fn reset(&mut self) {
        self.len = 0;
        self.frames = 0;
        self.overflowed = false;
        self.last_frame = None;
    }

    fn push(&mut self, class: u8, id: u8, payload: &[u8]) {
        self.last_frame = Some(Instant::now());
        let needed = payload.len() + 8;
        if self.len + needed > self.buf.len() {
            self.overflowed = true;
            return;
        }
        self.len += write_ubx_frame(&mut self.buf[self.len..], class, id, payload);
        self.frames += 1;
    }
}

#[derive(Default)]
struct NavDbSaver {
    poll_sent: Option<Instant>,
    last_attempt: Option<Instant>,
}

impl NavDbSaver {
    async fn tick(
        &mut self,
        gps_tx: &mut UartTx<'static, Async>,
        flash: &'static Mutex<&'static mut Flash>,
        tracker: &NavDbTracker,
        capture: &mut DbdCapture<'_>,
    ) {
        // Never poll or write flash in flight; the dump also delays NAV messages
        let in_flight = FLASH_LOGGING_ENABLED.load(Ordering::Relaxed);

        if let Some(sent) = self.poll_sent {
            let quiet = capture.last_frame.is_some_and(|t| t.elapsed() > NAV_DB_QUIET);
            if !quiet && sent.elapsed() < NAV_DB_TIMEOUT {
                return;
            }
            self.poll_sent = None;
            self.last_attempt = Some(Instant::now());

            if capture.frames == 0 {
                warn!("GPS nav DB poll got no MGA-DBD frames");
            } else if capture.overflowed {
                warn!("GPS nav DB larger than {} bytes, not saved", capture.buf.len());
            } else if in_flight {
                warn!("GPS nav DB dump finished in flight, not saved");
            } else if let Some(pos) = tracker.last_pos {
                let result = flash.lock().await.save_gps_nav_db(&capture.buf[..capture.len], capture.frames, pos).await;
                if let Err(err) = result {
                    error!("Failed to save GPS nav DB: {}", Debug2Format(&err));
                }
            }
            return;
        }

        if in_flight {
            return;
        }
        let Some(since) = tracker.good_fix_since else { return };
        let due = match self.last_attempt {
            None => since.elapsed() >= NAV_DB_FIRST_SAVE_AFTER,
            Some(t) => t.elapsed() >= NAV_DB_SAVE_INTERVAL,
        };
        if !due {
            return;
        }

        capture.reset();
        let mut poll = [0u8; 8];
        write_ubx_frame(&mut poll, UBX_CLASS_MGA, UBX_ID_MGA_DBD, &[]);
        match gps_tx.write(&poll).await {
            Ok(()) => {
                info!("Polling GPS nav DB");
                self.poll_sent = Some(Instant::now());
            }
            Err(err) => {
                error!("Failed to poll GPS nav DB: {}", Debug2Format(&err));
                self.last_attempt = Some(Instant::now());
            }
        }
    }
}

/// Send the saved position, then every saved MGA-DBD frame
async fn send_nav_db(gps_tx: &mut UartTx<'static, Async>, data: &[u8], pos: GpsNavDbPos) {
    // UBX-MGA-INI-POS_LLH: type 0x01, version 0, 2 reserved, lat, lon, alt, posAcc
    let mut payload = [0u8; 20];
    payload[0] = 0x01;
    payload[4..8].copy_from_slice(&pos.lat_e7.to_le_bytes());
    payload[8..12].copy_from_slice(&pos.lon_e7.to_le_bytes());
    payload[12..16].copy_from_slice(&pos.alt_cm.to_le_bytes());
    payload[16..20].copy_from_slice(&NAV_DB_POS_ACC_CM.to_le_bytes());
    let mut frame = [0u8; 28];
    write_ubx_frame(&mut frame, UBX_CLASS_MGA, UBX_ID_MGA_INI, &payload);
    if let Err(err) = gps_tx.write(&frame).await {
        error!("Failed to send GPS position aid: {}", Debug2Format(&err));
    }
    Timer::after_millis(NAV_DB_FRAME_GAP_MS).await;

    let mut i = 0;
    while i + 8 <= data.len() {
        let len = u16::from_le_bytes([data[i + 4], data[i + 5]]) as usize + 8;
        if data[i] != 0xB5 || data[i + 1] != 0x62 || i + len > data.len() {
            error!("Saved GPS nav DB is malformed at byte {}", i);
            return;
        }
        if let Err(err) = gps_tx.write(&data[i..i + len]).await {
            error!("Failed to send GPS nav DB frame: {}", Debug2Format(&err));
            return;
        }
        i += len;
        Timer::after_millis(NAV_DB_FRAME_GAP_MS).await;
    }
}

/// Write a complete UBX frame into `out`, returning its length
fn write_ubx_frame(out: &mut [u8], class: u8, id: u8, payload: &[u8]) -> usize {
    let len = payload.len();
    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = class;
    out[3] = id;
    out[4..6].copy_from_slice(&(len as u16).to_le_bytes());
    out[6..6 + len].copy_from_slice(payload);
    let (mut ck_a, mut ck_b) = (0u8, 0u8);
    for b in &out[2..6 + len] {
        ck_a = ck_a.wrapping_add(*b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    out[6 + len] = ck_a;
    out[7 + len] = ck_b;
    len + 8
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
    packet_parser: &mut Parser<FixedBuffer<512>, Proto31>,
    buf: &mut [u8],
    fix: &mut GpsFix,
    pvt_itow: &mut u32,
    tracker: &mut NavDbTracker,
    mut capture: Option<&mut DbdCapture<'_>>,
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
                        //println!("ECEF Fix found!");
                        fix.itow = p.itow();
                        // Not actually meters, I think.
                        fix.pos.x.value = p.ecef_x_meters_raw();
                        fix.pos.y.value = p.ecef_y_meters_raw();
                        fix.pos.z.value = p.ecef_z_meters_raw();
                        fix.pos_acc.value = p.p_acc_meters_raw();
                        // Signal once both messages of this epoch have arrived
                        if *pvt_itow == fix.itow {
                            GPS_FIX.signal(fix.clone());
                        }
                    }
                    PacketRef::NavPvt(p) => {
                        //println!("NavPvt packet found!");
                        *pvt_itow = p.itow();
                        fix.lon = p.longitude();
                        fix.lat = p.latitude();
                        fix.satellites = p.num_satellites();
                        match p.fix_type() {
                            GnssFixType::TimeOnlyFix => {
                                fix.fix_type = GpsFixType::TimeOnlyFix;
                            }
                            GnssFixType::GPSPlusDeadReckoning => {
                                fix.fix_type = GpsFixType::FixDifferential;
                            }
                            GnssFixType::NoFix => {
                                fix.fix_type = GpsFixType::NoFix;
                            }
                            GnssFixType::DeadReckoningOnly => {
                                fix.fix_type = GpsFixType::FixPrediction;
                            }
                            GnssFixType::Fix2D => {
                                fix.fix_type = GpsFixType::Fix2d;
                            }
                            GnssFixType::Fix3D => {
                                fix.fix_type = GpsFixType::Fix3d;
                            }
                            _ => {
                                fix.fix_type = GpsFixType::NoFix;
                            }
                        }
                        if !p.flags().contains(NavPvtFlags::GPS_FIX_OK) {
                            fix.fix_type = GpsFixType::NoFix;
                        }
                        // Track fix quality for the nav DB saver
                        if fix.fix_type == GpsFixType::Fix3d && fix.satellites >= NAV_DB_MIN_SATELLITES {
                            tracker.good_fix_since.get_or_insert_with(Instant::now);
                            tracker.last_pos = Some(GpsNavDbPos {
                                lat_e7: (p.latitude() * 1e7) as i32,
                                lon_e7: (p.longitude() * 1e7) as i32,
                                alt_cm: (p.height_above_ellipsoid() * 100.0) as i32,
                            });
                        } else {
                            tracker.good_fix_since = None;
                        }
                        if fix.itow == *pvt_itow {
                            GPS_FIX.signal(fix.clone());
                        }
                    }
                    PacketRef::Unknown(u) if u.class == UBX_CLASS_MGA && u.msg_id == UBX_ID_MGA_DBD => {
                        if let Some(capture) = capture.as_deref_mut() {
                            capture.push(u.class, u.msg_id, u.payload);
                        }
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
            Err(e) => return Err(SirinError::GpsPacketParseError(e)),
        }
    }

    Ok(())
}
