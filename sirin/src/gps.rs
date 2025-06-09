use defmt::{error, info};
use embassy_executor::task;
use embassy_stm32::{mode::Async, pac::Interrupt::PVD_AVD, usart::Uart};
use embassy_time::Instant;
use sirin_shared::packet::{Log, OutPacket};

use crate::io::{broadcast, broadcast_log};

#[task]
pub async fn gps_task(
    gps: &'static mut Uart<'static, Async>
) {
    gps.write(&[0xA0, 0xA1, 0x00, 0x03, 0x09, 0x02, 0x00, 0x09 ^ 0x02, 0x0D, 0x0A]).await.unwrap();

    loop {
        let mut response = [0u8; 512];
        let Ok(len) = gps.read_until_idle(&mut response).await else {
            //error!("Could not read GPS");
            continue;
        };
        info!("{:x}", response[..len]);
        
        /*let Ok(str) = core::str::from_utf8(&response[..len]) else {
            error!("Could not convert to UTF8");
            continue;
        };
        info!("{}", str);

        // split up messages
        let mut start = 0;
        let mut i = 1;
        loop {
            if i >= len || (response[i - 1] == b'\r' && response[i] == b'\n') {
                // end of message
                let mut message = [0; 200];
                message[0..(i - start)].copy_from_slice(&response[start..i]);
                broadcast_log(Instant::now(), Log::GpsNmea(message));
                start = i;
            }

            if i >= len {
                break;
            }

            i += 1;
        }

        continue;*/

        let mut k = 0;
        while response[k] == 0xA0 && response[k + 1] == 0x0A1 {
            let len = ((response[k + 2] as u16) << 8) + response[k + 3] as u16;
            
            //info!("{:x}", &response[k..((len + 7) as usize + k)]);
            let id = response[k + 4];
            
            match id {
                0xDF => {
                    let msg = match response[k + 6] {
                        0x00 => "no fix",
                        0x01 => "fix prediction",
                        0x02 => "2d fix",
                        0x03 => "3d fix",
                        0x04 => "differential fix",
                        _ => "unknown"
                    };
                    
                    info!("fix status: {} ({})", msg, response[k + 6])
                },
                0xDE => {
                    let nsvs = response[k + 6];
                    let chsize = 8;
                    for i in 0..nsvs {
                        let offset = k + 7 + (chsize as usize * i as usize);
                        let chid = response[offset];
                        let svid = response[offset + 1];
                        let status = response[offset + 2];

                        let almanac = status & 0b001 > 0;
                        let ephemeris = status & 0b010 > 0;
                        let healthy = status & 0b100 > 0;

                        info!(
                            "GPS (chid: {}, svid: {}): [{}] almanac, [{}] ephemeris, [{}] healthy", 
                            chid, svid, almanac, ephemeris, healthy
                        )
                    }
                    info!("{} satellites total", nsvs)
                },
                _ => {}
            }

            k += len as usize + 7;
        }
    }
}