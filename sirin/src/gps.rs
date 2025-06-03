use defmt::info;
use embassy_executor::task;
use embassy_stm32::{mode::Async, usart::Uart};

#[task]
pub async fn gps_task(
    gps: &'static mut Uart<'static, Async>
) {
    loop {
        let mut response = [0u8; 1024];
        let _ = gps.read_until_idle(&mut response).await;

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