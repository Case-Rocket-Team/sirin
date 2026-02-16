use defmt::info;
use embassy_time::{Duration, Instant};
use sirin_shared::{mode::SirinMode, packet::{Request, Response}, time::AbsoluteTimeReference};

use crate::{Sirin, error::SirinError, io::IoRequest, time::{duration_since_epoch, set_duration_since_epoch}};

impl Sirin {
    pub async fn handle_request(&mut self, request: IoRequest) -> Result<(), SirinError> {
        match request.request {
            Request::DeployMain => {
                //self.parachute_main.set_high();
                request.reply(Response::Ok);
            }
            Request::DeployApo => {
                //self.parachute_apo.set_high();
                request.reply(Response::Ok);
            }
            Request::Null => {}
            Request::Ping => {
                request.reply(Response::Ok);
            }
            Request::SetTime(ref reference) => {
                info!("SetTime packet received!");
                if duration_since_epoch().is_some() {
                    //TODO: request.reply(Response::Error(())));
                    return Ok(());
                }

                // Subtract current uptime from time since boot
                let ms_since_epoch = reference.ms_since_epoch - Instant::now().as_millis();

                set_duration_since_epoch(Duration::from_millis(ms_since_epoch));
                //set_flash_logging_enabled(true);
                info!("Awaiting flash lock...");
                let mut flash = self.io.flash.lock().await;
                info!("Flash locked in main");
                flash
                    .set_absolute_time_reference(AbsoluteTimeReference { ms_since_epoch })
                    .await?;
                info!("Flash task complete");
                request.reply(Response::Ok);
            }
            Request::Reboot => {
                request.reply(Response::Ok);
                Sirin::reboot();
            }
            Request::QueryConfig => {
                request.reply(Response::Config(self.config.clone()));
            }
            Request::SetConfig(ref config) => {
                self.io.flash.lock().await.save_config(&config).await.unwrap();
                request.reply(Response::Ok);
                Sirin::reboot();
            }
            Request::SetMode(m) => {
                self.state.mode = m;
            }
            /*Request::QueryFlights => {
                //info!("QueryFlights packet received!");
                let flash = flash.lock().await;

                //info!("Querying flights...");

                for (i, header) in flash.flight_headers.iter().enumerate() {
                    send_packet(io_packet.reply(Log::FlightHeader(Page::new(
                        i as u16,
                        header.header.clone(),
                    ))));
                }
            }
            Request::ReadFlight(index) => {
                let mut flash = flash.lock().await;
                let Some(header) = flash.flight_headers.get(index as usize) else {
                    send_packet(io_packet.reply(Log::Error(SirinError::FlightNotFound(index))));
                    continue;
                };

                //info!("Reading flight with header: {:?}", Debug2Format(&header));

                // borrow checker :(
                let header = header.clone();

                let mut iter = flash.read_logs(&header);
                while let Some(log) = iter.next().await {
                    //info!("Sent log: {}", Debug2Format(&log));
                    send_packet(io_packet.reply(log?));
                }

                //info!("Done writing logs.")
            }
            Request::Tail(enabled) => {
                set_usb_broadcasting_enabled(enabled);
                continue;
            }
            Request::EraseFlash(..) => {
                let mut flash = flash.lock().await;
                info!("Starting chip erase...");
                flash.w25q.chip_erase().await?;
                info!("Waiting until flash is ready...");
                flash.w25q.until_ready().await?;
                info!("Finished chip erase.");
                send_packet(io_packet.reply(Log::Ok));

                Timer::after_millis(500).await;

                panic!("Reboot");
            }*/
        }

        Ok(())
    }
}
