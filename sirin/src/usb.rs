use core::mem::MaybeUninit;

use defmt::error;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, peripherals::{self, PA11, PA12, USB_OTG_FS}, usb::{DmPin, DpPin, Driver}, Peripheral};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_usb::{class::cdc_acm::{CdcAcmClass, State}, Builder, UsbDevice};
use sirin_shared::song::MAX_OUT_PACKET_SIZE;

bind_interrupts!(pub struct Irqs {
    OTG_FS => embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

pub type UsbSerialClass = CdcAcmClass<'static, Driver<'static, USB_OTG_FS>>;

// I wanted to do these statically allocated stuff similar to `main.rs` with
// a local function variable where there's a `loop`, but you can't do that here
// because we need to return the USB device class from the function.
static mut EP_OUT_BUFFER: [u8; MAX_OUT_PACKET_SIZE * 2] = [0; MAX_OUT_PACKET_SIZE * 2];
static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
static mut CONTROL_BUF: [u8; 64] = [0; 64];
static mut STATE: Option<State> = None;

/// SAFETY: function may only be called once
#[allow(static_mut_refs)]
pub unsafe fn usb_serial(
    spawner: &Spawner,
    usb_fs: USB_OTG_FS,
    dp: PA12,
    dm: PA11
) -> UsbSerialClass {
    // SAFETY: all of the static mut's above must only have one `&mut` taken of them.

    let mut usb_config = embassy_stm32::usb::Config::default();
    usb_config.vbus_detection = false;

    let driver = Driver::new_fs(usb_fs, Irqs, dp, dm, &mut EP_OUT_BUFFER, usb_config);
    
    let mut usb_config = embassy_usb::Config::new(0x12A3, 0x0232);
    usb_config.manufacturer = Some("Nautki");
    usb_config.product = Some("Sirin S1-R2");
    
    STATE = Some(State::new());

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut CONFIG_DESCRIPTOR,
        &mut BOS_DESCRIPTOR,
        &mut [], // no msos descriptors
        &mut CONTROL_BUF,
    );

    let class = CdcAcmClass::new(&mut builder, STATE.as_mut().unwrap(), MAX_OUT_PACKET_SIZE as u16);

    let usb = builder.build();

    let res = spawner.spawn(usb_serial_task(usb));

    match res {
        Ok(()) => {},
        Err(e) => error!("Failed to init USB: {}", e)
    }

    class
}

#[embassy_executor::task]
async fn usb_serial_task(
    mut usb: UsbDevice<'static, Driver<'static, USB_OTG_FS>>
) -> ! {
    usb.run().await;
}