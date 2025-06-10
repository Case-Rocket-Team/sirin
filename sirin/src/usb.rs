use core::mem::MaybeUninit;

use defmt::error;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, peripherals::{self, PA11, PA12, USB_OTG_FS}, usb::{DmPin, DpPin, Driver}, Peripheral};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_usb::{class::cdc_acm::{CdcAcmClass, State}, msos::{self, windows_version}, types::InterfaceNumber, Builder, UsbDevice};
use sirin_shared::{packet::MAX_OUT_PACKET_SIZE, usb::{USB_CLASS, USB_PID, USB_PROTOCOL, USB_SUBCLASS, USB_VID}};
use embassy_usb_driver::EndpointIn as UsbDriverEndpointIn;
use embassy_usb_driver::EndpointOut as UsbDriverEndpointOut;

bind_interrupts!(pub struct Irqs {
    OTG_FS => embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

pub type UsbDriver = Driver<'static, USB_OTG_FS>;
pub type UsbSerialClass = CdcAcmClass<'static, UsbDriver>;
pub type WriteEp = <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn;
pub type ReadEp = <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut;

const DEVICE_INTERFACE_GUIDS: &[&str] = &["{EAA9A5DC-30BA-44BC-9232-606CDC875321}"];

// I wanted to do these statically allocated stuff similar to `main.rs` with
// a local function variable where there's a `loop`, but you can't do that here
// because we need to return the USB device class from the function.
static mut EP_OUT_BUFFER: [u8; MAX_OUT_PACKET_SIZE * 2] = [0; MAX_OUT_PACKET_SIZE * 2];
static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
static mut MSOS_DESCRIPTOR: [u8; 256] = [0; 256];
static mut CONTROL_BUF: [u8; 64] = [0; 64];
static mut STATE: Option<State> = None;

pub struct SirinUsb {
    pub read_ep: ReadEp,
    pub write_ep: WriteEp,
}

/// SAFETY: function may only be called once
#[allow(static_mut_refs)]
pub unsafe fn setup_usb(
    spawner: &Spawner,
    usb_fs: USB_OTG_FS,
    dp: PA12,
    dm: PA11
) -> SirinUsb {
    // SAFETY: all of the static mut's above must only have one `&mut` taken of them.

    let mut usb_config = embassy_stm32::usb::Config::default();
    usb_config.vbus_detection = false;

    let driver = Driver::new_fs(usb_fs, Irqs, dp, dm, &mut EP_OUT_BUFFER, usb_config);
    
    let mut usb_config = embassy_usb::Config::new(USB_VID, USB_PID);
    usb_config.manufacturer = Some("Nautki");
    usb_config.product = Some("Sirin S1-R2");
    usb_config.serial_number = Some("SirinBeta");
    usb_config.max_power = 200;
    usb_config.max_packet_size_0 = 64;
    //usb_config.device_class = 0x02;
    
    STATE = Some(State::new());

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut CONFIG_DESCRIPTOR,
        &mut BOS_DESCRIPTOR,
        &mut MSOS_DESCRIPTOR,
        &mut CONTROL_BUF,
    );

    builder.msos_descriptor(windows_version::WIN10, 2);
    let msos_writer = builder.msos_writer();
    msos_writer.configuration(0);
    msos_writer.function(InterfaceNumber(0));
    msos_writer.function_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    msos_writer.function_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));


    let mut func = builder.function(USB_CLASS, USB_SUBCLASS, USB_PROTOCOL);

    let mut iface = func.interface();

    // Data endpoint
    let mut alt = iface.alt_setting(USB_CLASS, USB_SUBCLASS, USB_PROTOCOL, None);
    let ep_in = alt.endpoint_bulk_in(MAX_OUT_PACKET_SIZE as u16);
    let ep_out = alt.endpoint_bulk_out(MAX_OUT_PACKET_SIZE as u16);
    
    drop(func);

    let usb = builder.build();

    let res = spawner.spawn(usb_task(usb));

    match res {
        Ok(()) => {},
        Err(e) => error!("Failed to init USB: {}", e)
    }

    SirinUsb {
        read_ep: ep_out,
        write_ep: ep_in
    }
}

#[embassy_executor::task]
async fn usb_task(
    mut usb: UsbDevice<'static, Driver<'static, USB_OTG_FS>>
) -> ! {
    usb.run().await;
}