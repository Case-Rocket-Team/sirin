#![no_std]

pub mod state;
pub mod song;
pub mod packet;

pub const USB_CLASS: u8 = 0xff;
pub const USB_SUBCLASS: u8 = 0x01;
pub const USB_PROTOCOL: u8 = 0x01;
pub const USB_VID: u16 = 0x12A3;
pub const USB_PID: u16 = 0x0232;