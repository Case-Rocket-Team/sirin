#![no_std]
#![doc = include_str!("../README.md")]

use core::ffi::c_float;

extern "C" {
    fn test_sin(x: c_float) -> c_float;
}

pub fn cmsis_dsp_sin(x: f32) -> f32 {
    unsafe {
        test_sin(x)
    }
}