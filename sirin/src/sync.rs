use core::{cell::UnsafeCell, mem::MaybeUninit};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub type Mutex<V> = embassy_sync::mutex::Mutex<CriticalSectionRawMutex, V>;
pub type MutexGuard<'a, V> = embassy_sync::mutex::MutexGuard<'a, CriticalSectionRawMutex, V>;
