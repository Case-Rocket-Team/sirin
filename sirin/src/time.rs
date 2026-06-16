//! Wall clock time. We don't always have the absolute available, so what happens is that we'll normally
//! just use a timer from system boot. A host computer can hint to us the the wall clock time, and then
//! we can figure out retroactively what the boot time was and then the other recorded times from there.
//! If the time is never hinted, we'll still have the relative time.

use core::cell::UnsafeCell;

use embassy_time::{Duration, TICK_HZ};

use sirin_shared::time::AbsoluteTimeReference;
use defmt::info;

// this is safe bc the references to it only live for it during the duration of the function
// so its impossible for two mut refs to exist simultaneously
static mut DURATION_SINCE_EPOCH: Option<Duration> = None;

pub fn set_duration_since_epoch(duration: Duration) {
    unsafe {
        DURATION_SINCE_EPOCH = Some(duration);
    }
}

/// The duration since the Unix epoch, if known.
pub fn duration_since_epoch() -> Option<Duration> {
    unsafe {
        DURATION_SINCE_EPOCH
    }
}

pub fn absolute_time_reference() -> Option<AbsoluteTimeReference> {
    info!("Shitass code ran");
    let dur = duration_since_epoch()?;
    info!("Duration since epoch: {}", &dur.as_millis());
    Some(AbsoluteTimeReference {
        ms_since_epoch: dur.as_millis(),
    })
}