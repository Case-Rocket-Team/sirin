//! Wall clock time. We don't always have the absolute available, so what happens is that we'll normally
//! just use a timer from system boot. A host computer can hint to us the the wall clock time, and then
//! we can figure out retroactively what the boot time was and then the other recorded times from there.
//! If the time is never hinted, we'll still have the relative time.

use embassy_time::{Duration, TICK_HZ};

use embassy_sync::once_lock::OnceLock;
use sirin_shared::time::AbsoluteTimeReference;

static DURATION_SINCE_EPOCH: OnceLock<Duration> = OnceLock::new();

pub(crate) fn set_duration_since_epoch(duration: Duration) {
    DURATION_SINCE_EPOCH.init(duration).unwrap()
}

/// The duration since the Unix epoch, if known.
pub fn duration_since_epoch() -> Option<&'static Duration> {
    DURATION_SINCE_EPOCH.try_get()
}

pub fn absolute_time_reference() -> Option<AbsoluteTimeReference> {
    let dur = duration_since_epoch()?;
    Some(AbsoluteTimeReference {
        ms_since_epoch: dur.as_millis(),
    })
}