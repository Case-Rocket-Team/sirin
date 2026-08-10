use crate::{
    measurement::{
        GpsPositionObservation, GpsVelocityObservation, ImuSample,
    },
    update::{UpdateDecision, UpdateResult},
    Eskf, PropagationError,
};

#[derive(Debug, Clone, Copy)]
pub enum HistoryProcessResult {
    Propagated,
    PropagationRejected(PropagationError),
    Measurement(UpdateResult),
}

#[derive(Debug, Clone)]
struct HistoryEntry {
    timestamp_us: u64,
    imu: ImuSample,
    filter: Eskf,
}

/// Fixed-capacity delayed-measurement history.
///
/// Each entry stores the complete filter snapshot after one IMU sample. A
/// delayed GNSS update restores the newest snapshot at or before its epoch,
/// applies the correction, then deterministically replays subsequent IMU
/// samples. Measurements older than the retained horizon are rejected.
#[derive(Debug, Clone)]
pub struct FixedLagHistory<const N: usize> {
    pub filter: Eskf,
    entries: [Option<HistoryEntry>; N],
    len: usize,
}

impl<const N: usize> FixedLagHistory<N> {
    pub fn new(filter: Eskf) -> Self {
        Self {
            filter,
            entries: core::array::from_fn(|_| None),
            len: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn process_imu(&mut self, sample: ImuSample) -> HistoryProcessResult {
        match self.filter.propagate_imu(sample) {
            Ok(()) => {
                if N != 0 {
                    self.push(HistoryEntry {
                        timestamp_us: sample.timestamp_us,
                        imu: sample,
                        filter: self.filter.clone(),
                    });
                }
                HistoryProcessResult::Propagated
            }
            Err(error) => HistoryProcessResult::PropagationRejected(error),
        }
    }

    pub fn fuse_gps_position(
        &mut self,
        observation: &GpsPositionObservation,
    ) -> HistoryProcessResult {
        let result = self.rewind_and_replay_position(observation);
        HistoryProcessResult::Measurement(result)
    }

    pub fn fuse_gps_velocity(
        &mut self,
        observation: &GpsVelocityObservation,
    ) -> HistoryProcessResult {
        let result = self.rewind_and_replay_velocity(observation);
        HistoryProcessResult::Measurement(result)
    }

    fn rewind_and_replay_position(
        &mut self,
        observation: &GpsPositionObservation,
    ) -> UpdateResult {
        let Some(index) = self.snapshot_index(observation.timestamp_us) else {
            return UpdateResult::rejected(
                UpdateDecision::MeasurementTooOld,
                f32::NAN,
                self.filter.config.gps_position_gate,
            );
        };

        self.filter = self.entries[index]
            .as_ref()
            .expect("history index is valid")
            .filter
            .clone();
        let result = self.filter.fuse_gps_position_unchecked(observation);
        if !result.accepted {
            return result;
        }
        if let Some(entry) = self.entries[index].as_mut() {
            entry.filter = self.filter.clone();
        }
        self.replay_after(index);
        result
    }

    fn rewind_and_replay_velocity(
        &mut self,
        observation: &GpsVelocityObservation,
    ) -> UpdateResult {
        let Some(index) = self.snapshot_index(observation.timestamp_us) else {
            return UpdateResult::rejected(
                UpdateDecision::MeasurementTooOld,
                f32::NAN,
                self.filter.config.gps_velocity_gate,
            );
        };

        self.filter = self.entries[index]
            .as_ref()
            .expect("history index is valid")
            .filter
            .clone();
        let result = self.filter.fuse_gps_velocity_unchecked(observation);
        if !result.accepted {
            return result;
        }
        if let Some(entry) = self.entries[index].as_mut() {
            entry.filter = self.filter.clone();
        }
        self.replay_after(index);
        result
    }

    fn replay_after(&mut self, index: usize) {
        for replay_index in (index + 1)..self.len {
            let Some(sample) = self.entries[replay_index].as_ref().map(|entry| entry.imu) else {
                continue;
            };
            if self.filter.propagate_imu(sample).is_ok() {
                if let Some(updated) = self.entries[replay_index].as_mut() {
                    updated.filter = self.filter.clone();
                }
            }
        }
    }

    fn snapshot_index(&self, timestamp_us: u64) -> Option<usize> {
        if self.len == 0 {
            return None;
        }
        if let Some(current_time_us) = self.filter.timestamp_us() {
            if timestamp_us
                > current_time_us
                    .saturating_add(self.filter.config.future_measurement_tolerance_us)
            {
                return None;
            }
        }
        let mut selected = None;
        for index in 0..self.len {
            let Some(entry) = self.entries[index].as_ref() else {
                continue;
            };
            if entry.timestamp_us <= timestamp_us {
                selected = Some(index);
            }
        }
        selected
    }

    fn push(&mut self, entry: HistoryEntry) {
        if self.len < N {
            self.entries[self.len] = Some(entry);
            self.len += 1;
            return;
        }

        for index in 1..N {
            let moved = self.entries[index].take();
            self.entries[index - 1] = moved;
        }
        self.entries[N - 1] = Some(entry);
    }
}

impl<const N: usize> Default for FixedLagHistory<N> {
    fn default() -> Self {
        Self::new(Eskf::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::Vec3;

    fn imu(timestamp_us: u64, sequence: u32) -> ImuSample {
        ImuSample {
            timestamp_us,
            accel_mps2_b: Vec3::new(0.0, 0.0, -9.80665),
            gyro_radps_b: Vec3::zeros(),
            temperature_c: None,
            accel_saturated: [false; 3],
            gyro_saturated: [false; 3],
            sequence,
        }
    }

    #[test]
    fn delayed_position_is_replayed_to_current_time() {
        let mut history = FixedLagHistory::<8>::default();
        assert!(matches!(
            history.process_imu(imu(0, 0)),
            HistoryProcessResult::Propagated
        ));
        assert!(matches!(
            history.process_imu(imu(10_000, 1)),
            HistoryProcessResult::Propagated
        ));
        assert!(matches!(
            history.process_imu(imu(20_000, 2)),
            HistoryProcessResult::Propagated
        ));

        let result = history.fuse_gps_position(&GpsPositionObservation {
            timestamp_us: 10_000,
            position_ned_m: Vec3::new(1.0, 0.0, 0.0),
            variance_m2: Vec3::repeat(0.1),
        });

        assert!(matches!(
            result,
            HistoryProcessResult::Measurement(UpdateResult { accepted: true, .. })
        ));
        assert!(history.filter.state.pos.x > 0.0);
        assert_eq!(history.filter.timestamp_us(), Some(20_000));
    }
}
