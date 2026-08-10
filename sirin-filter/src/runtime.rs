use crate::{
    health::EstimatorHealth,
    measurement::{
        BarometerObservation, GpsPositionObservation, GpsVelocityObservation,
        ImuSample, MagnetometerObservation,
    },
    Eskf, PropagationError, UpdateResult,
};

#[derive(Debug, Clone, Copy)]
pub enum EstimatorEvent {
    Imu(ImuSample),
    GpsVelocity(GpsVelocityObservation),
    GpsPosition(GpsPositionObservation),
    Barometer(BarometerObservation),
    Magnetometer(MagnetometerObservation),
}

impl EstimatorEvent {
    pub fn timestamp_us(&self) -> u64 {
        match self {
            Self::Imu(value) => value.timestamp_us,
            Self::GpsVelocity(value) => value.timestamp_us,
            Self::GpsPosition(value) => value.timestamp_us,
            Self::Barometer(value) => value.timestamp_us,
            Self::Magnetometer(value) => value.timestamp_us,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum ProcessResult {
    Propagated,
    PropagationRejected(PropagationError),
    Measurement(UpdateResult),
}

#[derive(Debug, Clone, Default)]
pub struct EstimatorDiagnostics {
    pub imu_samples: u64,
    pub propagation_rejections: u32,
    pub accepted_measurements: u32,
    pub rejected_measurements: u32,
    pub last_event_timestamp_us: Option<u64>,
    pub last_propagation_error: Option<PropagationError>,
}

/// Deterministic event processor shared by firmware and host replay.
#[derive(Debug, Clone)]
pub struct EstimatorRuntime {
    pub filter: Eskf,
    pub diagnostics: EstimatorDiagnostics,
    pub health: EstimatorHealth,
}

impl EstimatorRuntime {
    pub fn new(filter: Eskf) -> Self {
        Self {
            filter,
            diagnostics: EstimatorDiagnostics::default(),
            health: EstimatorHealth::default(),
        }
    }

    pub fn process(&mut self, event: EstimatorEvent) -> ProcessResult {
        self.diagnostics.last_event_timestamp_us = Some(event.timestamp_us());
        let result = match event {
            EstimatorEvent::Imu(sample) => {
                self.diagnostics.imu_samples =
                    self.diagnostics.imu_samples.saturating_add(1);
                match self.filter.propagate_imu(sample) {
                    Ok(()) => ProcessResult::Propagated,
                    Err(error) => {
                        self.diagnostics.propagation_rejections = self
                            .diagnostics
                            .propagation_rejections
                            .saturating_add(1);
                        self.diagnostics.last_propagation_error = Some(error);
                        self.health.imu_saturated =
                            matches!(error, PropagationError::SaturatedInput);
                        ProcessResult::PropagationRejected(error)
                    }
                }
            }
            EstimatorEvent::GpsVelocity(observation) => {
                let result = self.filter.fuse_gps_velocity(&observation);
                self.health.gps.record(result);
                self.record_update(result)
            }
            EstimatorEvent::GpsPosition(observation) => {
                let result = self.filter.fuse_gps_position(&observation);
                self.health.gps.record(result);
                self.record_update(result)
            }
            EstimatorEvent::Barometer(observation) => {
                let result = self.filter.fuse_barometer(&observation);
                self.health.barometer.record(result);
                self.record_update(result)
            }
            EstimatorEvent::Magnetometer(observation) => {
                let result = self.filter.fuse_magnetometer(&observation);
                self.health.magnetometer.record(result);
                self.record_update(result)
            }
        };
        self.health.covariance_valid = self.filter.covariance.is_finite();
        result
    }

    fn record_update(&mut self, result: UpdateResult) -> ProcessResult {
        if result.accepted {
            self.diagnostics.accepted_measurements =
                self.diagnostics.accepted_measurements.saturating_add(1);
        } else {
            self.diagnostics.rejected_measurements =
                self.diagnostics.rejected_measurements.saturating_add(1);
        }
        ProcessResult::Measurement(result)
    }
}

impl Default for EstimatorRuntime {
    fn default() -> Self {
        Self::new(Eskf::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{state::Vec3, GpsPositionObservation};

    fn imu(timestamp_us: u64, sequence: u32) -> EstimatorEvent {
        EstimatorEvent::Imu(ImuSample {
            timestamp_us,
            accel_mps2_b: Vec3::new(0.0, 0.0, -9.80665),
            gyro_radps_b: Vec3::zeros(),
            temperature_c: None,
            accel_saturated: [false; 3],
            gyro_saturated: [false; 3],
            sequence,
        })
    }

    #[test]
    fn identical_event_streams_are_deterministic() {
        let mut first = EstimatorRuntime::default();
        let mut second = EstimatorRuntime::default();
        let events = [
            imu(0, 0),
            imu(10_000, 1),
            EstimatorEvent::GpsPosition(GpsPositionObservation {
                timestamp_us: 10_000,
                position_ned_m: Vec3::new(1.0, 2.0, -3.0),
                variance_m2: Vec3::repeat(1.0),
            }),
        ];

        for event in events {
            first.process(event);
            second.process(event);
        }

        assert_eq!(first.filter.state.pos, second.filter.state.pos);
        assert_eq!(
            first.filter.covariance.data,
            second.filter.covariance.data
        );
        assert_eq!(
            first.diagnostics.accepted_measurements,
            second.diagnostics.accepted_measurements
        );
    }
}
