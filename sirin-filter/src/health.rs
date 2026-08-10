use crate::update::{UpdateDecision, UpdateResult};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AidingStatus {
    Inactive,
    Suspect,
    Active,
}

#[derive(Debug, Clone, Copy)]
pub struct AidingHealth {
    pub status: AidingStatus,
    accepted_streak: u8,
    rejected_streak: u8,
    pub total_accepted: u32,
    pub total_rejected: u32,
    activate_after: u8,
    deactivate_after: u8,
}

impl Default for AidingHealth {
    fn default() -> Self {
        Self {
            status: AidingStatus::Inactive,
            accepted_streak: 0,
            rejected_streak: 0,
            total_accepted: 0,
            total_rejected: 0,
            activate_after: 3,
            deactivate_after: 3,
        }
    }
}

impl AidingHealth {
    pub fn record(&mut self, result: UpdateResult) {
        if result.accepted {
            self.total_accepted = self.total_accepted.saturating_add(1);
            self.accepted_streak = self.accepted_streak.saturating_add(1);
            self.rejected_streak = 0;
            if self.accepted_streak >= self.activate_after {
                self.status = AidingStatus::Active;
            } else if self.status == AidingStatus::Inactive {
                self.status = AidingStatus::Suspect;
            }
        } else {
            self.total_rejected = self.total_rejected.saturating_add(1);
            self.rejected_streak = self.rejected_streak.saturating_add(1);
            self.accepted_streak = 0;
            if self.rejected_streak >= self.deactivate_after {
                self.status = AidingStatus::Inactive;
            } else {
                self.status = AidingStatus::Suspect;
            }
        }
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn last_decision_is_fault(result: UpdateResult) -> bool {
        matches!(
            result.reason,
            UpdateDecision::NonFiniteInput
                | UpdateDecision::InnovationNotPositiveDefinite
                | UpdateDecision::NonFiniteResult
        )
    }
}

#[derive(Debug, Clone, Default)]
pub struct EstimatorHealth {
    pub gps: AidingHealth,
    pub barometer: AidingHealth,
    pub magnetometer: AidingHealth,
    pub imu_saturated: bool,
    pub covariance_valid: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn accepted() -> UpdateResult {
        UpdateResult {
            accepted: true,
            nis: 1.0,
            gate: 9.0,
            correction_norm: 0.1,
            reason: UpdateDecision::Accepted,
        }
    }

    fn rejected() -> UpdateResult {
        UpdateResult {
            accepted: false,
            nis: 20.0,
            gate: 9.0,
            correction_norm: 0.0,
            reason: UpdateDecision::InnovationRejected,
        }
    }

    #[test]
    fn aiding_requires_good_streak_and_drops_after_bad_streak() {
        let mut health = AidingHealth::default();
        health.record(accepted());
        health.record(accepted());
        assert_eq!(health.status, AidingStatus::Suspect);
        health.record(accepted());
        assert_eq!(health.status, AidingStatus::Active);
        health.record(rejected());
        health.record(rejected());
        assert_eq!(health.status, AidingStatus::Suspect);
        health.record(rejected());
        assert_eq!(health.status, AidingStatus::Inactive);
    }
}
