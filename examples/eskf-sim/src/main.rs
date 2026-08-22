use sirin_filter::{
    FixedLagHistory, GpsPositionObservation, GpsVelocityObservation,
    HistoryProcessResult, ImuSample, MagnetometerObservation,
    BarometerObservation, Vec3,
};

#[derive(Debug, Clone, Copy)]
pub struct SimulationResult {
    pub final_position_ned_m: Vec3,
    pub final_velocity_ned_mps: Vec3,
    pub true_position_ned_m: Vec3,
    pub true_velocity_ned_mps: Vec3,
    pub accepted_gps_updates: u32,
}

fn truth_at(time_s: f32) -> (Vec3, Vec3) {
    if time_s <= 1.0 {
        (Vec3::zeros(), Vec3::zeros())
    } else if time_s <= 3.0 {
        let elapsed = time_s - 1.0;
        (
            Vec3::new(0.5 * elapsed * elapsed, 0.0, 0.0),
            Vec3::new(elapsed, 0.0, 0.0),
        )
    } else {
        (
            Vec3::new(2.0 + 2.0 * (time_s - 3.0), 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
        )
    }
}

fn run_deterministic_simulation() -> SimulationResult {
    let mut history = FixedLagHistory::<128>::default();
    let mut accepted_gps_updates = 0;
    let dt_s = 0.01;
    let gravity = 9.80665;

    for sample_index in 0..500u32 {
        let time_s = sample_index as f32 * dt_s;
        let (true_position, true_velocity) = truth_at(time_s);
        let specific_force_x = if (1.0..3.0).contains(&time_s) {
            1.0
        } else {
            0.0
        };

        let imu = ImuSample {
            timestamp_us: sample_index as u64 * 10_000,
            accel_mps2_b: Vec3::new(specific_force_x, 0.0, -gravity),
            gyro_radps_b: Vec3::zeros(),
            temperature_c: None,
            accel_saturated: [false; 3],
            gyro_saturated: [false; 3],
            sequence: sample_index,
        };
        assert!(matches!(
            history.process_imu(imu),
            HistoryProcessResult::Propagated
        ));

        if sample_index >= 5 && sample_index % 10 == 0 {
            let delayed_index = sample_index - 3;
            let delayed_time_s = delayed_index as f32 * dt_s;
            let (delayed_position, delayed_velocity) = truth_at(delayed_time_s);

            let position_result = history.fuse_gps_position(&GpsPositionObservation {
                timestamp_us: delayed_index as u64 * 10_000,
                position_ned_m: delayed_position,
                variance_m2: Vec3::repeat(0.04),
            });
            let velocity_result = history.fuse_gps_velocity(&GpsVelocityObservation {
                timestamp_us: delayed_index as u64 * 10_000,
                velocity_ned_mps: delayed_velocity,
                variance_m2ps2: Vec3::repeat(0.04),
            });

            if let HistoryProcessResult::Measurement(result) = position_result {
                accepted_gps_updates += if result.accepted { 1 } else { 0 };
            }
            if let HistoryProcessResult::Measurement(result) = velocity_result {
                accepted_gps_updates += if result.accepted { 1 } else { 0 };
            }

            let barometer_result = history.filter.fuse_barometer(
                &BarometerObservation {
                    timestamp_us: sample_index as u64 * 10_000,
                    height_up_m: 0.0,
                    variance_m2: 0.25,
                },
            );
            assert!(barometer_result.accepted);

            let magnetometer_result = history.filter.fuse_magnetometer(
                &MagnetometerObservation {
                    timestamp_us: sample_index as u64 * 10_000,
                    field_ut_b: Vec3::new(19.0, -2.7, 48.0),
                    variance_ut2: Vec3::repeat(1.0),
                },
            );
            assert!(magnetometer_result.accepted);
        }

        let _ = (true_position, true_velocity);
    }

    let (true_position_ned_m, true_velocity_ned_mps) = truth_at(4.99);
    SimulationResult {
        final_position_ned_m: history.filter.state.pos,
        final_velocity_ned_mps: history.filter.state.vel,
        true_position_ned_m,
        true_velocity_ned_mps,
        accepted_gps_updates,
    }
}

fn main() {
    let result = run_deterministic_simulation();
    println!("accepted delayed GPS updates: {}", result.accepted_gps_updates);
    println!("final position NED: {:?}", result.final_position_ned_m);
    println!("truth position NED: {:?}", result.true_position_ned_m);
    println!("final velocity NED: {:?}", result.final_velocity_ned_mps);
    println!("truth velocity NED: {:?}", result.true_velocity_ned_mps);

    assert!(result.accepted_gps_updates > 50);
    assert!((result.final_position_ned_m - result.true_position_ned_m).norm() < 1.0);
    assert!((result.final_velocity_ned_mps - result.true_velocity_ned_mps).norm() < 0.5);

    println!("\n✓ Simulation passed all assertions!");
}
