fn main() {
    let result = eskf_sim::run_deterministic_simulation();
    println!("accepted delayed GPS updates: {}", result.accepted_gps_updates);
    println!("final position NED: {:?}", result.final_position_ned_m);
    println!("truth position NED: {:?}", result.true_position_ned_m);
    println!("final velocity NED: {:?}", result.final_velocity_ned_mps);
    println!("truth velocity NED: {:?}", result.true_velocity_ned_mps);

    assert!(result.accepted_gps_updates > 50);
    assert!((result.final_position_ned_m - result.true_position_ned_m).norm() < 1.0);
    assert!((result.final_velocity_ned_mps - result.true_velocity_ned_mps).norm() < 0.5);
}
