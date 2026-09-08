//! Flight phase detection: `Standby -> Flight -> Descent -> Landed`.
//!
//! [`FlightDetector`] is the single place that decides which [`SirinMode`] the rocket is in and
//! when the parachutes should be fired. It is deliberately pure -- no hardware, no timers, no
//! I/O -- so the whole state machine can be unit tested on the host:
//!
//! ```sh
//! cargo test -p sirin-shared --target <your host triple>
//! ```
//!
//! The flight binaries in `examples/` feed it one [`FlightInput`] per control-loop tick and act
//! on the returned [`FlightUpdate`] (fire charges, toggle the LED, start/stop flash logging).
//!
//! # Phases
//!
//! * **Standby** -- on the pad. Launch is detected when the altitude above the pad exceeds
//!   [`FlightParams::launch_altitude`] or the acceleration exceeds
//!   [`FlightParams::launch_accel_squared`].
//! * **Flight** -- ascending. The running maximum altitude is tracked. Apogee is declared once
//!   the altitude has fallen [`FlightParams::apogee_error`] below that maximum *and*
//!   [`FlightParams::apogee_lockout_ms`] has elapsed since launch (the lockout keeps
//!   boost-phase pressure transients from declaring an early apogee). Declaring apogee fires
//!   the apogee charge and enters `Descent`.
//! * **Descent** -- past apogee. The recorded apogee is corrected upwards if the rocket turns
//!   out to still be climbing, the main charge fires when the altitude drops below
//!   [`FlightParams::main_deployment_altitude`], and landing is declared once the altitude has
//!   stayed within [`FlightParams::landed_altitude_band`] for
//!   [`FlightParams::landed_stable_ms`].
//! * **Landed** -- on the ground. Nothing further happens.
//!
//! [`FlightParams::flight_duration_ms`] after launch the flight is declared over from either
//! airborne phase, so logging always stops eventually even if the sensors misbehave.
//!
//! An operator can force any mode with `InPacket::SetMode`. A forced transition never fires a
//! charge by itself, but the detection logic keeps running in the new mode, so forcing
//! `Descent` below the main deployment altitude will fire the main charge on the next tick.

use uunit::{Meters, WithUnits};

use crate::mode::SirinMode;

/// Tunable thresholds for flight phase detection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightParams {
    /// Altitude above the pad above which a launch is detected, in meters.
    pub launch_altitude: f64,
    /// Squared magnitude of the IMU acceleration above which a launch is detected, in G².
    pub launch_accel_squared: f64,
    /// How far the altitude must fall below the running maximum before apogee is declared, in
    /// meters. This is the hysteresis that keeps barometric noise from declaring apogee.
    pub apogee_error: f64,
    /// Time after launch during which apogee is never declared, in milliseconds.
    pub apogee_lockout_ms: u64,
    /// Altitude below which the main parachute is deployed during descent, in meters.
    /// `None` means the detector never asks for a main deployment.
    pub main_deployment_altitude: Option<f64>,
    /// Altitude band that counts as "not moving" for landing detection, in meters.
    pub landed_altitude_band: f64,
    /// How long the altitude must stay inside `landed_altitude_band` before landing is
    /// declared, in milliseconds.
    pub landed_stable_ms: u64,
    /// Time after launch at which the flight is declared over regardless of the sensors, in
    /// milliseconds.
    pub flight_duration_ms: u64,
}

/// Measurements fed to the detector once per control-loop tick.
#[derive(Clone, Copy, Debug)]
pub struct FlightInput {
    /// Milliseconds since boot.
    pub now_ms: u64,
    /// Barometric altitude above the pad, if the barometer produced a reading this tick.
    pub altitude: Option<Meters<f64>>,
    /// Squared magnitude of the IMU acceleration in G², if the IMU produced a reading this tick.
    pub accel_squared: Option<f64>,
    /// A mode requested by the operator (`InPacket::SetMode`), if one arrived this tick.
    pub requested_mode: Option<SirinMode>,
}

/// What the flight computer should do after a tick.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FlightUpdate {
    /// The mode entered during this tick, if the mode changed.
    pub new_mode: Option<SirinMode>,
    /// Apogee was declared this tick: fire the apogee charge.
    pub deploy_apo: bool,
    /// The main deployment altitude was crossed this tick: fire the main charge.
    pub deploy_main: bool,
}

/// The flight phase state machine. See the [module documentation](self).
#[derive(Clone, Debug)]
pub struct FlightDetector {
    params: FlightParams,
    mode: SirinMode,
    /// Most recent barometric altitude above the pad.
    altitude: Meters<f64>,
    /// Highest altitude seen since launch.
    max_altitude: Meters<f64>,
    /// Altitude at which apogee was declared, corrected upwards if the rocket kept climbing.
    apogee: Option<Meters<f64>>,
    /// Time of launch, in milliseconds since boot.
    launched_at_ms: Option<u64>,
    /// The main deployment altitude has already been crossed this flight.
    main_altitude_reached: bool,
    /// Reference altitude and the time it was taken, for landing detection.
    landed_anchor: Option<(Meters<f64>, u64)>,
}

impl FlightDetector {
    /// A detector in `Standby` on the pad.
    pub fn new(params: FlightParams) -> Self {
        let zero: Meters<f64> = 0.0.with_units();

        Self {
            params,
            mode: SirinMode::Standby,
            altitude: zero,
            max_altitude: zero,
            apogee: None,
            launched_at_ms: None,
            main_altitude_reached: false,
            landed_anchor: None,
        }
    }

    pub fn params(&self) -> &FlightParams {
        &self.params
    }

    pub fn mode(&self) -> SirinMode {
        self.mode
    }

    /// The apogee altitude, once apogee has been declared.
    pub fn apogee(&self) -> Option<Meters<f64>> {
        self.apogee
    }

    /// Highest altitude seen since launch.
    pub fn max_altitude(&self) -> Meters<f64> {
        self.max_altitude
    }

    /// Time of launch in milliseconds since boot, once launched.
    pub fn launched_at_ms(&self) -> Option<u64> {
        self.launched_at_ms
    }

    /// Advance the state machine by one tick.
    pub fn update(&mut self, input: FlightInput) -> FlightUpdate {
        let FlightInput { now_ms, altitude, accel_squared, requested_mode } = input;
        let mut out = FlightUpdate::default();

        if let Some(altitude) = altitude {
            self.altitude = altitude;
            if self.mode.is_airborne() && altitude.value > self.max_altitude.value {
                self.max_altitude = altitude;
            }
        }

        match self.mode {
            SirinMode::Standby => {
                let launched = altitude.is_some_and(|a| a.value > self.params.launch_altitude)
                    || accel_squared.is_some_and(|a| a > self.params.launch_accel_squared);

                if launched {
                    self.enter(SirinMode::Flight, now_ms, &mut out);
                }
            }
            SirinMode::Flight => {
                if self.flight_timed_out(now_ms) {
                    self.enter(SirinMode::Landed, now_ms, &mut out);
                } else if let Some(altitude) = altitude {
                    let descending = self.max_altitude.value > altitude.value + self.params.apogee_error;

                    if descending && self.past_apogee_lockout(now_ms) {
                        self.apogee = Some(self.max_altitude);
                        out.deploy_apo = true;
                        self.enter(SirinMode::Descent, now_ms, &mut out);

                        // A low apogee can already be below the main deployment altitude.
                        self.check_main_altitude(altitude, &mut out);
                    }
                }
            }
            SirinMode::Descent => {
                if self.flight_timed_out(now_ms) {
                    self.enter(SirinMode::Landed, now_ms, &mut out);
                } else if let Some(altitude) = altitude {
                    // Apogee was declared while the rocket was still climbing (e.g. because of a
                    // pressure transient): keep the recorded apogee honest.
                    if let Some(apogee) = self.apogee {
                        if self.max_altitude.value > apogee.value + self.params.apogee_error {
                            self.apogee = Some(self.max_altitude);
                        }
                    }

                    self.check_main_altitude(altitude, &mut out);
                    self.check_landed(altitude, now_ms, &mut out);
                }
            }
            SirinMode::Landed => {}
        }

        // An operator request wins over whatever detection decided this tick.
        if let Some(mode) = requested_mode {
            if mode != self.mode {
                self.enter(mode, now_ms, &mut out);
            }
        }

        out
    }

    fn enter(&mut self, mode: SirinMode, now_ms: u64, out: &mut FlightUpdate) {
        match mode {
            SirinMode::Standby => {
                self.launched_at_ms = None;
                self.max_altitude = self.altitude;
                self.apogee = None;
                self.main_altitude_reached = false;
            }
            SirinMode::Flight => {
                self.launched_at_ms.get_or_insert(now_ms);
                self.max_altitude = self.altitude;
                self.apogee = None;
                self.main_altitude_reached = false;
            }
            SirinMode::Descent => {
                self.launched_at_ms.get_or_insert(now_ms);
                self.apogee.get_or_insert(self.max_altitude);
            }
            SirinMode::Landed => {}
        }

        self.landed_anchor = None;
        self.mode = mode;
        out.new_mode = Some(mode);
    }

    fn flight_timed_out(&self, now_ms: u64) -> bool {
        self.launched_at_ms
            .is_some_and(|launched_at| now_ms.saturating_sub(launched_at) > self.params.flight_duration_ms)
    }

    fn past_apogee_lockout(&self, now_ms: u64) -> bool {
        self.launched_at_ms
            .is_some_and(|launched_at| now_ms.saturating_sub(launched_at) > self.params.apogee_lockout_ms)
    }

    fn check_main_altitude(&mut self, altitude: Meters<f64>, out: &mut FlightUpdate) {
        let Some(main_altitude) = self.params.main_deployment_altitude else {
            return;
        };

        if !self.main_altitude_reached && altitude.value < main_altitude {
            self.main_altitude_reached = true;
            out.deploy_main = true;
        }
    }

    fn check_landed(&mut self, altitude: Meters<f64>, now_ms: u64, out: &mut FlightUpdate) {
        let band = self.params.landed_altitude_band;

        match self.landed_anchor {
            Some((anchor, since_ms)) if {
                let delta = altitude.value - anchor.value;
                -band <= delta && delta <= band
            } => {
                if now_ms.saturating_sub(since_ms) >= self.params.landed_stable_ms {
                    self.enter(SirinMode::Landed, now_ms, out);
                }
            }
            _ => self.landed_anchor = Some((altitude, now_ms)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TICK_MS: u64 = 100;

    /// The Comp 2026 values from `examples/nosecone-ebay`.
    const PARAMS: FlightParams = FlightParams {
        launch_altitude: 20.0,
        launch_accel_squared: 10.0 * 10.0,
        apogee_error: 4.0,
        apogee_lockout_ms: 25_000,
        main_deployment_altitude: Some(457.2),
        landed_altitude_band: 4.0,
        landed_stable_ms: 30_000,
        flight_duration_ms: 1_000_000,
    };

    fn meters(value: f64) -> Meters<f64> {
        value.with_units()
    }

    fn entered(mode: SirinMode) -> FlightUpdate {
        FlightUpdate { new_mode: Some(mode), ..Default::default() }
    }

    /// A non-empty update, with the time and altitude it happened at.
    #[derive(Clone, Copy, Debug, PartialEq)]
    struct Event {
        now_ms: u64,
        altitude: f64,
        update: FlightUpdate,
    }

    /// Drives a detector with a barometric altitude profile at `TICK_MS` intervals.
    struct Sim {
        detector: FlightDetector,
        now_ms: u64,
    }

    impl Sim {
        fn new(params: FlightParams) -> Self {
            Self { detector: FlightDetector::new(params), now_ms: 0 }
        }

        fn tick(&mut self, altitude: Option<f64>, accel_squared: Option<f64>, requested_mode: Option<SirinMode>) -> FlightUpdate {
            self.now_ms += TICK_MS;
            self.detector.update(FlightInput {
                now_ms: self.now_ms,
                altitude: altitude.map(meters),
                accel_squared,
                requested_mode,
            })
        }

        /// One tick at `altitude` with a quiet IMU.
        fn altitude(&mut self, altitude: f64) -> FlightUpdate {
            self.tick(Some(altitude), Some(1.0), None)
        }

        /// One tick at the current altitude carrying an operator request.
        fn request(&mut self, mode: SirinMode) -> FlightUpdate {
            self.tick(Some(self.detector.altitude.value), Some(1.0), Some(mode))
        }

        /// Holds `altitude` for `duration_ms`, asserting that nothing happens.
        fn hold(&mut self, altitude: f64, duration_ms: u64) {
            for _ in 0..duration_ms / TICK_MS {
                assert_eq!(self.altitude(altitude), FlightUpdate::default(), "at t={}ms", self.now_ms);
            }
        }

        /// Ramps linearly from the current altitude to `target` over `duration_ms` and returns
        /// every non-empty update.
        fn ramp(&mut self, target: f64, duration_ms: u64) -> Vec<Event> {
            let start = self.detector.altitude.value;
            let steps = duration_ms / TICK_MS;
            let mut events = Vec::new();

            for step in 1..=steps {
                let altitude = start + (target - start) * step as f64 / steps as f64;
                let update = self.altitude(altitude);
                if update != FlightUpdate::default() {
                    events.push(Event { now_ms: self.now_ms, altitude, update });
                }
            }

            events
        }

        /// Launches at 25 m and climbs to `apogee` over `ascent_ms`, then dips 10 m over one
        /// second so that apogee is declared. Returns the apogee event.
        fn fly_to_apogee(&mut self, apogee: f64, ascent_ms: u64) -> Event {
            assert_eq!(self.altitude(25.0), entered(SirinMode::Flight));
            assert_eq!(self.ramp(apogee, ascent_ms), vec![]);

            let events = self.ramp(apogee - 10.0, 1_000);
            assert_eq!(events.len(), 1, "{events:?}");
            assert_eq!(events[0].update.new_mode, Some(SirinMode::Descent));
            assert!(events[0].update.deploy_apo);
            assert_eq!(self.apogee(), Some(apogee));
            events[0]
        }

        fn mode(&self) -> SirinMode {
            self.detector.mode()
        }

        fn apogee(&self) -> Option<f64> {
            self.detector.apogee().map(|apogee| apogee.value)
        }

        fn max_altitude(&self) -> f64 {
            self.detector.max_altitude().value
        }

        fn since_launch(&self) -> u64 {
            self.now_ms - self.detector.launched_at_ms().expect("not launched")
        }
    }

    #[test]
    fn starts_in_standby() {
        let detector = FlightDetector::new(PARAMS);
        assert_eq!(detector.mode(), SirinMode::Standby);
        assert!(detector.apogee().is_none());
        assert_eq!(detector.launched_at_ms(), None);
        assert_eq!(detector.params(), &PARAMS);
    }

    #[test]
    fn standby_ignores_noise_below_thresholds() {
        let mut sim = Sim::new(PARAMS);
        sim.hold(0.0, 1_000);
        sim.hold(19.0, 1_000);
        assert_eq!(sim.tick(Some(0.0), Some(99.0), None), FlightUpdate::default());
        assert_eq!(sim.tick(None, None, None), FlightUpdate::default());
        assert_eq!(sim.mode(), SirinMode::Standby);
        assert_eq!(sim.detector.launched_at_ms(), None);
    }

    #[test]
    fn launch_by_altitude() {
        let mut sim = Sim::new(PARAMS);
        sim.hold(0.0, 1_000);

        assert_eq!(sim.altitude(20.5), entered(SirinMode::Flight));
        assert_eq!(sim.mode(), SirinMode::Flight);
        assert_eq!(sim.detector.launched_at_ms(), Some(sim.now_ms));
        assert_eq!(sim.max_altitude(), 20.5);
        assert_eq!(sim.apogee(), None);
    }

    #[test]
    fn launch_by_acceleration() {
        let mut sim = Sim::new(PARAMS);
        sim.hold(0.0, 1_000);

        assert_eq!(sim.tick(Some(0.0), Some(11.0 * 11.0), None), entered(SirinMode::Flight));
        assert_eq!(sim.mode(), SirinMode::Flight);
    }

    #[test]
    fn launch_by_acceleration_without_barometer() {
        let mut sim = Sim::new(PARAMS);
        assert_eq!(sim.tick(None, Some(11.0 * 11.0), None), entered(SirinMode::Flight));
    }

    #[test]
    fn nominal_flight() {
        let mut sim = Sim::new(PARAMS);
        sim.hold(0.0, 2_000);

        // Boost and coast to a 3000 m apogee over 30 s.
        let events = sim.ramp(3000.0, 30_000);
        assert_eq!(events.len(), 1, "{events:?}");
        assert_eq!(events[0].update, entered(SirinMode::Flight));
        assert_eq!(sim.mode(), SirinMode::Flight);
        assert_eq!(sim.apogee(), None);

        // Fall back to the pad over 120 s (2.5 m per tick).
        let events = sim.ramp(0.0, 120_000);
        assert_eq!(events.len(), 2, "{events:?}");

        let apogee = events[0];
        assert_eq!(apogee.update, FlightUpdate { new_mode: Some(SirinMode::Descent), deploy_apo: true, deploy_main: false });
        assert!(apogee.altitude < 3000.0 - PARAMS.apogee_error, "apogee declared at {} m", apogee.altitude);
        assert!(apogee.altitude >= 3000.0 - PARAMS.apogee_error - 2.5, "apogee declared late at {} m", apogee.altitude);
        assert_eq!(sim.apogee(), Some(3000.0));

        let main = events[1];
        assert_eq!(main.update, FlightUpdate { new_mode: None, deploy_apo: false, deploy_main: true });
        assert!(main.altitude < 457.2 && main.altitude >= 457.2 - 2.5, "main deployed at {} m", main.altitude);

        // Still moving when we reach the pad: not landed yet.
        assert_eq!(sim.mode(), SirinMode::Descent);

        // Sitting on the ground (with a little noise) for the stable period lands the flight.
        let on_ground_at = sim.now_ms;
        let mut landed_at = None;
        for step in 0..(2 * PARAMS.landed_stable_ms / TICK_MS) {
            let update = sim.altitude(if step % 2 == 0 { 0.5 } else { -0.5 });
            if update != FlightUpdate::default() {
                assert_eq!(update, entered(SirinMode::Landed));
                landed_at = Some(sim.now_ms);
                break;
            }
        }
        let landed_at = landed_at.expect("never landed");
        // The anchor may have been taken a tick before the ramp ended.
        assert!(landed_at - on_ground_at >= PARAMS.landed_stable_ms - TICK_MS, "landed too early");
        assert!(landed_at - on_ground_at <= PARAMS.landed_stable_ms + TICK_MS, "landed too late");
        assert_eq!(sim.mode(), SirinMode::Landed);
        assert_eq!(sim.apogee(), Some(3000.0));

        // Nothing happens after landing, whatever the sensors say.
        sim.hold(0.0, 10_000);
        sim.hold(500.0, 10_000);
        assert_eq!(sim.tick(Some(0.0), Some(400.0), None), FlightUpdate::default());
        assert_eq!(sim.mode(), SirinMode::Landed);
    }

    #[test]
    fn apogee_is_not_declared_during_lockout() {
        let mut sim = Sim::new(PARAMS);
        assert_eq!(sim.altitude(25.0), entered(SirinMode::Flight));

        // A pressure transient during boost looks like a 50 m drop 5 s into the flight.
        assert_eq!(sim.ramp(400.0, 5_000), vec![]);
        assert_eq!(sim.altitude(360.0), FlightUpdate::default());
        assert_eq!(sim.altitude(350.0), FlightUpdate::default());
        assert!(sim.since_launch() < PARAMS.apogee_lockout_ms);
        assert_eq!(sim.mode(), SirinMode::Flight);
        assert_eq!(sim.apogee(), None);

        // The rocket keeps climbing past the transient and reaches apogee after the lockout.
        assert_eq!(sim.ramp(2000.0, 30_000), vec![]);
        let events = sim.ramp(1990.0, 1_000);
        assert_eq!(events.len(), 1, "{events:?}");
        assert_eq!(events[0].update, FlightUpdate { new_mode: Some(SirinMode::Descent), deploy_apo: true, deploy_main: false });
        assert!(sim.since_launch() > PARAMS.apogee_lockout_ms);
        assert_eq!(sim.apogee(), Some(2000.0));
    }

    #[test]
    fn early_apogee_is_declared_as_soon_as_lockout_ends() {
        let mut sim = Sim::new(PARAMS);
        assert_eq!(sim.altitude(25.0), entered(SirinMode::Flight));
        let launched_at = sim.detector.launched_at_ms().unwrap();

        // An underperforming flight: apogee at 800 m only 20 s in, then falling at 10 m/s.
        assert_eq!(sim.ramp(800.0, 20_000), vec![]);
        let events = sim.ramp(700.0, 10_000);

        assert_eq!(events.len(), 1, "{events:?}");
        let apogee = events[0];
        assert_eq!(apogee.update, FlightUpdate { new_mode: Some(SirinMode::Descent), deploy_apo: true, deploy_main: false });

        // Declared on the first tick after the lockout, with the true maximum as the apogee.
        let elapsed = apogee.now_ms - launched_at;
        assert!(elapsed > PARAMS.apogee_lockout_ms, "declared during lockout at {elapsed} ms");
        assert!(elapsed <= PARAMS.apogee_lockout_ms + TICK_MS, "declared late at {elapsed} ms");
        assert_eq!(sim.apogee(), Some(800.0));
    }

    #[test]
    fn apogee_is_corrected_if_still_climbing() {
        let mut sim = Sim::new(PARAMS);
        sim.fly_to_apogee(1000.0, 30_000);
        assert_eq!(sim.mode(), SirinMode::Descent);
        assert_eq!(sim.apogee(), Some(1000.0));

        // The dip was a transient and the rocket is still climbing. The recorded apogee follows
        // the true maximum, the mode stays Descent, and the apogee charge is not fired again.
        assert_eq!(sim.ramp(1500.0, 10_000), vec![]);
        assert_eq!(sim.mode(), SirinMode::Descent);
        assert_eq!(sim.apogee(), Some(1500.0));

        // Noise inside the hysteresis band does not move the apogee.
        assert_eq!(sim.altitude(1503.0), FlightUpdate::default());
        assert_eq!(sim.apogee(), Some(1500.0));
    }

    #[test]
    fn main_fires_once() {
        let mut sim = Sim::new(PARAMS);
        sim.fly_to_apogee(2000.0, 30_000);

        let events = sim.ramp(300.0, 60_000);
        assert_eq!(events.len(), 1, "{events:?}");
        assert_eq!(events[0].update, FlightUpdate { new_mode: None, deploy_apo: false, deploy_main: true });
        assert!(events[0].altitude < 457.2);

        // Bouncing back above and below the main altitude does not fire it again.
        assert_eq!(sim.ramp(500.0, 5_000), vec![]);
        assert_eq!(sim.ramp(300.0, 5_000), vec![]);
    }

    #[test]
    fn low_apogee_fires_both_charges_in_one_tick() {
        let mut sim = Sim::new(PARAMS);
        let apogee = sim.fly_to_apogee(300.0, 30_000);
        assert_eq!(apogee.update, FlightUpdate { new_mode: Some(SirinMode::Descent), deploy_apo: true, deploy_main: true });
    }

    #[test]
    fn main_deployment_can_be_disabled() {
        let mut sim = Sim::new(FlightParams { main_deployment_altitude: None, ..PARAMS });
        sim.fly_to_apogee(2000.0, 30_000);
        assert_eq!(sim.ramp(10.0, 60_000), vec![]);
        assert_eq!(sim.mode(), SirinMode::Descent);
    }

    #[test]
    fn flight_times_out_without_apogee() {
        let mut sim = Sim::new(FlightParams { flight_duration_ms: 60_000, ..PARAMS });
        assert_eq!(sim.altitude(25.0), entered(SirinMode::Flight));

        // A stuck barometer: the altitude never comes down, so apogee is never declared.
        assert_eq!(sim.ramp(1000.0, 59_000), vec![]);
        assert_eq!(sim.mode(), SirinMode::Flight);

        for _ in 0..20 {
            let update = sim.altitude(1000.0);
            if update != FlightUpdate::default() {
                assert_eq!(update, entered(SirinMode::Landed));
                assert!(sim.since_launch() > 60_000);
                assert!(sim.since_launch() <= 60_000 + TICK_MS);
                assert_eq!(sim.apogee(), None);
                return;
            }
        }

        panic!("flight never timed out");
    }

    #[test]
    fn descent_times_out() {
        let mut sim = Sim::new(FlightParams { flight_duration_ms: 60_000, ..PARAMS });
        sim.fly_to_apogee(2000.0, 30_000);

        // Coming down slowly enough that the altitude never settles before the timeout.
        let events = sim.ramp(1500.0, 40_000);
        assert_eq!(events.len(), 1, "{events:?}");
        assert_eq!(events[0].update, entered(SirinMode::Landed));
        assert!(events[0].now_ms - sim.detector.launched_at_ms().unwrap() > 60_000);
        assert_eq!(sim.apogee(), Some(2000.0));
    }

    #[test]
    fn timeout_also_applies_without_a_barometer() {
        let mut sim = Sim::new(FlightParams { flight_duration_ms: 10_000, ..PARAMS });
        assert_eq!(sim.tick(None, Some(200.0), None), entered(SirinMode::Flight));

        for _ in 0..200 {
            let update = sim.tick(None, None, None);
            if update != FlightUpdate::default() {
                assert_eq!(update, entered(SirinMode::Landed));
                assert!(sim.since_launch() > 10_000);
                return;
            }
        }

        panic!("flight never timed out");
    }

    #[test]
    fn landing_needs_fresh_readings() {
        let mut sim = Sim::new(PARAMS);
        sim.fly_to_apogee(2000.0, 30_000);
        sim.ramp(0.0, 60_000);
        assert_eq!(sim.mode(), SirinMode::Descent);

        // A dead barometer must not look like a stable altitude.
        for _ in 0..(2 * PARAMS.landed_stable_ms / TICK_MS) {
            assert_eq!(sim.tick(None, Some(1.0), None), FlightUpdate::default());
        }
        assert_eq!(sim.mode(), SirinMode::Descent);
    }

    #[test]
    fn landing_anchor_resets_while_moving() {
        let mut sim = Sim::new(PARAMS);
        sim.fly_to_apogee(2000.0, 30_000);
        sim.ramp(300.0, 60_000);
        assert_eq!(sim.mode(), SirinMode::Descent);

        // Descending at 5 m/s for two minutes never looks like a landing.
        assert_eq!(sim.ramp(300.0 - 5.0 * 120.0, 120_000), vec![]);
        assert_eq!(sim.mode(), SirinMode::Descent);

        // Jitter inside the band does count, once it has gone on long enough.
        let settled_at = sim.now_ms;
        let mut landed_at = None;
        for step in 0..1_000 {
            let update = sim.altitude(-300.0 + (step % 3) as f64);
            if update != FlightUpdate::default() {
                assert_eq!(update, entered(SirinMode::Landed));
                landed_at = Some(sim.now_ms);
                break;
            }
        }
        let landed_at = landed_at.expect("never landed");
        assert!(landed_at - settled_at >= PARAMS.landed_stable_ms - TICK_MS);
        assert!(landed_at - settled_at <= PARAMS.landed_stable_ms + TICK_MS);
    }

    #[test]
    fn requested_modes_walk_the_state_machine() {
        let mut sim = Sim::new(PARAMS);
        sim.hold(0.0, 1_000);

        assert_eq!(sim.request(SirinMode::Flight), entered(SirinMode::Flight));
        assert_eq!(sim.mode(), SirinMode::Flight);
        assert_eq!(sim.detector.launched_at_ms(), Some(sim.now_ms));

        // Forcing Descent never fires the apogee charge, but does record an apogee.
        assert_eq!(sim.request(SirinMode::Descent), entered(SirinMode::Descent));
        assert_eq!(sim.mode(), SirinMode::Descent);
        assert_eq!(sim.apogee(), Some(0.0));

        // Detection keeps running in the forced mode: on the pad we are below the main
        // deployment altitude, so the main charge fires on the next tick.
        assert_eq!(sim.altitude(0.0), FlightUpdate { new_mode: None, deploy_apo: false, deploy_main: true });

        assert_eq!(sim.request(SirinMode::Landed), entered(SirinMode::Landed));
        assert_eq!(sim.mode(), SirinMode::Landed);

        // Standby re-arms everything.
        assert_eq!(sim.request(SirinMode::Standby), entered(SirinMode::Standby));
        assert_eq!(sim.mode(), SirinMode::Standby);
        assert_eq!(sim.apogee(), None);
        assert_eq!(sim.detector.launched_at_ms(), None);
        sim.hold(0.0, 1_000);
        assert_eq!(sim.altitude(30.0), entered(SirinMode::Flight));
    }

    #[test]
    fn requesting_the_current_mode_does_nothing() {
        let mut sim = Sim::new(PARAMS);
        assert_eq!(sim.request(SirinMode::Standby), FlightUpdate::default());
        sim.request(SirinMode::Flight);
        assert_eq!(sim.request(SirinMode::Flight), FlightUpdate::default());
        assert_eq!(sim.mode(), SirinMode::Flight);
    }

    #[test]
    fn request_wins_over_detection_in_the_same_tick() {
        let mut sim = Sim::new(PARAMS);
        sim.hold(0.0, 1_000);

        // Launch would be detected this tick, but the operator asked for Landed.
        let update = sim.tick(Some(100.0), Some(1.0), Some(SirinMode::Landed));
        assert_eq!(update, entered(SirinMode::Landed));
        assert_eq!(sim.mode(), SirinMode::Landed);
    }

    #[test]
    fn forced_flight_from_descent_starts_a_fresh_ascent() {
        let mut sim = Sim::new(PARAMS);
        sim.fly_to_apogee(2000.0, 30_000);
        let launched_at = sim.detector.launched_at_ms();

        assert_eq!(sim.request(SirinMode::Flight), entered(SirinMode::Flight));
        assert_eq!(sim.apogee(), None);
        assert_eq!(sim.max_altitude(), 1990.0);
        // The original launch time is kept so the flight timeout still counts from launch.
        assert_eq!(sim.detector.launched_at_ms(), launched_at);
    }
}
