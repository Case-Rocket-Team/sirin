# Sirin Rocket Flight Computer
Sirin is an in-progress flight computer for high-powered rocketry written in bare metal Rust (and some C), developed by students on Case Rocket Team (at Case Western Reserve University). This code runs on a custom PCB featuring an STM32H753, the KiCAD for the PCB is [here](https://github.com/Nautki/sirin-s-kicad) and the companion CLI is [here](https://github.com/nautki/sirin-cli).

This is the main Cargo workspace for the project. Each peripheral that we need to interact with has its own crate. `sirin` is the main library that ties everything together; `examples` contains the binaries, with `examples/main` being the most fully featured. We want to be able to specialize Sirins for different tasks (e.g. position finding, data collection, firing charge wells, etc.), hence the modular design with multiple binaries.

## Features

Currently implemented:
- [X] IMU, barometer
- [X] Radio live telemetry
- [X] USB communication for CLI
- [X] Firing of events and charge wells

Currently being worked on:
- [ ] GPS
- [ ] ESKF Kalman filter for sensor fusion (check out `eskf` branch)

Planned:
- [ ] React Dashboard

## Flight phases
The flight computer moves through `Standby -> Flight -> Descent -> Landed` (`SirinMode`). The detection logic lives in `sirin-shared/src/flight.rs` (`FlightDetector`) and is pure, so the flight binaries in `examples/` only feed it sensor readings and act on what it returns (fire charges, LED, flash logging).

## Testing without hardware
These following crates have unit tests that work without hardware:
- w25qx
- sirin-shared (flight phase detection, mode wire format)

```sh
cargo test --target <your local target>
```
For example, on my computer I would do
```sh
cargo test --target x86_64-unknown-linux-gnu
```