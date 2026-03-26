#probe-rs run --chip STM32H743VITx $1 | tee >(cat >&2) | uv run live-view.py
cargo run | uv run live-view-magnet.py