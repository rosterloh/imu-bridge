# Repository Guidelines

## Project Structure & Module Organization
`src/lib.rs` exposes the reusable crate modules: `board`, `config`, `network`, `sensor`, `domain`, and `error`. The firmware entrypoint lives in `src/bin/main.rs`. Sensor integrations are grouped under `src/sensor/` (`bmi088.rs`, `imu.rs`), and message/domain types live under `src/domain/`. Build-time setup is in `build.rs`, while target and runner defaults are in `.cargo/config.toml`.

## Build, Test, and Development Commands
Use the ESP toolchain from `rust-toolchain.toml` (`channel = "esp"`).

- `cargo check`: fast validation for the default `xtensa-esp32s3-none-elf` target.
- `cargo build`: compile the firmware binary.
- `cargo run`: build and flash/run through the configured `probe-rs` runner for ESP32-S3.
- `cargo clippy --all-targets -- -D warnings`: enforce lint cleanliness.
- `cargo fmt --check`: verify formatting before opening a PR.

## Coding Style & Naming Conventions
Follow standard Rust formatting with 4-space indentation and `snake_case` for modules, files, functions, and variables. Use `PascalCase` for types and enums, matching existing names such as `BoardContext` and `SensorReading`. Keep modules focused by hardware or domain responsibility rather than adding large mixed files. Clippy is part of the style contract here; note the explicit `#![deny(...)]` rules in `src/bin/main.rs` and the stack-frame threshold in `.clippy.toml`.

## Testing Guidelines
There is no dedicated `tests/` directory yet, so add unit tests next to host-testable logic and use integration tests only when they do not depend on board peripherals. Run `cargo test` for pure logic and formatting helpers, and use `cargo check`/`cargo clippy` as the minimum gate for hardware-facing changes. Name tests after behavior, for example `formats_sensor_reading_payload`.

## Commit & Pull Request Guidelines
The current history uses short, imperative commit subjects (`Basic connection implemented`). Keep commits focused and write subjects in that style. PRs should include a brief summary, affected modules, required hardware/runtime setup, and the commands you ran. Include logs or screenshots only when they clarify flashing, networking, or sensor output behavior.

## Security & Configuration Tips
`.cargo/config.toml` currently holds SSID, password, and router endpoint values for local development. Treat these as environment-specific, avoid committing real secrets, and document any replacement values in the PR when configuration changes affect reviewers.
