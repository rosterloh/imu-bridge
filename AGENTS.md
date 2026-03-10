# Repository Guidelines

## MANDATORY: Use td for Task Management

Run td usage --new-session at conversation start (or after /clear). This tells you what to work on next.

Sessions are automatic (based on terminal/agent context). Optional:
- td session "name" to label the current session
- td session --new to force a new session in the same context

Use td usage -q after first read.

## Project Structure & Module Organization
`src/lib.rs` exposes the reusable crate modules: `app`, `board`, `config`, `network`, `sensor`, `domain`, `telemetry`, and `error`. Keep `src/bin/main.rs` as the composition root only: hardware init, task spawning, and watchdog setup. Runtime orchestration lives in `src/app/`, sensor backends live in `src/sensor/`, pure data types live in `src/domain/`, and payload encoders live in `src/telemetry/`. The host-side Zenoh CSV logger lives in `tools/imu-logger/`. Build-time setup is in `build.rs`, while target and runner defaults are in `.cargo/config.toml`.

## Build, Test, and Development Commands
Use the ESP toolchain from `rust-toolchain.toml` (`channel = "esp"`).

- `cargo check`: fast validation for the default `xtensa-esp32s3-none-elf` target.
- `cargo build`: compile the firmware binary.
- `cargo run`: build and flash/run through the configured `probe-rs` runner for ESP32-S3.
- `cargo clippy --all-targets -- -D warnings`: enforce lint cleanliness.
- `cargo fmt --check`: verify formatting before opening a PR.
- `cargo +stable run --manifest-path tools/imu-logger/Cargo.toml --target x86_64-unknown-linux-gnu -- --help`: validate the host logger without using the ESP toolchain.

## Coding Style & Naming Conventions
Follow standard Rust formatting with 4-space indentation and `snake_case` for modules, files, functions, and variables. Use `PascalCase` for types and enums, matching existing names such as `BoardContext`, `ConnectionState`, and `SensorReading`. Keep domain models free of transport formatting; new wire encoders belong under `src/telemetry/`. Keep `main.rs` thin and prefer adding behavior in `app`, `network`, or `sensor` modules instead of growing another mixed control file. Clippy is part of the style contract here; note the explicit `#![deny(...)]` rules in `src/bin/main.rs` and the stack-frame threshold in `.clippy.toml`.

## Testing Guidelines
There is no dedicated `tests/` directory yet, so add unit tests next to host-testable logic and use integration tests only when they do not depend on board peripherals. Prefer unit tests for `domain` and `telemetry` helpers, and use `cargo check`/`cargo clippy` as the minimum gate for hardware-facing changes. Run `cargo test` for pure logic and formatting code, and name tests after behavior, for example `encodes_sensor_reading_text_payload`.

## Commit & Pull Request Guidelines
The current history uses short, imperative commit subjects (`Basic connection implemented`). Keep commits focused and write subjects in that style. PRs should include a brief summary, affected modules, required hardware/runtime setup, and the commands you ran. Include logs or screenshots only when they clarify flashing, networking, or sensor output behavior.

## Security & Configuration Tips
`.cargo/config.toml` currently holds SSID, password, and router endpoint values for local development. Treat these as environment-specific, avoid committing real secrets, and document any replacement values in the PR when configuration changes affect reviewers.
