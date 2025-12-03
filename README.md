# esp-hal-servo

A library for controlling servo motors using ESP32 LEDC (LED Control) peripheral.
No `esp-idf` and `std` dependencies (pure `esp-hal`).

## Features

- Control servo motors using PWM via LEDC
- Support for custom servo configurations (SG90/SG90S preconfigured)
- Angle calculation and position tracking
- Support for all ESP32 variants (ESP32, ESP32C2, ESP32C3, ESP32C6, ESP32H2, ESP32S2, ESP32S3)

## Features

This library supports all ESP32 variants through optional features. You must enable at least one chip feature:

- `esp32` - Original ESP32
- `esp32c2` - ESP32-C2
- `esp32c3` - ESP32-C3
- `esp32c6` - ESP32-C6
- `esp32h2` - ESP32-H2
- `esp32s2` - ESP32-S2
- `esp32s3` - ESP32-S3

## Usage

Add the library to your `Cargo.toml` with the appropriate chip feature:

```toml
[dependencies]
esp-hal-servo = { version = "0.1", features = ["esp32c3"] }
```

## Examples

[examples/README.md](examples/README.md) for more information about available examples.