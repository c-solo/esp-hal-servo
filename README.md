# esp-hal-servo

[![Crates.io](https://img.shields.io/crates/v/esp-hal-servo.svg)](https://crates.io/crates/esp-hal-servo)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Documentation](https://docs.rs/esp-hal-servo/badge.svg)](https://docs.rs/esp-hal-servo)

A library for controlling servo motors using ESP32 LEDC (LED Control) peripheral.
No `esp-idf` and `std` dependencies (pure `esp-hal`).

## Features

- Control servo motors using PWM via LEDC
- Support for custom servo configurations (SG90/SG90S preconfigured)
- Angle calculation and position tracking
- Support for all ESP32 variants (ESP32, ESP32C2, ESP32C3, ESP32C6, ESP32H2, ESP32S2, ESP32S3)
- Optional async support via Embassy runtime (enable `embassy` feature)

### Chip Features

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
esp-hal-servo = { version = "0.3", features = ["esp32c3"] }
```

To enable async support with Embassy, add the `embassy` feature:

```toml
[dependencies]
esp-hal-servo = { version = "0.3", features = ["esp32c3", "embassy"] }
embassy-time = "0.3"
```

## API Overview

This library provides two approaches for controlling servo motors:

### 1. Direct Angle Control

Simply specify the desired angle and the servo will move to that position:

```rust
// Set servo to a specific angle (e.g., 42 degrees)
servo.set_angle(42.0);
```

### 2. Step-by-Step Control with Direction

Control the servo incrementally by setting direction and making steps:

```rust
// Set direction to clockwise
servo.set_dir(Dir::CW);
// Make a step of 10 duty units
servo.step(10.0)?;
// Or make a step as a percentage of the total range
servo.step_pct(5)?; // 5% of the range
```

Use **direct angle control** when you need to position the servo at a specific angle.  
Use **step-by-step control** when you need smooth, incremental movement or continuous rotation.

### 3. Async Control with Embassy (Optional)

Enable `embassy` feature for using `AsyncServo` (allows non-blocking servo control).
Delay is automatically calculated based on servo speed and rotation angle.

```rust
use esp_hal_servo::{Servo, ServoConfig, async_servo::AsyncServo};

// Create servo with speed of 60 degrees per second (typical for SG90)
// Speed is configured in ServoConfig (e.g., ServoConfig::sg90() sets it to 60.0)
let servo = Servo::new("servo", config, &mut ledc, &mut timer, channel_num, pin)?;
let mut async_servo = AsyncServo::new(servo);

// Set angle asynchronously (delay is automatically calculated)
async_servo.set_angle(90.0).await;

// Step asynchronously (delay is automatically calculated)
async_servo.set_dir(Dir::CW);
async_servo.step_async(10.0).await?;
```
## Examples

[examples/README.md](examples/README.md) for more information about available examples.