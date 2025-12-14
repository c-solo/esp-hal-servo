# Examples

Examples of using the `esp-hal-servo` library to control servo motors on ESP32.

All examples are runnable only for RISC-V chips (C2, C3, C6, H2) with target: `riscv32imc-unknown-none-elf`.
S2, S3 required different target (`xtensa-esp32s3-espidf`) and stable Rust does not support it yet 
(required additional Espressif toolchain).

## simple_sweep

Smooth movement between minimum and maximum positions.

### Hardware

- **Microcontroller**: ESP32-C3 SuperMini (or any ESP32)
- **Servo motor**: SG90 (or compatible)
- **Power supply**: External 5V power supply for servo (recommended)

```
ESP32-C3          SG90 Servo
--------          -----------
GPIO 2    ------> Signal (orange/yellow wire)
GND       ------> GND (brown/black wire)
          ------> VCC (red wire) -> external 5V power supply (Use an external power supply for the servo)
```

### Building and Running

```bash
RUST_LOG=info cargo run --release -p examples --bin simple_sweep --features esp32c3
```

**Note**: Replace `esp32c3` with the appropriate chip feature for your target (e.g., `esp32c2`, `esp32c6`, `esp32h2`).

Expected output:
```
INFO - Starting servo sweep example
INFO - sweep servo: duty_range=102..491, center_duty=296
INFO - Initial angle: 90.00°
INFO - Sweeping clockwise...
INFO - Reached max position: 180.00°
INFO - Sweeping counter-clockwise...
INFO - Reached min position: 0.00°
```

## async_angle_sweep

Async servo control using only angle-based API. Demonstrates non-blocking servo movement
using `AsyncServo` wrapper with automatic delay calculation based on servo speed.

### Hardware

Same as `simple_sweep` example.

### Building and Running

**Note**: This example requires the `async` feature and uses async/await.
The example uses `esp-rtos` to initialize the `embassy-time` driver, and `embassy_time::Delay`
(which implements `DelayNs` trait) is used directly. Any `DelayNs` implementation can be used.

```bash
RUST_LOG=info cargo run --release -p examples --bin async_angle_sweep --features esp32c3,async
```
Expected output:
```
INFO - Starting async servo angle sweep example
INFO - async_sweep servo: duty_range=102.375..491.4, center_duty=296.8875
INFO - Initial angle: 90.00°
INFO - Moving to 0°...
INFO - Reached 0°
INFO - Moving to 180°...
INFO - Reached 180°
```
