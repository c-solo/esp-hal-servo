# Examples

Examples of using the `esp-hal-servo` library to control servo motors on ESP32.

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

### Running

```bash
RUST_LOG=info cargo run --release --example simple_sweep
```
```
INFO - Starting servo sweep example
INFO - sweep servo: duty_range=102..491, center_duty=296
INFO - Initial angle: 90.00°
INFO - Sweeping clockwise...
INFO - Reached max position: 180.00°
INFO - Sweeping counter-clockwise...
INFO - Reached min position: 0.00°
```