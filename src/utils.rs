//! Utility functions for servo calculations.
//! These functions are independent of `ServoConfig` and can be tested in isolation.

use core::ops::Range;

const NANOS_IS_SEC: f64 = 1_000_000_000.0;

/// Maps pulse width in nanoseconds to absolute duty value as u32 (not percentage).
///
/// # Arguments
///
/// * `pulse_ns` - Pulse width in nanoseconds
/// * `frequency_hz` - PWM frequency in Hz
/// * `max_duty` - Maximum duty value (e.g., 4095 for 12-bit resolution)
///
/// # Returns
///
/// Absolute duty value (0..max_duty)
pub fn pulse_to_duty(pulse_ns: u32, frequency_hz: f64, max_duty: u32) -> u32 {
    let duty_f = pulse_ns as f64 * frequency_hz * max_duty as f64 / NANOS_IS_SEC;
    (duty_f + 0.5) as u32
}

/// Transforms absolute duty value to angle in degrees.
///
/// # Arguments
///
/// * `duty` - Current duty value (0..max_duty)
/// * `max_duty` - Maximum duty value (e.g., 4095 for 12-bit resolution)
/// * `frequency_hz` - PWM frequency in Hz
/// * `pulse_width_ns` - Pulse width range in nanoseconds (min..max)
/// * `max_angle` - Maximum angle that servo can be turned (e.g., 180.0 or 360.0)
///
/// # Returns
///
/// Angle in degrees (0.0..max_angle)
pub fn calculate_angle(
    duty: u32,
    max_duty: u32,
    frequency_hz: f64,
    pulse_width_ns: Range<u32>,
    max_angle: f64,
) -> f64 {
    // Convert duty to pulse width in nanoseconds
    let pulse_ns = duty as f64 * NANOS_IS_SEC / frequency_hz / max_duty as f64;

    // Map pulse width to angle
    let pulse_range = (pulse_width_ns.end - pulse_width_ns.start) as f64;
    // Prevent division by zero
    if pulse_range <= 0.0 {
        return 0.0;
    }

    // Normalize pulse width to [0, 1] range within servo's pulse width range
    let normalized = ((pulse_ns - pulse_width_ns.start as f64) / pulse_range).clamp(0.0, 1.0);
    normalized * max_angle
}

/// Calculates duty range in absolute values for given servo configuration.
///
/// # Arguments
///
/// * `pulse_width_ns` - Pulse width range in nanoseconds (min..max)
/// * `frequency_hz` - PWM frequency in Hz
/// * `max_duty` - Maximum duty value (e.g., 4095 for 12-bit resolution)
///
/// # Returns
///
/// Absolute duty values range (min_duty..max_duty), not percentages.
pub fn calc_duty_range(pulse_width_ns: Range<u32>, frequency_hz: f64, max_duty: u32) -> Range<u32> {
    let min_pulse = pulse_width_ns.start;
    let max_pulse = pulse_width_ns.end;

    // Calculate absolute duty values directly from pulse width
    let min_duty = pulse_to_duty(min_pulse, frequency_hz, max_duty);
    let max_duty_val = pulse_to_duty(max_pulse, frequency_hz, max_duty);

    // Ensure values are within valid range
    let min_duty = min_duty.min(max_duty);
    let max_duty_val = max_duty_val.min(max_duty);

    // Ensure valid range (min < max)
    let min_duty = min_duty.min(max_duty_val.saturating_sub(1));
    let max_duty_val = max_duty_val.max(min_duty + 1);

    min_duty..max_duty_val
}
