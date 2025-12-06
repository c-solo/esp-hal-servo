//! Utility functions for servo calculations.
//! These functions are independent of `ServoConfig` and can be tested in isolation.

use core::ops::Range;

const NANOS_IS_SEC: f32 = 1_000_000_000.0;
pub const EPSILON: f32 = 0.5;

/// Checks if two duty values are approximately equal.
pub fn approx_eq(left: f32, right: f32) -> bool {
    (left - right).abs() < EPSILON
}

/// Transforms absolute duty value to angle in degrees.
pub fn duty_to_angle(
    duty: f32,
    max_angle: f32,
    duty_range: &Range<f32>,
) -> f32 {
    let clamped_duty = duty.clamp(duty_range.start, duty_range.end);
    ((clamped_duty - duty_range.start) / (duty_range.end - duty_range.start)) * max_angle
}

/// Transforms angle in degrees to absolute duty value.
pub fn angle_to_duty(angle: f32, max_angle: f32, duty_range: &Range<f32>) -> f32 {
    let clamped_angle = angle.clamp(0.0, max_angle);
    duty_range.start + (clamped_angle / max_angle) * (duty_range.end - duty_range.start)
}

/// Calculates duty range in absolute values for given servo configuration.
pub fn calc_duty_range(pulse_width_ns: Range<u32>, frequency_hz: f32, max_duty: f32) -> Range<f32> {
    pub fn pulse_to_duty(pulse_ns: u32, frequency_hz: f32, max_duty: f32) -> f32 {
        pulse_ns as f32 * frequency_hz * max_duty / NANOS_IS_SEC
    }

    let min_pulse = pulse_width_ns.start;
    let max_pulse = pulse_width_ns.end;

    // Calculate absolute duty values directly from pulse width
    let min_duty = pulse_to_duty(min_pulse, frequency_hz, max_duty);
    let max_duty_val = pulse_to_duty(max_pulse, frequency_hz, max_duty);

    // Ensure values are within valid range
    let mut min_duty = min_duty.min(max_duty);
    let mut max_duty_val = max_duty_val.min(max_duty);

    // Ensure valid range (min < max)
    // If min >= max, adjust them to ensure min < max
    if min_duty >= max_duty_val {
        let mid = (min_duty + max_duty_val) / 2.0;
        min_duty = mid.min(max_duty - 1.0);
        max_duty_val = (min_duty + 1.0).min(max_duty);
    }

    min_duty..max_duty_val
}
