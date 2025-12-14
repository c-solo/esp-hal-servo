//! Servo motor configuration.
//!
//! This module provides the [`ServoConfig`] struct for configuring servo motors,
//! including frequency, pulse width range, rotation speed, and pre-configured
//! settings for common servo models like SG90 and MG995.

use crate::utils;
use core::ops::Range;
use esp_hal::{
    ledc::{
        LSGlobalClkSource, Ledc,
        timer::{self, Timer, TimerHW, TimerIFace, TimerSpeed, config::Duty},
    },
    time::Rate,
};

/// Configuration for a servo motor.
///
/// This struct contains all the parameters needed to configure a servo motor,
/// including frequency, pulse width range, and rotation speed.
#[derive(Debug, Clone)]
pub struct ServoConfig {
    /// Max angle that servo can be turned, mostly 180, 360.
    pub max_angle: f32,
    /// What frequency expect servo (ex. 50Hz for SG90).
    pub frequency: Rate,
    /// What pulse width in nanos servo supports (ex. 500000-2400000ns for SG90).
    pub pulse_width_ns: Range<u32>,
    /// PWM resolution in bits. Higher bits means more precise control.
    pub duty: Duty,
    /// Servo rotation speed in degrees per second.
    /// Used to calculate delay time based on rotation angle.
    pub speed_deg_per_sec: f32,
}

impl ServoConfig {
    /// Default servo configuration with 50Hz frequency and
    /// pulse width range of 500000-2500000 ns (0.5-2.5ms).
    pub fn default_servo(duty: Duty, max_angle: f32, speed_deg_per_sec: f32) -> Self {
        ServoConfig {
            max_angle,
            frequency: Rate::from_hz(50),
            // Standard servo pulse width range: 500-2500 us
            pulse_width_ns: 500_000..2_500_000,
            duty,
            speed_deg_per_sec,
        }
    }

    /// Config for [SG90](https://www.friendlywire.com/projects/ne555-servo-safe/SG90-datasheet.pdf).
    /// Can be used for SG90s as well.
    pub fn sg90(duty: Duty) -> Self {
        Self {
            pulse_width_ns: 500_000..2_400_000,
            ..Self::default_servo(duty, 180.0, 60.0)
        }
    }

    /// Config for [MG995](https://www.electronicoscaldas.com/datasheet/MG995_Tower-Pro.pdf).
    /// High-torque servo motor with metal gears.
    /// Can be used for MG996, MG996R as well.
    pub fn mg995(duty: Duty) -> Self {
        Self::default_servo(duty, 180.0, 100.0)
    }

    /// Helper function to configure a timer with this servo's configuration.
    pub fn configure_timer<'a, S: TimerSpeed>(
        &self,
        ledc: &mut Ledc<'a>,
        timer_num: timer::Number,
        clock_source: S::ClockSourceType,
    ) -> Result<Timer<'a, S>, timer::Error>
    where
        Timer<'a, S>: TimerHW<S> + TimerIFace<S>,
    {
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
        let mut timer = ledc.timer::<S>(timer_num);
        timer.configure(timer::config::Config {
            duty: self.duty,
            clock_source,
            frequency: self.frequency,
        })?;
        Ok(timer)
    }

    /// Calculates duty range in absolute values for this servo configuration.
    /// Returns absolute duty values (0..max_duty), not percentages.
    pub fn calc_duty_range(&self, max_duty: f32) -> Range<f32> {
        utils::calc_duty_range(
            self.pulse_width_ns.clone(),
            self.frequency.as_hz() as f32,
            max_duty,
        )
    }

    /// Transforms absolute duty value to angle in degrees.
    /// Returns angle in degrees (0.0..max_angle).
    pub fn duty_to_angle(&self, duty: f32, max_angle: f32, duty_range: &Range<f32>) -> f32 {
        utils::duty_to_angle(duty, max_angle, duty_range)
    }

    /// Transforms angle in degrees to absolute duty value.
    pub fn angle_to_duty(&self, angle: f32, duty_range: &Range<f32>) -> f32 {
        utils::angle_to_duty(angle, self.max_angle, duty_range)
    }
}
