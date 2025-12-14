//! Servo motor driver implementation.
//!
//! This module provides the [`Servo`] struct for controlling servo motors
//! and the [`Dir`] enum for specifying rotation direction.

use crate::{servo_config::ServoConfig, utils};
use core::{
    marker::PhantomData,
    ops::{Neg, Range},
};
use esp_hal::{
    gpio::DriveMode,
    ledc::{
        Ledc,
        channel::{self, Channel, ChannelHW, ChannelIFace},
        timer::{Timer, TimerHW, TimerIFace, TimerSpeed},
    },
};
use log::{info, trace};

/// Direction of servo rotation.
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum Dir {
    /// Clockwise, increases angle.
    CW,
    /// Counter-clockwise, decreases angle.
    CCW,
}

/// Servo motor driver instance.
///
/// This struct represents a servo motor connected to an ESP32 LEDC channel.
/// It provides methods for controlling the servo position using either angle-based
/// or step-based control.
pub struct Servo<'a, S: TimerSpeed> {
    name: &'static str,
    channel: Channel<'a, S>,
    /// Valid duty cycle range in absolute values (e.g., 102..491 for SG90 with 12-bit).
    /// This corresponds to the pulse width range of the servo.
    pub duty_range: Range<f32>,
    config: ServoConfig,
    /// Cached max duty value for further calculations.
    max_duty: f32,
    /// Current direction. Clockwise or counter-clockwise.
    direction: Dir,
    /// Current duty in absolute value (0..max_duty).
    current_duty: f32,
    _p: PhantomData<&'a mut ()>,
}

impl<'d, S: TimerSpeed> Servo<'d, S> {
    /// Creates new servo driver instance for LEDC channel.
    ///
    /// # Arguments
    ///
    /// * `name` - Name identifier for the servo (for logging)
    /// * `config` - Servo configuration
    /// * `ledc` - LEDC peripheral instance
    /// * `timer` - Configured timer instance (use `ServoConfig::configure_timer` to create it)
    /// * `channel_num` - Channel number (e.g., `channel::Number::Channel0`)
    /// * `pin` - GPIO pin to use for PWM output
    pub fn new<'a>(
        name: &'static str,
        config: ServoConfig,
        ledc: &mut Ledc<'a>,
        timer: &'a Timer<'a, S>,
        channel_num: channel::Number,
        pin: impl esp_hal::gpio::OutputPin + 'a,
    ) -> Result<Servo<'a, S>, channel::Error>
    where
        Timer<'a, S>: TimerHW<S> + TimerIFace<S>,
    {
        // Calculate max duty before configuring channel
        let max_duty = match timer.duty() {
            Some(duty) => (1u32 << duty as u32) - 1,
            None => 4095, // Default to 12-bit if not configured
        } as f32;

        // Calculate duty range in absolute values
        let duty_range = config.calc_duty_range(max_duty);

        let mut channel = ledc.channel(channel_num, pin);
        channel.configure(channel::config::Config {
            timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })?;

        let center_duty = duty_range.start + (duty_range.end - duty_range.start) / 2.0;
        channel.set_duty_hw(center_duty as u32);

        info!(
            "{name} servo: duty_range={duty_range:?}, center_duty={center_duty}",
            name = name,
            duty_range = duty_range,
            center_duty = center_duty,
        );

        Ok(Servo::<'a> {
            name,
            channel,
            duty_range,
            config,
            direction: Dir::CW,
            current_duty: center_duty,
            max_duty,
            _p: PhantomData,
        })
    }

    /// Sets servo to specified angle in degrees.
    /// Note: turn to angle takes some time depending on servo speed.
    /// Use [`calc_delay_ms()`](Self::calc_delay_ms) to get delay time based on angle and speed.
    /// Returns true if angle was changed, false if already at that position.
    pub fn set_angle(&mut self, angle: f32) -> bool {
        let new_duty = self.config.angle_to_duty(angle, &self.duty_range);

        let delta = new_duty - self.current_duty;
        if delta > utils::EPSILON {
            self.direction = Dir::CW;
        } else if delta < utils::EPSILON.neg() {
            self.direction = Dir::CCW;
        } else {
            return false;
        }

        self.channel.set_duty_hw(new_duty as u32);
        self.current_duty = new_duty;
        true
    }

    /// Returns current angle value in degrees.
    pub fn get_angle(&self) -> f32 {
        self.config
            .duty_to_angle(self.current_duty, self.config.max_angle, &self.duty_range)
    }

    /// Set servo to move new direction.
    /// Returns old direction if direction was actually changes.
    pub fn set_dir(&mut self, dir: Dir) -> Option<Dir> {
        if self.direction != dir {
            let old = self.direction;
            self.direction = dir;
            Some(old)
        } else {
            None
        }
    }

    /// Returns current direction value.
    pub fn get_dir(&self) -> Dir {
        self.direction
    }

    /// Makes step in percentage of total range.
    /// Returns false if servo reaches min or max position.
    /// See also [`step()`](Self::step) for absolute duty-based stepping.
    /// Note: Step takes some time depending on servo speed.
    pub fn step_pct(&mut self, step_pct: u8) -> Result<bool, channel::Error> {
        let step = (step_pct as f32 / 100.0) * self.duty_range();
        self.step(step)
    }

    /// Makes step in absolute duty units should be lesser than [`duty_range()`](Self::duty_range).
    /// Return false if servo reaches min or max position.
    /// See also [`step_pct()`](Self::step_pct) for percentage-based stepping.
    /// Note: Step takes some time depending on servo speed.
    pub fn step(&mut self, step_size: f32) -> Result<bool, channel::Error> {
        let new_duty = self.calc_duty(step_size);

        // Compare with epsilon to avoid floating point precision issues
        if utils::approx_eq(new_duty, self.current_duty) {
            return Ok(false);
        }

        // hardware method has better resolution
        self.channel.set_duty_hw(new_duty as u32);
        self.current_duty = new_duty;
        trace!(
            "{} servo step({}) to duty={}/{}",
            &self.name, step_size, new_duty, self.max_duty
        );
        Ok(true)
    }

    /// Returns the size of the duty range (difference between max and min duty values).
    /// This is not the number of steps, but the range size in duty units.
    pub fn duty_range(&self) -> f32 {
        self.duty_range.end - self.duty_range.start
    }

    /// Returns the servo rotation speed in degrees per second.
    pub fn speed(&self) -> f32 {
        self.config.speed_deg_per_sec
    }

    /// Sets the servo rotation speed in degrees per second.
    /// True is new speed is valid and set, false otherwise.
    pub fn set_speed(&mut self, speed_deg_per_sec: f32) -> bool {
        if speed_deg_per_sec <= 0.0 {
            false
        } else {
            self.config.speed_deg_per_sec = speed_deg_per_sec;
            true
        }
    }

    /// Util method for calculating delay time in milliseconds based on rotation angle and servo speed.
    pub fn calc_delay_ms(&self, angle_deg: f32) -> Option<u32> {
        if angle_deg > self.config.max_angle {
            return None;
        }

        let delay_sec = angle_deg / self.config.speed_deg_per_sec;
        Some((delay_sec * 1000.0) as u32)
    }

    /// Calculates new duty based on current direction and step size.
    /// Returns clamped duty value within valid range.
    fn calc_duty(&self, step: f32) -> f32 {
        let new_duty = match self.direction {
            Dir::CW => {
                // Move clockwise (increase duty)
                self.current_duty + step
            }
            Dir::CCW => {
                // Move counter-clockwise (decrease duty)
                self.current_duty - step
            }
        };

        let min_duty = self.duty_range.start;
        let max_duty = self.duty_range.end - utils::EPSILON;
        new_duty.clamp(min_duty, max_duty)
    }
}
