//! A library for controlling servo motors using LEDC from [`esp-hal`](https://docs.rs/esp-hal/1.0.0/esp_hal/).
//!
//! This library provides two approaches for controlling servo motors:
//!
//! ## 1. Direct Angle Control
//! Simply specify the desired angle from the servo's range and wait for it to reach the position.
//! This is the simplest approach when you know the exact angle you want.
//!
//! ```ignore
//! # use esp_hal_servo::*;
//! // servo is created elsewhere
//! // Set servo to 42 degrees and wait for it to reach the position
//! servo.set_angle(42.0);
//! ```
//!
//! ## 2. Step-by-Step Control with Direction
//! Specify the direction of movement and make a step. This approach gives you fine-grained
//! control over the servo movement, allowing you to move it incrementally.
//!
//! ```ignore
//! # use esp_hal_servo::*;
//! // servo is created elsewhere
//! // Set direction to clockwise
//! servo.set_dir(Dir::CW);
//! // Make a step of 10 duty units
//! servo.step(10.0)?;
//! // Or make a step as a percentage of the total range
//! servo.step_pct(5)?; // 5% of the range
//! ```
//!
//! ## 3. Async Control with Embassy (Optional)
//! When the `embassy` feature is enabled, you can use async versions of the control methods
//! with the [`AsyncServo`](async_servo::AsyncServo) wrapper. This allows non-blocking servo
//! control in async contexts. Delay is automatically calculated based on servo speed and rotation angle.
//!
//! ```ignore
//! # #[cfg(feature = "embassy")]
//! # async fn example() {
//! # use esp_hal_servo::{Servo, ServoConfig, async_servo::AsyncServo};
//! // Create servo with speed of 60 degrees per second (typical for SG90)
//! // servo is created elsewhere
//! let mut async_servo = AsyncServo::new(servo);
//!
//! // Set angle asynchronously (delay is automatically calculated)
//! async_servo.set_angle_async(90.0).await;
//!
//! // Step asynchronously (delay is automatically calculated)
//! async_servo.set_dir(Dir::CW);
//! async_servo.step_async(10.0).await?;
//! # }
//! ```

#![no_std]

pub mod utils;

#[cfg(feature = "embassy")]
pub mod async_servo;

use core::{
    marker::PhantomData,
    ops::{Neg, Range},
};
use esp_hal::{
    gpio::DriveMode,
    ledc::{
        LSGlobalClkSource, Ledc,
        channel::{self, Channel, ChannelHW, ChannelIFace},
        timer::{self, Timer, TimerHW, TimerIFace, TimerSpeed, config::Duty},
    },
    time::Rate,
};
use log::{info, trace};

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
        Timer<'a, S>: TimerHW<S>,
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
    pub fn duty_to_angle(&self, duty: f32, max_duty: f32, duty_range: &Range<f32>) -> f32 {
        utils::duty_to_angle(duty, max_duty, duty_range)
    }

    /// Transforms angle in degrees to absolute duty value.
    pub fn angle_to_duty(&self, angle: f32, duty_range: &Range<f32>) -> f32 {
        utils::angle_to_duty(angle, self.max_angle, duty_range)
    }
}

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
        Timer<'a, S>: TimerHW<S>,
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
            .duty_to_angle(self.current_duty, self.max_duty, &self.duty_range)
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
    pub fn calc_delay_ms(&self, angle_deg: f32) -> Option<u64> {
        if angle_deg > self.config.max_angle {
            return None;
        }

        let delay_sec = angle_deg / self.config.speed_deg_per_sec;
        Some((delay_sec * 1000.0) as u64)
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

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum Dir {
    /// Clockwise, increases angle.
    CW,
    /// Counter-clockwise, decreases angle.
    CCW,
}
