//! This is a small lib for controlling servo using LEDC from [`esp-hal`](https://docs.rs/esp-hal/1.0.0/esp_hal/).

#![no_std]

use core::{marker::PhantomData, ops::Range};

use esp_hal::ledc::timer::config::Duty;
use esp_hal::ledc::timer::{TimerHW, TimerSpeed};
use esp_hal::{
    gpio::DriveMode,
    ledc::{
        LSGlobalClkSource, Ledc,
        channel::{self, Channel, ChannelHW, ChannelIFace},
        timer::{self, Timer, TimerIFace},
    },
    time::Rate,
};
use log::{info, trace};

#[derive(Debug, Clone)]
pub struct ServoConfig {
    /// Max angle that servo can be turned, mostly 180, 360.
    pub max_angle: f64,
    /// What frequency expect servo (ex. 50Hz for SG90).
    pub frequency: Rate,
    /// What pulse width in nanos servo supports (ex. 500000-2400000ns for SG90).
    pub pulse_width_ns: Range<u32>,
    /// PWM resolution in bits. Higher bits means more precise control.
    pub duty: Duty,
}

impl ServoConfig {
    /// Config for [SG90](https://www.friendlywire.com/projects/ne555-servo-safe/SG90-datasheet.pdf).
    pub fn sg90(duty: Duty) -> Self {
        // SG90 pulse width range: 500-2400 us (500000-2400000 ns)
        let pulse_width_ns = 500_000..2_400_000;
        let max_angle = 180.0;
        ServoConfig {
            max_angle,
            frequency: Rate::from_hz(50),
            pulse_width_ns,
            duty,
        }
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
}

pub struct Servo<'a, S: TimerSpeed> {
    name: &'static str,
    channel: Channel<'a, S>,
    /// Valid duty cycle range in absolute values (e.g., 102..491 for SG90 with 12-bit).
    /// This corresponds to the pulse width range of the servo.
    pub duty_range: Range<u32>,
    config: ServoConfig,
    /// Cached max duty value for further calculations.
    max_duty: u32,
    /// Current direction. Clockwise or counter-clockwise.
    direction: Dir,
    /// Current duty in absolute value (0..max_duty).
    current_duty: u32,
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
        timer: &'a mut Timer<'a, S>,
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
        };

        // Calculate duty range in absolute values
        let duty_range = calc_duty_range(&config, max_duty);
        
        let mut channel = ledc.channel(channel_num, pin);
        channel.configure(channel::config::Config {
            timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })?;

        let center_duty = duty_range.start + (duty_range.end - duty_range.start) / 2;
        channel.set_duty_hw(center_duty);

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

    /// Makes step in absolute duty units should be lesser than [`duty_range()`](Self::duty_range).
    /// Return false if servo reaches min or max position.
    /// See also [`step_pct()`](Self::step_pct) for percentage-based stepping.
    pub fn step(&mut self, step_size: u32) -> Result<bool, channel::Error> {
        let new_duty = self.calc_duty(step_size);

        if new_duty == self.current_duty {
            return Ok(false);
        }

        // hardware method has better resolution
        self.channel.set_duty_hw(new_duty);
        self.current_duty = new_duty;
        trace!(
            "{} servo step({}) to duty={}/{}",
            &self.name,
            step_size,
            new_duty,
            self.max_duty
        );
        Ok(true)
    }

    /// Makes step in percentage of total range.
    /// Returns false if servo reaches min or max position.
    /// See also [`step()`](Self::step) for absolute duty-based stepping.
    pub fn step_pct(&mut self, step_pct: u8) -> Result<bool, channel::Error> {
        let step = ((step_pct as f64 / 100.0) * self.max_duty as f64 + 0.5) as u32;
        self.step(step)
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

    /// Returns current angle value in degrees.
    pub fn get_angle(&self) -> f64 {
        calculate_angle(&self.config, self.current_duty, self.max_duty)
    }

    /// Returns the size of the duty range (difference between max and min duty values).
    /// This is not the number of steps, but the range size in duty units.
    pub fn duty_range(&self) -> u32 {
        self.duty_range.end.saturating_sub(self.duty_range.start)
    }

    /// Calculates new duty based on current direction and step size.
    /// Returns clamped duty value within valid range.
    fn calc_duty(&self, step: u32) -> u32 {
        let new_duty = match self.direction {
            Dir::CW => {
                // Move clockwise (increase duty)
                self.current_duty.saturating_add(step)
            },
            Dir::CCW => {
                // Move counter-clockwise (decrease duty)
                self.current_duty.saturating_sub(step)
            },
        };

        new_duty.clamp(self.duty_range.start, self.duty_range.end.saturating_sub(1))
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum Dir {
    /// Clockwise, increases angle.
    CW,
    /// Counter-clockwise, decreases angle.
    CCW,
}

const NANOS_IS_SEC: f64 = 1_000_000_000.0;

/// Calculates duty range in absolute values for given servo configuration.
/// Returns absolute duty values (0..max_duty), not percentages.
fn calc_duty_range(config: &ServoConfig, max_duty: u32) -> Range<u32> {
    let min_pulse = config.pulse_width_ns.start;
    let max_pulse = config.pulse_width_ns.end;
    
    // Calculate absolute duty values directly from pulse width
    let min_duty = pulse_to_duty(config, min_pulse, max_duty);
    let max_duty_val = pulse_to_duty(config, max_pulse, max_duty);
    
    // Ensure values are within valid range
    let min_duty = min_duty.min(max_duty);
    let max_duty_val = max_duty_val.min(max_duty);
    
    // Ensure valid range (min < max)
    let min_duty = min_duty.min(max_duty_val.saturating_sub(1));
    let max_duty_val = max_duty_val.max(min_duty + 1);

    min_duty..max_duty_val
}

/// Transforms absolute duty value to angle in respect that given servo pulse range.
fn calculate_angle(config: &ServoConfig, duty: u32, max_duty: u32) -> f64 {
    // Convert duty to pulse width in nanoseconds
    let pulse_ns = duty as f64 * NANOS_IS_SEC / config.frequency.as_hz() as f64 / max_duty as f64;

    // Map pulse width to angle
    let pulse_range = (config.pulse_width_ns.end - config.pulse_width_ns.start) as f64;
    // Prevent division by zero
    if pulse_range <= 0.0 {
        return 0.0;
    }
    
    (pulse_ns - config.pulse_width_ns.start as f64)
        / pulse_range
        * config.max_angle
}

/// Maps pulse width in nanoseconds to absolute duty value as u32 (not percentage).
fn pulse_to_duty(config: &ServoConfig, pulse_ns: u32, max_duty: u32) -> u32 {
    let duty_f = pulse_ns as f64 * config.frequency.as_hz() as f64 * max_duty as f64 / NANOS_IS_SEC;
    (duty_f + 0.5) as u32
}