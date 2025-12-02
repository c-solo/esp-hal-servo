//! This is a small lib for controlling servo using LEDC from [`esp-hal`](https://docs.rs/esp-hal/1.0.0/esp_hal/).

#![no_std]

use core::{marker::PhantomData, ops::Range};

use esp_hal::ledc::timer::config::Duty;
use esp_hal::ledc::timer::{TimerHW, TimerSpeed};
use esp_hal::{
    gpio::DriveMode,
    ledc::{
        LSGlobalClkSource, Ledc,
        channel::{self, Channel, ChannelIFace},
        timer::{self, Timer, TimerIFace},
    },
    time::Rate,
};
use log::{info, trace};

#[derive(Debug, Clone)]
pub struct ServoConfig {
    /// Max angle that servo can't be turned, mostly 180, 360.
    pub max_angle: f64,
    /// What frequency expect servo (ex. 50Hz for SG90).
    pub frequency: Rate,
    /// What pulse width in nanos servo supports (ex. 500-2400ns for SG90).
    pub pulse_width_ns: Range<u32>,
    /// PWM resolution in bits. Higher bits means more precise control.
    pub duty: Duty,
    /// How much add\subtract to 'duty' for making micro step
    pub step: u32,
}

impl ServoConfig {
    /// Config for [SG90](https://www.friendlywire.com/projects/ne555-servo-safe/SG90-datasheet.pdf).
    pub fn sg90(duty: Duty) -> Self {
        let pulse_width_ns = 500..2600;
        let max_angle = 180.0;
        let step = 5;
        ServoConfig {
            max_angle,
            frequency: Rate::from_hz(50),
            pulse_width_ns,
            duty,
            step,
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
    duty_range_pct: Range<u8>,
    config: ServoConfig,
    /// Cached max duty value for further calculations.
    max_duty: u32,
    /// Current direction. Clockwise or counter-clockwise.
    direction: Dir,
    /// Current duty in percentage.
    current_duty_pct: u8,
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

        let duty_range_pct = calc_duty_range_pct(&config, max_duty);

        // set to center position
        let center_pct = duty_range_pct.start + (duty_range_pct.end - duty_range_pct.start) / 2;

        let mut channel = ledc.channel(channel_num, pin);
        channel.configure(channel::config::Config {
            timer,
            duty_pct: center_pct,
            drive_mode: DriveMode::PushPull,
        })?;

        info!(
            "{name} servo: center={center_pct}%, duty_range={duty_range_pct:?}%",
            name = name,
            center_pct = center_pct,
            duty_range_pct = duty_range_pct
        );

        Ok(Servo::<'a> {
            name,
            channel,
            duty_range_pct,
            config,
            direction: Dir::CW,
            current_duty_pct: center_pct,
            max_duty,
            _p: PhantomData,
        })
    }

    /// Makes micro step, return false if servo reaches min or max position.
    pub fn step(&mut self, step: u32) -> Result<bool, channel::Error> {
        let new_duty_pct = self.calc_duty_pct(step);

        if new_duty_pct > self.duty_range_pct.end || new_duty_pct < self.duty_range_pct.start {
            // servo reaches bounds, skip step
            return Ok(false);
        }

        self.channel.set_duty(new_duty_pct)?;
        self.current_duty_pct = new_duty_pct;
        trace!("{} servo step({}) to {}%", &self.name, step, new_duty_pct);
        Ok(true)
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
        calculate_angle(&self.config, self.current_duty_pct, self.max_duty)
    }

    fn calc_duty_pct(&self, step: u32) -> u8 {
        // Convert step to percentage change
        // We need to calculate based on the duty range
        let duty_range_size = (self.duty_range_pct.end - self.duty_range_pct.start) as u32;
        let step_pct = if duty_range_size > 0 {
            (step * 100) / duty_range_size
        } else {
            0
        };

        let new_duty = match self.direction {
            Dir::CW => self.current_duty_pct as u32 + step_pct,
            Dir::CCW => self.current_duty_pct as u32 - step_pct.min(self.current_duty_pct as u32),
        };

        new_duty.min(100).max(0) as u8
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

/// Calculates duty range in percentage for given servo configuration.
fn calc_duty_range_pct(config: &ServoConfig, max_duty: u32) -> Range<u8> {
    let min_pulse = config.pulse_width_ns.start;
    let max_pulse = config.pulse_width_ns.end;
    let min_duty = pulse_to_duty(config, min_pulse, max_duty);
    let max_duty_val = pulse_to_duty(config, max_pulse, max_duty);

    // Convert duty values to percentages
    let min_pct = ((min_duty * 100) / max_duty).min(100) as u8;
    let max_pct = ((max_duty_val * 100) / max_duty).min(100) as u8;

    min_pct..max_pct
}

/// Transforms 'duty' percentage to 'angle' in respect that given servo pulse range.
fn calculate_angle(config: &ServoConfig, duty_pct: u8, max_duty: u32) -> f64 {
    let duty = (duty_pct as u32 * max_duty) / 100;
    let pulse_ns = duty as f64 * NANOS_IS_SEC / config.frequency.as_hz() as f64 / max_duty as f64;

    (pulse_ns - config.pulse_width_ns.start as f64)
        / (config.pulse_width_ns.end - config.pulse_width_ns.start) as f64
        * config.max_angle
}

/// Maps pulse width in nanoseconds to duty value.
fn pulse_to_duty(config: &ServoConfig, pulse_ns: u32, max_duty: u32) -> u32 {
    let duty = pulse_ns as f64 * config.frequency.as_hz() as f64 * max_duty as f64 / NANOS_IS_SEC;
    duty as u32
}