//! Wrapper that makes [`Servo`] asynchronous by using async delays.
//! Enable the `async` feature to use these types and methods.
//!
//! # Example
//!
//! ```no_run
//! # #[cfg(feature = "async")]
//! # async fn example() {
//! use esp_hal_servo::{Servo, ServoConfig, async_servo::AsyncServo};
//! use embedded_hal_async::delay::DelayNs;
//!
//! // ledc, timer, config, peripherals, and delay are created elsewhere
//! let servo = Servo::new("servo", config, &mut ledc, &mut timer, channel::Number::Channel0, peripherals.GPIO2)?;
//! let mut async_servo = AsyncServo::new(servo, delay);
//!
//! // Set angle asynchronously (delay is automatically calculated)
//! async_servo.set_angle(90.0).await;
//!
//! // Step asynchronously
//! async_servo.set_dir(esp_hal_servo::Dir::CW);
//! async_servo.step_async(10.0).await?;
//! # }
//! ```

use crate::{Dir, Servo, TimerSpeed};
use core::ops::{Deref, DerefMut};
use embedded_hal_async::delay::DelayNs;
use esp_hal::ledc::channel;

/// Async wrapper around [`Servo`] that provides async versions of control methods.
///
/// This type wraps a synchronous [`Servo`] and provides async methods that use
/// a [`DelayNs`] implementation for non-blocking delays. The delay is automatically calculated
/// based on the servo's rotation speed and the angle of movement.
///
/// The `D` generic parameter represents any type that implements [`DelayNs`] trait,
/// allowing you to use any async delay implementation (e.g., Embassy timer, RTOS delay, etc.).
///
/// # Example
///
/// ```no_run
/// # #[cfg(feature = "async")]
/// # async fn example() {
/// use esp_hal_servo::{Servo, ServoConfig, async_servo::AsyncServo};
/// use embedded_hal_async::delay::DelayNs;
///
/// // Create servo with speed of 60 degrees per second (typical for SG90)
/// // servo and delay are created elsewhere
/// let mut async_servo = AsyncServo::new(servo, delay);
///
/// // Move servo to 45 degrees asynchronously
/// // Delay is automatically calculated based on rotation angle and speed
/// async_servo.set_angle(45.0).await;
///
/// // Make incremental steps
/// async_servo.set_dir(esp_hal_servo::Dir::CW);
/// for _ in 0..10 {
///     async_servo.step_async(5.0).await?;
/// }
/// # }
/// ```
pub struct AsyncServo<'a, S: TimerSpeed, D: DelayNs> {
    servo: Servo<'a, S>,
    delay: D,
}

impl<'a, S: TimerSpeed, D: DelayNs> AsyncServo<'a, S, D> {
    /// Creates a new async servo wrapper from a synchronous servo and a delay implementation.
    ///
    /// The servo's rotation speed (set in `ServoConfig`) is used to
    /// automatically calculate delays based on rotation angles.
    ///
    /// # Arguments
    ///
    /// * `servo` - The synchronous servo instance to wrap
    /// * `delay` - An implementation of [`DelayNs`] trait for async delays
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(feature = "async")]
    /// # async fn example() {
    /// # use esp_hal_servo::{Servo, ServoConfig, async_servo::AsyncServo};
    /// # use embedded_hal_async::delay::DelayNs;
    /// // Create servo with speed of 60 degrees per second (typical for SG90)
    /// // config, ledc, timer, channel_num, pin, and delay are created elsewhere
    /// let servo = Servo::new("servo", config, &mut ledc, &mut timer, channel_num, pin)?;
    /// let mut async_servo = AsyncServo::new(servo, delay);
    /// # }
    /// ```
    pub fn new(servo: Servo<'a, S>, delay: D) -> Self {
        Self { servo, delay }
    }

    /// Sets servo to specified angle in degrees asynchronously.
    ///
    /// This method sets the servo position and then waits for a calculated delay
    /// based on the rotation angle and servo speed to allow the servo to move to the new position.
    ///
    /// # Arguments
    ///
    /// * `angle` - Target angle in degrees
    ///
    /// # Returns
    ///
    /// Returns `true` if the angle was changed, `false` if the servo was already at that angle.
    pub async fn set_angle(&mut self, angle: f32) -> bool {
        let current_angle = self.servo.get_angle();
        let changed = self.servo.set_angle(angle);
        if changed {
            let angle_after = self.servo.get_angle();
            self.wait_for_movement(current_angle, angle_after).await;
        }
        changed
    }

    /// Sets servo to specified angle in degrees with a custom delay.
    ///
    /// Similar to [`set_angle`](Self::set_angle), but allows specifying
    /// a custom delay time instead of calculating it from servo speed.
    ///
    /// # Arguments
    ///
    /// * `angle` - Target angle in degrees
    /// * `delay_ms` - Delay in milliseconds after setting the angle
    ///
    /// # Returns
    ///
    /// Returns `true` if the angle was changed, `false` if the servo was already at that angle.
    pub async fn set_angle_with_delay(&mut self, angle: f32, delay_ms: u32) -> bool {
        let changed = self.servo.set_angle(angle);
        if changed && delay_ms > 0 {
            self.delay.delay_ms(delay_ms).await;
        }
        changed
    }

    /// Returns current angle value in degrees.
    pub fn get_angle(&self) -> f32 {
        self.servo.get_angle()
    }

    /// Set servo to move in new direction.
    ///
    /// This is a synchronous operation (instantaneous), so it doesn't need to be async.
    ///
    /// # Arguments
    ///
    /// * `dir` - New direction (clockwise or counter-clockwise)
    ///
    /// # Returns
    ///
    /// Returns the old direction if the direction was actually changed, `None` otherwise.
    pub fn set_dir(&mut self, dir: Dir) -> Option<Dir> {
        self.servo.set_dir(dir)
    }

    /// Returns current direction value.
    pub fn get_dir(&self) -> Dir {
        self.servo.get_dir()
    }

    /// Makes a step in percentage of total range asynchronously.
    ///
    /// This method makes a percentage-based step and then waits for a calculated delay
    /// based on the angle change and servo speed.
    ///
    /// # Arguments
    ///
    /// * `step_pct` - Step size as percentage of total range (0-100)
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if the step was made, `Ok(false)` if the servo reached
    /// min or max position, or an error if the operation failed.
    pub async fn step_pct(&mut self, step_pct: u8) -> Result<bool, channel::Error> {
        let angle_before = self.servo.get_angle();
        let result = self.servo.step_pct(step_pct)?;
        if result {
            let angle_after = self.servo.get_angle();
            self.wait_for_movement(angle_before, angle_after).await;
        }
        Ok(result)
    }

    /// Makes a step in percentage of total range with a custom delay.
    ///
    /// Similar to [`step_pct`](Self::step_pct), but allows specifying
    /// a custom delay time instead of using the default.
    ///
    /// # Arguments
    ///
    /// * `step_pct` - Step size as percentage of total range (0-100)
    /// * `delay_ms` - Delay in milliseconds after the step
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if the step was made, `Ok(false)` if the servo reached
    /// min or max position, or an error if the operation failed.
    pub async fn step_pct_with_delay(
        &mut self,
        step_pct: u8,
        delay_ms: u32,
    ) -> Result<bool, channel::Error> {
        let result = self.servo.step_pct(step_pct)?;
        if result && delay_ms > 0 {
            self.delay.delay_ms(delay_ms).await;
        }
        Ok(result)
    }

    /// Makes a step in absolute duty units asynchronously.
    ///
    /// This method makes a step and then waits for a calculated delay based on
    /// the angle change and servo speed.
    ///
    /// # Arguments
    ///
    /// * `step_size` - Step size in absolute duty units
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if the step was made, `Ok(false)` if the servo reached
    /// min or max position, or an error if the operation failed.
    pub async fn step_async(&mut self, step_size: f32) -> Result<bool, channel::Error> {
        let angle_before = self.servo.get_angle();
        let result = self.servo.step(step_size)?;
        if result {
            let angle_after = self.servo.get_angle();
            self.wait_for_movement(angle_before, angle_after).await;
        }
        Ok(result)
    }

    /// Returns the size of the duty range (difference between max and min duty values).
    ///
    /// This is not the number of steps, but the range size in duty units.
    pub fn duty_range(&self) -> f32 {
        self.servo.duty_range()
    }

    /// Returns a reference to the underlying synchronous servo.
    ///
    /// This method is provided for explicit access. You can also use
    /// `Deref` to access `Servo` methods directly (e.g., `async_servo.speed()`).
    pub fn sync_servo(&self) -> &Servo<'a, S> {
        &self.servo
    }

    /// Returns a mutable reference to the underlying synchronous servo.
    ///
    /// This method is provided for explicit access. You can also use
    /// `DerefMut` to access `Servo` methods directly (e.g., `async_servo.set_speed(100.0)`).
    pub fn sync_servo_mut(&mut self) -> &mut Servo<'a, S> {
        &mut self.servo
    }

    /// Waits for the servo to complete movement based on angle change.
    ///
    /// Calculates the delay based on the angle difference and servo speed,
    /// then waits for that duration if the delay is greater than zero.
    async fn wait_for_movement(&mut self, angle_before: f32, angle_after: f32) {
        let angle_diff = (angle_after - angle_before).abs();
        if let Some(delay_ms) = self.servo.calc_delay_ms(angle_diff)
            && delay_ms > 0
        {
            self.delay.delay_ms(delay_ms).await;
        }
    }
}

impl<'a, S: TimerSpeed, D: DelayNs> Deref for AsyncServo<'a, S, D> {
    type Target = Servo<'a, S>;

    fn deref(&self) -> &Self::Target {
        &self.servo
    }
}

impl<'a, S: TimerSpeed, D: DelayNs> DerefMut for AsyncServo<'a, S, D> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.servo
    }
}
