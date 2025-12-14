#![doc = include_str!("../README.md")]
#![warn(missing_docs)]
#![no_std]

pub mod servo;
pub mod servo_config;
pub mod utils;

#[cfg(feature = "async")]
pub mod async_servo;

// Re-export main types for convenience
pub use servo::{Dir, Servo};
pub use servo_config::ServoConfig;

// Re-export TimerSpeed for convenience
pub use esp_hal::ledc::timer::TimerSpeed;
