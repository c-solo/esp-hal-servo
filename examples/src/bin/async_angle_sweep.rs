//! Async servo angle sweep example.
//!
//! This example demonstrates async servo control using only angle-based API.
//! The servo sweeps back and forth between minimum and maximum positions
//! using `set_angle` method with automatic delay calculation.
//!
//! # Build and Flash
//!
//! ```bash
//! cargo run -p examples --bin async_angle_sweep --features esp32c3,embassy --release
//! ```

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_hal::{
    Config,
    ledc::{Ledc, channel, timer},
};
use esp_hal_servo::{Servo, ServoConfig, async_servo::AsyncServo};
use log::info;

esp_app_desc!();

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let peripherals = esp_hal::init(Config::default());
    esp_println::logger::init_logger(log::LevelFilter::Info);

    // Initialize Embassy time driver
    // Note: For esp-hal 1.0, you need to initialize the time driver
    // This typically requires creating a driver that implements embassy_time_driver::Driver trait
    // and registering it using embassy_time_driver::time_driver_impl! macro
    // Please refer to esp-hal and embassy-time-driver documentation for the correct initialization method
    // Example: embassy_time_driver::time_driver_impl!(static DRIVER: MyDriver = MyDriver::new(systimer.alarm0, systimer.alarm1, systimer.alarm2));

    info!("Starting async servo angle sweep example");

    let config = ServoConfig::sg90(timer::config::Duty::Duty12Bit);

    // Create LEDC controller and configure timer
    let mut ledc = Ledc::new(peripherals.LEDC);
    let mut timer = config
        .configure_timer(
            &mut ledc,
            timer::Number::Timer0,
            timer::LSClockSource::APBClk,
        )
        .expect("failed to configure timer");

    // Create servo on GPIO 2
    let servo = Servo::new(
        "async_sweep",
        config.clone(),
        &mut ledc,
        &mut timer,
        channel::Number::Channel0,
        peripherals.GPIO2,
    )
    .expect("failed to create servo");

    info!("Initial angle: {:.2}°", servo.get_angle());

    // Wrap servo in AsyncServo for async control
    let mut async_servo = AsyncServo::new(servo);

    loop {
        // Move to minimum position (0 degrees)
        info!("Moving to 0°...");
        async_servo.set_angle(0.0).await;
        info!("Reached 0°");

        // Wait a bit
        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;

        // Move to maximum position (180 degrees)
        info!("Moving to 180°...");
        async_servo.set_angle(180.0).await;
        info!("Reached 180°");

        // Wait a bit
        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;

        // Move to center position (90 degrees)
        info!("Moving to 90°...");
        async_servo.set_angle(90.0).await;
        info!("Reached 90°");

        // Wait before next cycle
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
