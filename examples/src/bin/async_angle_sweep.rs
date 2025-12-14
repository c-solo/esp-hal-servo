//! Async servo angle sweep example.
//!
//! This example demonstrates async servo control using only angle-based API.
//! The servo sweeps back and forth between minimum and maximum positions
//! using `set_angle` method with automatic delay calculation.
//!
//! # Build and Flash
//!
//! ```bash
//! cargo run -p examples --bin async_angle_sweep --features esp32c3,async --release
//! ```

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_hal::{
    Config,
    interrupt::software::SoftwareInterruptControl,
    ledc::{Ledc, channel, timer},
    timer::timg::TimerGroup,
};
use esp_hal_servo::{Servo, ServoConfig, async_servo::AsyncServo};
use log::info;

esp_app_desc!();

#[esp_rtos::main]
async fn main(_s: embassy_executor::Spawner) {
    let peripherals = esp_hal::init(Config::default());
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let timer_group0 = TimerGroup::new(peripherals.TIMG0);
    let timer0 = timer_group0.timer0;
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let int0 = sw_ints.software_interrupt0;
    esp_rtos::start(timer0, int0);

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
    // embassy_time::Delay implements DelayNs trait and is provided by esp-rtos
    let mut async_servo = AsyncServo::new(servo, embassy_time::Delay);

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
    }
}
