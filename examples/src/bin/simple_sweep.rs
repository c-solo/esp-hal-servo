//! Simple servo sweep example.
//!
//! This example demonstrates basic servo control by sweeping back and forth
//! between minimum and maximum positions.
//!
//! # Build and Flash
//!
//! ```bash
//! cargo run -p examples --bin simple_sweep --features esp32c3 --release
//! ```

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_hal::{
    Config,
    delay::Delay,
    ledc::{Ledc, channel, timer},
};
use esp_hal_servo::{Dir, Servo, ServoConfig};
use log::info;

esp_app_desc!();

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = esp_hal::init(Config::default());
    esp_println::logger::init_logger(log::LevelFilter::Info);

    info!("Starting servo sweep example");

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
    let mut servo = Servo::new(
        "sweep",
        config.clone(),
        &mut ledc,
        &mut timer,
        channel::Number::Channel0,
        peripherals.GPIO2,
    )
    .expect("failed to create servo");

    info!("Initial angle: {:.2}°", servo.get_angle());

    let delay = Delay::new();
    let mut step_pct = 1;
    loop {
        // Sweep forward (CW)
        servo.set_dir(Dir::CW);
        info!("Sweeping clockwise...");
        while servo.step_pct(step_pct).unwrap() {
            delay.delay_millis(50);
        }
        info!(
            "Reached max position: {:.2}°, step={step_pct}%",
            servo.get_angle()
        );
        delay.delay_millis(1000);

        // Sweep backward (CCW)
        servo.set_dir(Dir::CCW);
        info!("Sweeping counter-clockwise...");
        while servo.step_pct(step_pct).unwrap() {
            delay.delay_millis(50);
        }
        info!(
            "Reached min position: {:.2}°, step={step_pct}%",
            servo.get_angle()
        );
        delay.delay_millis(500);
        step_pct += 1;
        if step_pct >= 10 {
            step_pct = 1;
        }
    }
}
