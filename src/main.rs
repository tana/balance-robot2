use std::time::Duration;
use complementary_filter::ComplemtaryFilter;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::{prelude::*, spi::{SpiDeviceDriver, Dma, SpiConfig}, gpio::{Gpio0, PinDriver}, i2c::{I2cDriver, self}};
use esp_idf_svc::timer::EspTimerService;
use mpu6050::Mpu6886;
use nalgebra::{vector, matrix};
use shift_stepper::{ShiftStepper, SpeedController};

mod shift_stepper;
mod complementary_filter;

const CONTROL_PERIOD: Duration = Duration::from_millis(10);
const STEPS_PER_ROTATION: f32 = 240.0;
const MAX_MOTOR_ANG_VEL: f32 = 4.0 * core::f32::consts::PI;
const FALL_ANGLE: f32 = 45.0 * core::f32::consts::PI / 180.0;

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let mut delay = esp_idf_hal::delay::FreeRtos;
    
    let button = PinDriver::input(peripherals.pins.gpio39).unwrap();

    let mut debug_pin = PinDriver::output(peripherals.pins.gpio33).unwrap();

    // Initialize I2C for IMU
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio25, peripherals.pins.gpio21,
        &i2c::config::Config::new().baudrate(400.kHz().into()).sda_enable_pullup(true).scl_enable_pullup(true)
    ).unwrap();

    // Initialize IMU
    let mut imu = Mpu6886::new(i2c);
    imu.init(&mut delay).unwrap();

    // Initialize SPI for stepper driver
    let spi = SpiDeviceDriver::new_single(
        peripherals.spi2,
        peripherals.pins.gpio19, peripherals.pins.gpio22, Option::<Gpio0>::None,
        Dma::Disabled, Some(peripherals.pins.gpio23),
        &SpiConfig::new().baudrate(100.kHz().into()).data_mode(embedded_hal::spi::MODE_0)
    ).unwrap();

    // Initialize stepper driver
    let mut steppers = ShiftStepper::new(spi);
    steppers.init().unwrap();
    steppers.set_active(false, false).unwrap();

    // Initialize speed control for steppers
    let mut timer_service = EspTimerService::new().unwrap();
    let mut speed_controller = SpeedController::new(
        steppers, &mut timer_service, 0.001
    ).unwrap();

    // Initialize complementary filter
    let mut filter = ComplemtaryFilter::new(CONTROL_PERIOD.as_secs_f32());

    // For button handling
    let mut active = false;
    let mut prev_pressed = false;

    // For control
    let dt = CONTROL_PERIOD.as_secs_f32();
    let mut motor_ang_vel = 0.0;    // radians per sec
    let mut prev_angle = 0.0;
    let control_gain = matrix![-280.74261026, -89.66049857, -10.];    // LQR

    let mut last_time = std::time::Instant::now();

    loop {
        let pressed = button.is_low();
        if !prev_pressed && pressed {   // Detect button press
            active = !active;
            speed_controller.set_active(active, active);

            // Reset accumulating variables
            motor_ang_vel = 0.0;
        }
        prev_pressed = pressed;

        let accel = imu.get_acc().unwrap();
        let gyro = imu.get_gyro().unwrap();
        let accel_angle = -accel.y.atan2(-accel.z);   // Becomes 0 when a display is facing up
        let angle = filter.filter(accel_angle, gyro.x);
        let ang_vel = (angle - prev_angle) / dt;
        prev_angle = angle;

        // Stop motors when fell
        if angle.abs() >= FALL_ANGLE {
            active = false;
            speed_controller.set_active(active, active);
        }

        let state = vector![angle, ang_vel, motor_ang_vel];

        let motor_accel = (-control_gain * state)[(0, 0)];
        motor_ang_vel = (motor_ang_vel + motor_accel * dt).clamp(-MAX_MOTOR_ANG_VEL, MAX_MOTOR_ANG_VEL);

        let steps_per_sec = STEPS_PER_ROTATION * motor_ang_vel / (2.0 * core::f32::consts::PI);
        speed_controller.set_speed(-steps_per_sec, steps_per_sec);

        debug_pin.toggle().unwrap();

        std::thread::sleep(CONTROL_PERIOD - last_time.elapsed());
        last_time = std::time::Instant::now();
    }
}
