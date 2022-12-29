use embedded_hal::delay::DelayUs;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::{prelude::*, spi::{SpiDeviceDriver, Dma, SpiConfig}, gpio::{Gpio0, PinDriver}};
use esp_idf_svc::timer::EspTimerService;
use shift_stepper::{ShiftStepper, SpeedController};

mod shift_stepper;

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let mut delay = esp_idf_hal::delay::FreeRtos;
    
    let button = PinDriver::input(peripherals.pins.gpio39).unwrap();

    let spi = SpiDeviceDriver::new_single(
        peripherals.spi2,
        peripherals.pins.gpio19, peripherals.pins.gpio22, Option::<Gpio0>::None,
        Dma::Disabled, Some(peripherals.pins.gpio23),
        &SpiConfig::new().baudrate(100.kHz().into()).data_mode(embedded_hal::spi::MODE_0)
    ).unwrap();

    let mut steppers = ShiftStepper::new(spi);
    steppers.init().unwrap();
    steppers.set_active(false, false).unwrap();

    let mut timer_service = EspTimerService::new().unwrap();

    let mut speed_controller = SpeedController::new(
        steppers, &mut timer_service, 0.001
    ).unwrap();

    let mut active = false;
    let mut prev_pressed = false;

    loop {
        let pressed = button.is_low();
        if !prev_pressed && pressed {   // Detect button press
            println!("Button pressed");
            active = !active;
            speed_controller.set_active(active, active);
        }
        prev_pressed = pressed;

        speed_controller.set_speed(120.0, 0.0);
        delay.delay_ms(10).unwrap();
    }
}
