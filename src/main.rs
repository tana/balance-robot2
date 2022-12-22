use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::{prelude::*, spi::{SpiDeviceDriver, Dma, SpiConfig}, gpio::Gpio0, delay};
use shift_stepper::ShiftStepper;

mod shift_stepper;

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();

    let spi = SpiDeviceDriver::new_single(
        peripherals.spi2,
        peripherals.pins.gpio19, peripherals.pins.gpio22, Option::<Gpio0>::None,
        Dma::Disabled, Some(peripherals.pins.gpio23),
        &SpiConfig::new().baudrate(KiloHertz(100).into()).data_mode(embedded_hal::spi::MODE_0)
    ).unwrap();

    let mut steppers = ShiftStepper::new(spi);
    steppers.init().unwrap();

    loop {
        for _ in 0..240 {
            steppers.step_motor1(true).unwrap();
            steppers.step_motor2(false).unwrap();
            delay::FreeRtos::delay_ms(10);
        }

        delay::FreeRtos::delay_ms(1000);

        for _ in 0..240 {
            steppers.step_motor1(false).unwrap();
            steppers.step_motor2(true).unwrap();
            delay::FreeRtos::delay_ms(10);
        }

        delay::FreeRtos::delay_ms(1000);
    }
}
