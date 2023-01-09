use std::{time::Duration, net::{UdpSocket, SocketAddrV4, Ipv4Addr}, sync::{Arc, Mutex}};
use complementary_filter::ComplemtaryFilter;
use embedded_hal::{digital::{InputPin, ToggleableOutputPin}, spi::{SpiDevice, SpiBusWrite}};
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::{prelude::*, spi::{SpiDeviceDriver, Dma, SpiConfig}, gpio::{Gpio0, PinDriver}, i2c::{I2cDriver, self}, task::thread::ThreadSpawnConfiguration, cpu::Core};
use esp_idf_svc::{timer::EspTimerService, wifi::EspWifi, eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition, mdns::EspMdns};
use mpu6050::Mpu6886;
use nalgebra::{vector, matrix};
use rosc::{OscMessage, OscPacket, OscType};
use shift_stepper::{ShiftStepper, SpeedController};

mod shift_stepper;
mod complementary_filter;

const WIFI_SSID: &str = "balance-robot";
const OSC_PORT: u16 = 12345;

const CONTROL_PERIOD: Duration = Duration::from_millis(10);
const STEPS_PER_ROTATION: f32 = 240.0;
const MAX_MOTOR_ANG_VEL: f32 = 4.0 * core::f32::consts::PI;
const FALL_ANGLE: f32 = 45.0 * core::f32::consts::PI / 180.0;
const TURN_RATE: f32 = 120.0 * core::f32::consts::PI / 180.0;

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();

    let button = PinDriver::input(peripherals.pins.gpio39).unwrap();

    let debug_pin = PinDriver::output(peripherals.pins.gpio33).unwrap();

    // Initialize I2C for IMU
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio25, peripherals.pins.gpio21,
        &i2c::config::Config::new().baudrate(400.kHz().into()).sda_enable_pullup(true).scl_enable_pullup(true)
    ).unwrap();

    // Initialize SPI for stepper driver
    let spi = SpiDeviceDriver::new_single(
        peripherals.spi2,
        peripherals.pins.gpio19, peripherals.pins.gpio22, Option::<Gpio0>::None,
        Dma::Disabled, Some(peripherals.pins.gpio23),
        &SpiConfig::new().baudrate(100.kHz().into()).data_mode(embedded_hal::spi::MODE_0)
    ).unwrap();
    
    // Initialize Wi-Fi in Soft AP mode
    let sysloop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().unwrap();
    let mut wifi = EspWifi::new(peripherals.modem, sysloop, Some(nvs)).unwrap();
    wifi.set_configuration(&embedded_svc::wifi::Configuration::AccessPoint(embedded_svc::wifi::AccessPointConfiguration {
        ssid: WIFI_SSID.into(),
        channel: 1,
        auth_method: embedded_svc::wifi::AuthMethod::None,  // No encryption because it was too heavy.
        ..Default::default()
    })).unwrap();
    wifi.start().unwrap();
    println!("IP: {}", wifi.ap_netif().get_ip_info().unwrap().ip);

    // Advertise this device as 'balance-robot.local' using mDNS
    let mut mdns = EspMdns::take().unwrap();
    mdns.set_hostname("balance-robot").unwrap();
    mdns.set_instance_name("Self-Balancing Robot").unwrap();

    // Shared variables for remote control
    let turn = Arc::<Mutex<f32>>::new(Mutex::new(0.0));

    // Handle OSC in another thread
    let _osc_thread_handle = {
        let turn = Arc::clone(&turn);
        std::thread::spawn(move || osc_thread(turn))
    };

    // Run control loop in another core with higiher priority
    let spawn_config = ThreadSpawnConfiguration {
        name: Some("control".as_bytes()),
        priority: 2,
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    };
    spawn_config.set().unwrap();
    let control_thread_handle = std::thread::spawn(
        move || control_thread(button, debug_pin, i2c, spi, turn)
    );

    control_thread_handle.join().unwrap();
}

fn control_thread<ButtonPin, DebugPin, Spi>(button: ButtonPin, mut debug_pin: DebugPin, i2c: I2cDriver, spi: Spi, turn: Arc<Mutex<f32>>) where
    ButtonPin: InputPin,
    DebugPin: ToggleableOutputPin,
    Spi: SpiDevice + Send + 'static,
    Spi::Bus: SpiBusWrite
{
    let mut delay = esp_idf_hal::delay::FreeRtos;

    // Initialize IMU
    let mut imu = Mpu6886::new(i2c);
    imu.init(&mut delay).unwrap();

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
        let pressed = button.is_low().unwrap();
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

        let (left_ang_vel, right_ang_vel) = {
            let turn = turn.lock().unwrap();
            apply_turn(motor_ang_vel, TURN_RATE * (*turn), MAX_MOTOR_ANG_VEL)
        };

        let left_steps_per_sec = -STEPS_PER_ROTATION * left_ang_vel / (2.0 * core::f32::consts::PI);    // Two motors are mounted in oppsite direction
        let right_steps_per_sec = STEPS_PER_ROTATION * right_ang_vel / (2.0 * core::f32::consts::PI);
        speed_controller.set_speed(left_steps_per_sec, right_steps_per_sec);

        debug_pin.toggle().unwrap();

        std::thread::sleep(CONTROL_PERIOD - last_time.elapsed());
        last_time = std::time::Instant::now();
    }
}

// Mix speed (average angular velocity of two motors) and turn (difference of angular velocities)
fn apply_turn(speed: f32, turn: f32, max_speed: f32) -> (f32, f32) {
    let headroom = max_speed - speed.abs();
    let turn = turn.clamp(-2.0 * headroom, 2.0 * headroom);
    (speed + 0.5 * turn, speed - 0.5 * turn)
}

fn osc_thread(turn: Arc<Mutex<f32>>) {
    // Create UDP socket for OSC
    let socket = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, OSC_PORT)).unwrap();

    let mut buf = vec![0; rosc::decoder::MTU];  // Use heap to avoid stack overflow

    loop {
        let (size, _) = socket.recv_from(&mut buf).unwrap();
        if let Ok((_, packet)) = rosc::decoder::decode_udp(&buf[..size])
        {
            let mut turn = turn.lock().unwrap();
            handle_osc_packet(&packet, &mut *turn);
        }
    }
}

fn handle_osc_packet(packet: &OscPacket, turn: &mut f32) {
    match packet {
        OscPacket::Message(msg) => handle_osc_message(msg, turn),
        OscPacket::Bundle(bundle) => bundle.content.iter().for_each(|p| handle_osc_packet(p, turn))
    }
}

fn handle_osc_message(msg: &OscMessage, turn: &mut f32) {
    match (msg.addr.as_str(), msg.args.as_slice()) {
        ("/turn", [OscType::Float(value)]) => *turn = *value,
        _ => ()
    }
}