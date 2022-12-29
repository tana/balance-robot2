use std::sync::{Mutex, Arc};
use embedded_hal::spi::{SpiDevice, SpiBusWrite};
use embedded_svc::timer::{TimerService, PeriodicTimer};

#[derive(Clone, Default, PartialEq, Eq, Debug)]
pub enum Direction {
    #[default]
    Still,
    Forward,
    Backward
}

// Half-step sequence
// Due to a mistake of PCB design, a weird bit order is used.
// (MSB) B- A- A+ B+ (LSB)
const SEQUENCE: [u8; 8] = [
    0b0010,
    0b0011,
    0b0001,
    0b0101,
    0b0100,
    0b1100,
    0b1000,
    0b1010
];

pub struct ShiftStepper<SPI> {
    spi: SPI,
    motor1_phase: usize,
    motor2_phase: usize
}

impl<SPI> ShiftStepper<SPI>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBusWrite
{
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            motor1_phase: 0,
            motor2_phase: 0
        }
    }

    pub fn init(&mut self) -> Result<(), SPI::Error> {
        self.motor1_phase = 0;
        self.motor2_phase = 0;
        
        self.send()
    }

    pub fn step(&mut self, motor1_dir: Direction, motor2_dir: Direction) -> Result<(), SPI::Error> {
        self.motor1_phase = match motor1_dir {
            Direction::Still => self.motor1_phase,
            Direction::Forward => update_phase(self.motor1_phase, true),
            Direction::Backward => update_phase(self.motor1_phase, false)
        };
        self.motor2_phase = match motor2_dir {
            Direction::Still => self.motor2_phase,
            Direction::Forward => update_phase(self.motor2_phase, true),
            Direction::Backward => update_phase(self.motor2_phase, false)
        };

        self.send()
    }

    fn send(&mut self) -> Result<(), SPI::Error> {
        let motor1_bits = SEQUENCE[self.motor1_phase];
        let motor2_bits = SEQUENCE[self.motor2_phase];

        self.spi.write(&[(motor2_bits << 4) | motor1_bits])
    }
}

fn update_phase(phase: usize, forward: bool) -> usize {
    let next = if forward {
        phase as isize + 1
    } else {
        phase as isize - 1
    };

    next.rem_euclid(SEQUENCE.len() as isize) as usize
}

pub struct SpeedController<S, T> where
    S: SpiDevice,
    S::Bus: SpiBusWrite,
    T: TimerService
{
    #[allow(dead_code)]
    steppers: Arc<Mutex<ShiftStepper<S>>>,
    #[allow(dead_code)]
    timer: T::Timer,
    increment: Arc<Mutex<(i32, i32)>>,
    timer_period: f32
}

const COUNTER_MAX: i32 = 1000;

impl<S, T> SpeedController<S, T> where
    S: SpiDevice + Send + 'static,
    S::Bus: SpiBusWrite,
    T: TimerService
{
    pub fn new(steppers: ShiftStepper<S>, timer_serice: &mut T, timer_period: f32) -> Result<Self, T::Error> {
        let steppers = Arc::new(Mutex::new(steppers));
        let increment = Arc::new(Mutex::new((0i32, 0i32)));

        let mut timer = {
            let steppers = Arc::clone(&steppers);
            let increment = Arc::clone(&increment);

            let mut motor1_counter = 0;
            let mut motor2_counter = 0;

            timer_serice.timer(move || {
                let (motor1_increment, motor2_increment) = {
                    let increment = increment.lock().unwrap();
                    *increment
                };

                let motor1_dir = if motor1_counter + motor1_increment >= COUNTER_MAX {
                    motor1_counter = motor1_counter + motor1_increment - COUNTER_MAX;
                    Direction::Forward
                } else if motor1_counter + motor1_increment < 0 {
                    motor1_counter = motor1_counter + motor1_increment + COUNTER_MAX;
                    Direction::Backward
                } else {
                    motor1_counter += motor1_increment;
                    Direction::Still
                };

                let motor2_dir = if motor2_counter + motor2_increment >= COUNTER_MAX {
                    motor2_counter = motor2_counter + motor2_increment - COUNTER_MAX;
                    Direction::Forward
                } else if motor2_counter + motor2_increment < 0 {
                    motor2_counter = motor2_counter + motor2_increment + COUNTER_MAX;
                    Direction::Backward
                } else {
                    motor2_counter += motor2_increment;
                    Direction::Still
                };

                {
                    let mut steppers = steppers.lock().unwrap();
                    steppers.step(motor1_dir, motor2_dir).unwrap();
                }
            })?
        };

        timer.every(core::time::Duration::from_secs_f32(timer_period))?;

        Ok(Self {
            steppers,
            timer,
            increment,
            timer_period
        })
    }

    pub fn set_speed(&mut self, motor1_steps_per_sec: f32, motor2_steps_per_sec: f32) {
        let mut increment = self.increment.lock().unwrap();
        *increment = (self.speed_to_increment(motor1_steps_per_sec), self.speed_to_increment(motor2_steps_per_sec));
    }

    fn speed_to_increment(&self, steps_per_sec: f32) -> i32 {
        (steps_per_sec * self.timer_period * (COUNTER_MAX as f32)).round() as i32
    }
}
