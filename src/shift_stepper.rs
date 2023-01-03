use std::{sync::{Mutex, Arc}, thread::JoinHandle};
use embedded_hal::spi::{SpiDevice, SpiBusWrite};
use embedded_svc::timer::PeriodicTimer;
use esp_idf_hal::task::thread::ThreadSpawnConfiguration;

#[derive(Clone, Default, PartialEq, Eq, Debug)]
pub enum Direction {
    #[default]
    Still,
    Forward,
    Backward
}

const STEP_DIV: usize = 4;

// Generate microstepping sequence
// Due to a mistake of PCB design, a weird order is used.
// [B-, A-, A+, B+]
fn generate_microstep_seq() -> [[usize; 4]; 4 * (STEP_DIV - 1)] {
    let mut sequence: [[usize; 4]; 4 * (STEP_DIV - 1)] = Default::default();

    // 0011 to 0101
    sequence[0..(STEP_DIV - 1)].iter_mut().enumerate().for_each(|(i, pattern)| {
        *pattern = [0, i, STEP_DIV - 1 - i, STEP_DIV - 1];
    });
    // 0101 to 1100
    sequence[(STEP_DIV - 1)..(2 * (STEP_DIV - 1))].iter_mut().enumerate().for_each(|(i, pattern)| {
        *pattern = [i, STEP_DIV - 1, 0, STEP_DIV - 1 - i];
    });
    // 1100 to 1010
    sequence[(2 * (STEP_DIV - 1))..(3 * (STEP_DIV - 1))].iter_mut().enumerate().for_each(|(i, pattern)| {
        *pattern = [STEP_DIV - 1, STEP_DIV - 1 - i, i, 0];
    });
    // 1010 to 0011
    sequence[(3 * (STEP_DIV - 1))..(4 * (STEP_DIV - 1))].iter_mut().enumerate().for_each(|(i, pattern)| {
        *pattern = [STEP_DIV - 1 - i, 0, STEP_DIV - 1, i];
    });

    sequence
}

pub struct ShiftStepper where
{
    sequence: [[usize; 4]; 4 * (STEP_DIV - 1)],
    #[allow(dead_code)]
    join_handle: JoinHandle<()>,
    duty: Arc<Mutex<([usize; 4], [usize; 4])>>,
    motor1_phase: usize,
    motor2_phase: usize,
    motor1_active: bool,
    motor2_active: bool
}

impl ShiftStepper where
{
    pub fn new<Spi>(mut spi: Spi) -> Self where
        Spi: SpiDevice + Send + 'static,
        Spi::Bus: SpiBusWrite,
    {
        let duty = Arc::new(Mutex::new(([0, 0, 0, 0], [0, 0, 0, 0])));

        let join_handle = {
            let duty = Arc::clone(&duty);

            // The thread runs in Core 1 (APP_CPU)
            let mut thread_conf = ThreadSpawnConfiguration::default();
            thread_conf.name = Some("PWM".as_bytes());
            thread_conf.pin_to_core = Some(esp_idf_hal::cpu::Core::Core1);
            thread_conf.priority = 10;
            thread_conf.set().unwrap();

            // PWM in another thread
            std::thread::spawn(move || {
                loop {
                    let (motor1_duty, motor2_duty) = {
                        let lock = duty.lock().unwrap();
                        *lock
                    };

                    for pwm_count in 0..=STEP_DIV {
                        let motor1_bits: u8 = motor1_duty.iter()
                            .enumerate().map(|(i, d)| if pwm_count < *d { 1 << i } else { 0 })
                            .sum();
                        let motor2_bits: u8 = motor2_duty.iter()
                            .enumerate().map(|(i, d)| if pwm_count < *d { 1 << i } else { 0 })
                            .sum();

                        spi.write(&[(motor2_bits << 4) | motor1_bits]).unwrap();

                        esp_idf_hal::delay::Ets::delay_us(100);
                    }
                }
            })
        };

        Self {
            sequence: generate_microstep_seq(),
            join_handle,
            duty,
            motor1_phase: 0,
            motor2_phase: 0,
            motor1_active: false,
            motor2_active: false
        }
    }

    pub fn init(&mut self) {
        self.motor1_phase = 0;
        self.motor2_phase = 0;
        self.motor1_active = true;
        self.motor2_active = true;

        self.step(Direction::Still, Direction::Still)
    }

    pub fn step(&mut self, motor1_dir: Direction, motor2_dir: Direction) {
        self.motor1_phase = match motor1_dir {
            Direction::Still => self.motor1_phase,
            Direction::Forward => update_phase(self.sequence.len(), self.motor1_phase, true),
            Direction::Backward => update_phase(self.sequence.len(), self.motor1_phase, false)
        };
        self.motor2_phase = match motor2_dir {
            Direction::Still => self.motor2_phase,
            Direction::Forward => update_phase(self.sequence.len(), self.motor2_phase, true),
            Direction::Backward => update_phase(self.sequence.len(), self.motor2_phase, false)
        };

        let motor1_duty = if self.motor1_active { self.sequence[self.motor1_phase] } else { [0, 0, 0, 0] };
        let motor2_duty = if self.motor2_active { self.sequence[self.motor2_phase] } else { [0, 0, 0, 0] };

        let mut lock = self.duty.lock().unwrap();
        *lock = (motor1_duty, motor2_duty);
    }

    pub fn set_active(&mut self, motor1: bool, motor2: bool) {
        self.motor1_active = motor1;
        self.motor2_active = motor2;

        self.step(Direction::Still, Direction::Still)
    }
}

fn update_phase(len: usize, phase: usize, forward: bool) -> usize {
    let next = if forward {
        phase as isize + 1
    } else {
        phase as isize - 1
    };

    next.rem_euclid(len as isize) as usize
}

pub struct SpeedController<TimerService> where
    TimerService: embedded_svc::timer::TimerService,
    ShiftStepper: Send
{
    #[allow(dead_code)]
    steppers: Arc<Mutex<ShiftStepper>>,
    #[allow(dead_code)]
    timer: TimerService::Timer,
    increment: Arc<Mutex<(i32, i32)>>,
    timer_period: f32
}

const COUNTER_MAX: i32 = 1000;

impl<TimerService> SpeedController<TimerService> where
    TimerService: embedded_svc::timer::TimerService + 'static,
    ShiftStepper: Send
{
    pub fn new(steppers: ShiftStepper, timer_serice: &mut TimerService, timer_period: f32) -> Result<Self, TimerService::Error> {
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
                    steppers.step(motor1_dir, motor2_dir)
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

    pub fn set_active(&mut self, motor1: bool, motor2: bool) {
        let mut steppers = self.steppers.lock().unwrap();
        steppers.set_active(motor1, motor2)
    }

    fn speed_to_increment(&self, steps_per_sec: f32) -> i32 {
        (steps_per_sec * self.timer_period * (COUNTER_MAX as f32)).round() as i32
    }
}
