use embedded_hal::spi::{SpiDevice, SpiBusWrite};

// Half-step sequence
// (MSB) B- B+ A- A+ (LSB)
const SEQUENCE: [u8; 8] = [
    0b0001,
    0b0101,
    0b0100,
    0b0110,
    0b0010,
    0b1010,
    0b1000,
    0b1001
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

    pub fn step_motor1(&mut self, forward: bool) -> Result<(), SPI::Error> {
        self.motor1_phase = update_phase(self.motor1_phase, forward);

        self.send()
    }

    pub fn step_motor2(&mut self, forward: bool) -> Result<(), SPI::Error> {
        self.motor2_phase = update_phase(self.motor2_phase, forward);

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