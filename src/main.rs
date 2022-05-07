#![no_std]
#![no_main]

use cortex_m::Peripherals;
use panic_rtt_target as _;

use cortex_m_rt::entry;
use stm32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};

mod a4988 {

    use embedded_hal::digital::v2::OutputPin;

    enum Pulse {
        PulseHigh,
        StepsUntilPulse(u32),
    }

    pub enum StateError {
        AlreadyTurning,
        NeedMoreTicksPerStep,
    }

    pub enum MotorMovingState {
        Idle,
        Moving,
    }

    #[derive(PartialEq)]
    enum State {
        Idle,
        Moving { steps_left: u32, ticks: u32 },
    }

    pub struct Motor<ControlPin: OutputPin> {
        control: ControlPin,
        pulse: Pulse,
        state: State,
    }

    impl<ControlPin: OutputPin> Motor<ControlPin> {
        pub fn new(control: ControlPin) -> Self {
            Self {
                control,
                pulse: Pulse::StepsUntilPulse(0),
                state: State::Idle,
            }
        }

        pub fn start_turning(&mut self, steps: u32, ticks_per_step: u32) -> Result<(), StateError> {
            if self.state != State::Idle {
                return Err(StateError::AlreadyTurning);
            }

            if ticks_per_step < 1 {
                return Err(StateError::NeedMoreTicksPerStep);
            }

            self.state = State::Moving {
                steps_left: steps,
                ticks: ticks_per_step - 1,
            };
            self.pulse = Pulse::StepsUntilPulse(0); // next pulse will happen now

            Ok(())
        }

        pub fn step(&mut self) -> Result<MotorMovingState, ControlPin::Error> {
            match self.state {
                State::Idle => return Ok(MotorMovingState::Idle),
                State::Moving { steps_left, ticks } => {
                    self.pulse = match self.pulse {
                        Pulse::PulseHigh => {
                            self.control.set_low()?;
                            Pulse::StepsUntilPulse(ticks)
                        }
                        Pulse::StepsUntilPulse(0) => {
                            if steps_left == 0 {
                                self.state = State::Idle;
                                return Ok(MotorMovingState::Idle);
                            }
                            self.state = State::Moving {
                                steps_left: steps_left - 1,
                                ticks,
                            };
                            self.control.set_high()?;
                            Pulse::PulseHigh
                        }
                        Pulse::StepsUntilPulse(cnt) => Pulse::StepsUntilPulse(cnt - 1),
                    };
                }
            }

            Ok(MotorMovingState::Moving)
        }
    }
}

// Move:
//   turn(degrees).in(ms)
//
// Degrees: multiple of 1.8 on this motor, generally "turn_steps"
// Time: multiple of "time_steps"
//
// Updated API:
//    start_turning(steps, time_per_turn) -> Result (error if already turning)
//    step() -> Result(done or not yet)

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let mut pac_peripherals = pac::Peripherals::take().unwrap();
    let cortex_peripherals = Peripherals::take().unwrap();

    let mut rcc = pac_peripherals
        .RCC
        .configure()
        .sysclk(48.mhz())
        .freeze(&mut pac_peripherals.FLASH);

    let mut delay = Delay::new(cortex_peripherals.SYST, &rcc);

    let gpiob = pac_peripherals.GPIOB.split(&mut rcc);
    let motor = cortex_m::interrupt::free(move |cs| gpiob.pb3.into_push_pull_output(cs));

    let mut motor = a4988::Motor::new(motor);

    rprintln!("Ready to start");

    // Full turn, 5 ticks per turn. At 1KHz this is 1rot/sec
    motor.start_turning(200, 5).ok();

    let mut turns = 5;

    loop {
        delay.delay_ms(1_u16);
        match motor.step() {
            Ok(a4988::MotorMovingState::Idle) => {
                // Turn again
                turns = 15 - turns;
                motor.start_turning(200, turns).ok();
            }
            Ok(_) => {}
            Err(_) => {
                rprintln!("Error turning!");
            }
        }
    }
}
