#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::interrupt::free;
use cortex_m::interrupt::Mutex;

use hal::gpio::Output;
use hal::gpio::PushPull;
use hal::time::KiloHertz;
use hal::timers::Timer;
use panic_rtt_target as _;

use cortex_m_rt::entry;
use hal::stm32::interrupt;
use hal::stm32::Interrupt;
use stm32f0::stm32f0x0::TIM3;
use stm32f0xx_hal as hal;

use crate::hal::{pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};

mod a4988 {

    use embedded_hal::digital::v2::OutputPin;

    enum Pulse {
        PulseHigh,
        StepsUntilPulse(u32),
    }

    impl Steps {
        pub fn count(&self) -> u32 {
            self.0
        }
    }

    #[derive(PartialEq, Copy, Clone)]
    pub struct Ticks(pub u32);

    impl Ticks {
        pub fn count(&self) -> u32 {
            self.0
        }
    }

    #[derive(PartialEq, Copy, Clone)]
    pub struct Steps(pub u32);

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
        Moving { steps: Steps, ticks: Ticks },
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

        pub fn start_turning(&mut self, steps: Steps, ticks: Ticks) -> Result<(), StateError> {
            if self.state != State::Idle {
                return Err(StateError::AlreadyTurning);
            }

            if ticks.count() < 1 {
                return Err(StateError::NeedMoreTicksPerStep);
            }

            self.state = State::Moving {
                steps,
                ticks: Ticks(ticks.count() - 1),
            };
            self.pulse = Pulse::StepsUntilPulse(0); // next pulse will happen now

            Ok(())
        }

        pub fn is_idle(&self) -> bool {
            self.state == State::Idle
        }

        pub fn step(&mut self) -> Result<MotorMovingState, ControlPin::Error> {
            match self.state {
                State::Idle => return Ok(MotorMovingState::Idle),
                State::Moving { steps, ticks } => {
                    self.pulse = match self.pulse {
                        Pulse::PulseHigh => {
                            self.control.set_low()?;
                            Pulse::StepsUntilPulse(ticks.count())
                        }
                        Pulse::StepsUntilPulse(0) => {
                            if steps.count() == 0 {
                                self.state = State::Idle;
                                return Ok(MotorMovingState::Idle);
                            }
                            self.state = State::Moving {
                                steps: Steps(steps.count() - 1),
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

static MOTOR_TIMER: Mutex<RefCell<Option<Timer<TIM3>>>> = Mutex::new(RefCell::new(None));
static MOTOR1: Mutex<RefCell<Option<a4988::Motor<hal::gpio::gpiob::PB3<Output<PushPull>>>>>> =
    Mutex::new(RefCell::new(None));

// Move:
//   turn(degrees).in(ms)
//
// Degrees: multiple of 1.8 on this motor, generally "turn_steps"
// Time: multiple of "time_steps"
//
// Updated API:
//    start_turning(steps, time_per_turn) -> Result (error if already turning)
//    step() -> Result(done or not yet)

fn global_motor_turn(steps: u32, ticks: u32) {
    free(|cs| {
        if let Some(ref mut m) = MOTOR1.borrow(cs).borrow_mut().deref_mut() {
            if let Err(_) = m.start_turning(a4988::Steps(steps), a4988::Ticks(ticks)) {
                rprintln!("ERROR STARTING TO TURN!");
            }
        }
    });
}

fn global_motor_idle() -> bool {
    free(|cs| {
        if let Some(ref mut m) = MOTOR1.borrow(cs).borrow_mut().deref_mut() {
            m.is_idle()
        } else {
            false
        }
    })
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let mut pac_peripherals = pac::Peripherals::take().unwrap();

    let mut rcc = pac_peripherals
        .RCC
        .configure()
        .sysclk(48.mhz())
        .freeze(&mut pac_peripherals.FLASH);

    let gpiob = pac_peripherals.GPIOB.split(&mut rcc);
    let motor = cortex_m::interrupt::free(move |cs| gpiob.pb3.into_push_pull_output(cs));

    let motor = a4988::Motor::new(motor);

    rprintln!("Ready to start");

    let mut t3 = Timer::tim3(pac_peripherals.TIM3, KiloHertz(100), &mut rcc);
    t3.listen(hal::timers::Event::TimeOut);

    free(|cs| {
        MOTOR_TIMER.borrow(cs).borrow_mut().replace(t3);
        MOTOR1.borrow(cs).borrow_mut().replace(motor);
    });

    hal::pac::NVIC::unpend(Interrupt::TIM3);
    unsafe {
        hal::pac::NVIC::unmask(Interrupt::TIM3);
    }

    // at quad:
    // 100KHz
    // 200*16 steps/sec = 3200
    let mut ticks = 31;
    let turns = 16 * 200;
    global_motor_turn(turns, ticks);

    loop {
        if global_motor_idle() {
            ticks = 93 - ticks;
            global_motor_turn(turns, ticks);
        } else {
            cortex_m::asm::wfe();
        }
    }
}

#[interrupt]
fn TIM3() {
    hal::pac::NVIC::unpend(stm32f0::stm32f0x0::Interrupt::TIM3);
    free(|cs| {
        if let Some(ref mut t3) = MOTOR_TIMER.borrow(cs).borrow_mut().deref_mut() {
            t3.wait().ok(); // clear the wrapped bit
        }
        if let Some(ref mut m) = MOTOR1.borrow(cs).borrow_mut().deref_mut() {
            if let Err(_) = m.step() {
                rprintln!("Error stepping!");
            }
        }
    })
}
