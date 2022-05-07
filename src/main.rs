#![no_std]
#![no_main]

use cortex_m::Peripherals;
use embedded_hal::digital::v2::OutputPin;
use panic_rtt_target as _;

use cortex_m_rt::entry;
use stm32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};

enum A4988State {
    On,
    Off,
}

struct A4988<ControlPin: OutputPin> {
    control: ControlPin,
    state: A4988State,
}

impl<ControlPin: OutputPin> A4988<ControlPin> {
    fn new(control: ControlPin) -> Self {
        Self {
            control,
            state: A4988State::Off,
        }
    }

    fn step(&mut self) -> Result<(), ControlPin::Error> {
        match self.state {
            A4988State::On => {
                self.control.set_low()?;
                self.state = A4988State::Off;
            }
            A4988State::Off => {
                self.control.set_high()?;
                self.state = A4988State::On;
            }
        }

        Ok(())
    }
}

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

    let gpioc = pac_peripherals.GPIOC.split(&mut rcc);
    let mut led = cortex_m::interrupt::free(move |cs| gpioc.pc13.into_push_pull_output(cs));

    let gpiob = pac_peripherals.GPIOB.split(&mut rcc);
    let motor = cortex_m::interrupt::free(move |cs| gpiob.pb3.into_push_pull_output(cs));

    let mut motor = A4988::new(motor);

    rprintln!("LED initialized. Will blink now.");

    loop {
        delay.delay_ms(20_u16);
        led.toggle().ok();
        motor.step().ok();
    }
}
