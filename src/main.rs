#![no_std]
#![no_main]

use cortex_m::Peripherals;
use panic_rtt_target as _;

use cortex_m_rt::entry;
use stm32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};

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

    let gpio = pac_peripherals.GPIOC.split(&mut rcc);
    let mut led = cortex_m::interrupt::free(move |cs| gpio.pc13.into_push_pull_output(cs));

    rprintln!("LED initialized. Will blink now.");

    loop {
        // your code goes here
        delay.delay_ms(300_u16);
        led.toggle().ok();
    }
}
