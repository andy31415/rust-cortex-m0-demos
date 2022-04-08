#![no_std]
#![no_main]

use cortex_m::Peripherals;
use hal::adc::Adc;
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

    let gpio = pac_peripherals.GPIOA.split(&mut rcc);
    let (mut a0, mut a1) =
        cortex_m::interrupt::free(move |cs| (gpio.pa0.into_analog(cs), gpio.pa1.into_analog(cs)));

    rprintln!("Initialized. Ready to display values.");

    let mut adc = Adc::new(pac_peripherals.ADC, &mut rcc);
    adc.set_precision(hal::adc::AdcPrecision::B_12);

    loop {
        // your code goes here
        delay.delay_ms(1000_u16);

        rprintln!(
            "Analog readings: A0 = {}, A1 = {}",
            adc.read_abs_mv(&mut a0),
            adc.read_abs_mv(&mut a1)
        );
    }
}
