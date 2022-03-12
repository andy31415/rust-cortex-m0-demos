#![no_std]
#![no_main]

use cortex_m::Peripherals;
use panic_rtt_target as _;

use cortex_m_rt::entry;
use stm32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*, pwm};

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
    let pa8 = cortex_m::interrupt::free(move |cs| gpio.pa8.into_alternate_af2(cs));

    let mut p = pwm::tim1(pac_peripherals.TIM1, pa8, &mut rcc, 50.hz());

    let duty_a = p.get_max_duty() / 20;
    let duty_b = p.get_max_duty() / 10;

    p.set_duty(duty_a);
    p.enable();

    rprintln!("PWM initialzed, ready to run");

    loop {
        delay.delay_ms(1000_u16);
        p.set_duty(duty_b);
        delay.delay_ms(1000_u16);
        p.set_duty(duty_a);
    }
}
