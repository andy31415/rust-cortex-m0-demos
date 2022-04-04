#![no_std]
#![no_main]

use core::{
    cell::{Cell, RefCell},
    ops::Deref,
};

use cortex_m::{interrupt::Mutex, peripheral::NVIC, Peripherals};
use panic_rtt_target as _;

use cortex_m_rt::entry;

use stm32f0::stm32f0x0::{interrupt, EXTI};
use stm32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};

static PUSH_COUNT: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
static EXTI_PERIPHERAL: Mutex<RefCell<Option<EXTI>>> = Mutex::new(RefCell::new(None));

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
    let _led = cortex_m::interrupt::free(move |cs| gpio.pc13.into_push_pull_output(cs));

    let gpioa = pac_peripherals.GPIOA.split(&mut rcc);
    let _btn = cortex_m::interrupt::free(move |cs| gpioa.pa2.into_pull_up_input(cs));

    rprintln!("LED initialized. Will blink now.");

    let exti = pac_peripherals.EXTI;

    exti.imr.modify(|_, w| w.mr2().set_bit()); // interrupt mask register
    exti.ftsr.modify(|_, w| w.tr2().set_bit()); // falling trigger register

    cortex_m::interrupt::free(move |cs| {
        EXTI_PERIPHERAL.borrow(cs).replace(Some(exti));
    });

    unsafe {
        NVIC::unpend(stm32f0::stm32f0x0::Interrupt::EXTI2_3);
        NVIC::unmask(stm32f0::stm32f0x0::Interrupt::EXTI2_3);
    }

    loop {
        // your code goes here
        delay.delay_ms(500_u16);
        rprintln!(
            "PUSH COUNT: {}",
            cortex_m::interrupt::free(|cs| { PUSH_COUNT.borrow(cs).get() })
        );
    }
}

#[interrupt]
fn EXTI2_3() {
    NVIC::unpend(stm32f0::stm32f0x0::Interrupt::EXTI2_3);

    rprintln!("PUSH DETECTED!!!");
    cortex_m::interrupt::free(|cs| {
        PUSH_COUNT.borrow(cs).set(PUSH_COUNT.borrow(cs).get() + 1);

        if let Some(ref exti) = EXTI_PERIPHERAL.borrow(cs).borrow().deref() {
            exti.pr.modify(|_, w| w.pr1().clear_bit());
        }
    });
}
