#![feature(alloc_error_handler)]
#![no_std]
#![no_main]

use panic_rtt_target as _;

use cortex_m_rt::entry;
use stm32f0xx_hal as hal;

use crate::hal::{i2c, pac, prelude::*};
use rtt_target::{rprintln, rtt_init_print};

// Example simple 'output text' dependencies:
//   - Graphics library
//   - I2C display driver for SSD1306
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

// Example enabling of cortex-M heap, of size HEAP_SIZE
#[macro_use]
extern crate alloc;

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use core::mem::MaybeUninit;

const HEAP_SIZE: usize = 1024;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

#[entry]
fn main() -> ! {
    {
        unsafe { ALLOCATOR.init((&mut HEAP).as_ptr() as usize, HEAP_SIZE) }
    }
    rtt_init_print!();

    let mut pac_peripherals = pac::Peripherals::take().unwrap();

    let mut rcc = pac_peripherals
        .RCC
        .configure()
        .sysclk(48.mhz())
        .freeze(&mut pac_peripherals.FLASH);

    // Pins setup for i2c communication
    // See https://docs.rs/stm32f0xx-hal/0.18.0/stm32f0xx_hal/i2c/struct.I2c.html
    // for what pins are allowed and how
    //    For example https://docs.rs/stm32f0xx-hal/0.18.0/stm32f0xx_hal/i2c/trait.SclPin.html for clocks
    let gpiob = pac_peripherals.GPIOB.split(&mut rcc);
    let i2c_pins = cortex_m::interrupt::free(move |cs| {
        (
            gpiob.pb6.into_alternate_af1(cs),
            gpiob.pb7.into_alternate_af1(cs),
        )
    });
    let i2c = i2c::I2c::i2c1(pac_peripherals.I2C1, i2c_pins, 100.khz(), &mut rcc);

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut cnt = 0;
    loop {
        cnt += 1;

        let txt = format!("Loop counter {}", cnt);

        display.clear();

        Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline(&txt, Point::new(0, 16), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();

        // your code goes here
        // delay.delay_ms(300_u16);
        // led.toggle().ok();
    }
}

#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    rprintln!("OOM");
    loop {}
}
