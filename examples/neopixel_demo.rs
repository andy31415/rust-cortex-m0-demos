#![feature(alloc_error_handler)]
#![no_std]
#![no_main]

use embedded_hal::spi::{Mode, Phase, Polarity};
use smart_leds::brightness;

use crate::hal::delay::Delay;

use cortex_m::{asm, Peripherals};
use hal::spi::Spi;
use panic_rtt_target as _;

use cortex_m_rt::entry;
use smart_leds::SmartLedsWrite;
use stm32f0xx_hal as hal;

use crate::hal::{pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};
use smart_leds::RGB8;

use spi_constants::ws2812_constants;

ws2812_constants!(WRITE_3_BYTE_CONSTANTS);

pub struct SpiWrapper<SPI: embedded_hal::blocking::spi::Write<u8>> {
    spi: SPI,
}

impl<SPI: embedded_hal::blocking::spi::Write<u8>> SpiWrapper<SPI> {
    fn new(spi: SPI) -> Self {
        SpiWrapper { spi }
    }

    fn flush(&mut self) -> anyhow::Result<()> {
        // Should be > 300μs, so for an SPI Freq. of 3.8MHz, we have to send at least 1140 low bits or 140 low bytes
        self.spi
            .write(&[0u8; 140])
            .map_err(|_e| anyhow::anyhow!("Write error"))?;
        Ok(())
    }
}

impl<SPI: embedded_hal::blocking::spi::Write<u8>> SmartLedsWrite for SpiWrapper<SPI> {
    type Error = anyhow::Error;
    type Color = RGB8;

    /// Write all the items of an iterator to a ws2812 strip
    fn write<T, I>(&mut self, iterator: T) -> Result<(), anyhow::Error>
    where
        T: Iterator<Item = I>,
        I: Into<Self::Color>,
    {
        let mut buffer = [0u8; 12];
        for item in iterator {
            let item = item.into();

            buffer[0..4].copy_from_slice(&WRITE_3_BYTE_CONSTANTS[item.g as usize]);
            buffer[4..8].copy_from_slice(&WRITE_3_BYTE_CONSTANTS[item.r as usize]);
            buffer[8..12].copy_from_slice(&WRITE_3_BYTE_CONSTANTS[item.b as usize]);

            self.spi
                .write(&buffer)
                .map_err(|_e| anyhow::anyhow!("Write error"))?;
        }
        self.flush()?;

        Ok(())
    }
}

// Example enabling of cortex-M heap, of size HEAP_SIZE
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
    rtt_init_print!();

    unsafe { ALLOCATOR.init((&HEAP).as_ptr() as usize, HEAP_SIZE) }

    let mut pac_peripherals = pac::Peripherals::take().unwrap();
    let cortex_peripherals = Peripherals::take().unwrap();

    let mut rcc = pac_peripherals
        .RCC
        .configure()
        .sysclk(48.mhz())
        .freeze(&mut pac_peripherals.FLASH);

    rprintln!("SClock: {} Hz", rcc.clocks.sysclk().0);
    rprintln!("PClock: {} Hz", rcc.clocks.pclk().0);

    let mut delay = Delay::new(cortex_peripherals.SYST, &rcc);

    let gpioa = pac_peripherals.GPIOA.split(&mut rcc);

    let (sck, miso, mosi) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa5.into_alternate_af0(cs),
            gpioa.pa6.into_alternate_af0(cs),
            gpioa.pa7.into_alternate_af0(cs),
        )
    });

    let spi = Spi::spi1(
        pac_peripherals.SPI1,
        (sck, miso, mosi),
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        2400.khz(),
        &mut rcc,
    );

    let spi = spi.into_8bit_width();

    const NUM_LEDS: usize = 10;
    let mut spi = SpiWrapper::new(spi);
    let mut data = [RGB8::default(); NUM_LEDS];

    rprintln!("Ready to run.");
    loop {
        rprintln!("Looping...");
        for j in 0..(256 * 5) {
            for (i, data) in data.iter_mut().enumerate() {
                *data = wheel((((i * 256) as u16 / NUM_LEDS as u16 + j as u16) & 255) as u8);
            }

            spi.write(brightness(data.iter().cloned(), 32)).unwrap();
            delay.delay_ms(5u8);
        }
    }
}

/// A range of values within [0, 255].
#[derive(Copy, Clone, Debug)]
pub struct Range8 {
    low: u8,
    high: u8,
}

impl Range8 {
    pub fn new(low: u8, high: u8) -> Self {
        Range8 { low, high }
    }

    /// Get the "location" of a value within the rante, with 0 representing
    /// the start and 255 representing the end.
    ///
    /// Examples:
    ///
    /// ```
    ///   let r = Range{low: 100, high: 200};
    ///   r.location(100); // Some(0)   == start
    ///   r.location(150); // Some(128) == middle
    ///   r.location(200); // Some(255) == end
    ///   r.location(0);   // None      == not in range
    ///   r.location(201); // None      == not in range
    /// ```
    pub fn location(&self, value: u8) -> Option<u8> {
        if (value >= self.low) && (value <= self.high) {
            let x = (value - self.low) as u16 * 255u16;
            let range = (self.high - self.low) as u16;
            Some((x / range) as u8)
        } else {
            None
        }
    }
}

/// TRIANGLE step function that combines two linear interpolators
/// value goes up for a range and down for another
///
/// To have smooth transitions, end of up should mach start of down, however
/// this is not enforced/validated as this is used with u8 and wrapping.
fn triangle(up: Range8, down: Range8, value: u8) -> u8 {
    up.location(value)
        .or_else(|| down.location(value).map(|x| 255 - x))
        .unwrap_or(0)
}

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(w: u8) -> RGB8 {
    let r0 = Range8::new(0, 85);
    let r1 = Range8::new(85, 170);
    let r2 = Range8::new(170, 255);

    let rgb = (
        triangle(r0, r1, w),
        triangle(r1, r2, w),
        triangle(r2, r0, w),
    );

    rgb.into()
}

#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    rprintln!("OOM");
    loop {
        asm::nop();
    }
}
