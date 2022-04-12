#![no_std]
#![no_main]

use crate::hal::spi::Mode;
use cortex_m::Peripherals;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::ascii::FONT_6X9;
use embedded_graphics::mono_font::iso_8859_2::FONT_4X6;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::OriginDimensions;
use embedded_graphics::prelude::Point;
use embedded_graphics::prelude::Size;
use embedded_graphics::primitives::Circle;
use embedded_graphics::Pixel;
use embedded_graphics::{
    mono_font::MonoTextStyle,
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Triangle},
    text::{Alignment, Text},
};
use panic_rtt_target as _;
use smart_leds_trait::SmartLedsWrite;
use smart_leds_trait::RGB8;
use stm32f0xx_hal::spi::Spi;

use cortex_m_rt::entry;
use stm32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};

use embedded_hal::blocking::spi::Write;
use embedded_hal::spi::{Phase, Polarity};
use ws2812_blocking_spi::Ws2812BlockingWriter;

struct LedDisplay<SPI: Write<u8>> {
    writer: Ws2812BlockingWriter<SPI>,
    frame_buffer: [RGB8; 16 * 16],
}

enum DrawError {
    OutOfBounds,
}

impl<SPI: Write<u8>> LedDisplay<SPI> {
    fn new(writer: Ws2812BlockingWriter<SPI>) -> Self {
        Self {
            writer,
            frame_buffer: [RGB8::default(); 16 * 16],
        }
    }

    fn flush(&mut self) {
        self.writer.write(self.frame_buffer.iter().cloned()).ok();
    }

    fn index_top_left(&self, x: i32, y: i32) -> Result<usize, DrawError> {
        if (x < 0) || (x >= 16) || (y < 0) || (y >= 16) {
            return Err(DrawError::OutOfBounds);
        }
        let x = x as usize;
        let y = y as usize;

        // PIXEL LAYOUT:
        //   - bottom to top
        //   - odd rows left to right, even rows right to left (if going from the)
        //     top, odd/even are inverted

        // place it to the right row (this will be the first/last LED depending
        // on pairity)
        let index = (15 - y) * 16 + if y % 2 == 1 { x } else { 15 - x };
        Ok(index)
    }
}

impl<SPI: Write<u8>> OriginDimensions for LedDisplay<SPI> {
    fn size(&self) -> Size {
        Size::new(16, 16)
    }
}

impl<SPI: Write<u8>> DrawTarget for LedDisplay<SPI> {
    type Color = Rgb888;
    type Error = DrawError;

    fn draw_iter<I>(&mut self, data: I) -> Result<(), DrawError>
    where
        I: IntoIterator<Item = Pixel<Rgb888>>,
    {
        for Pixel(point, color) in data {
            let idx = self.index_top_left(point.x, point.y)?;
            self.frame_buffer[idx] = [color.r(), color.g(), color.b()].into();
        }

        Ok(())
    }

    fn clear(&mut self, color: Rgb888) -> Result<(), DrawError> {
        self.frame_buffer
            .fill([color.r(), color.g(), color.b()].into());
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

    let writer = Ws2812BlockingWriter::new(spi);

    let mut display = LedDisplay::new(writer);

    rprintln!("Ready to run!");

    {
        /*
        let border_stroke = PrimitiveStyleBuilder::new()
            .stroke_color(Rgb888::new(30, 0, 0))
            .stroke_width(2)
            .stroke_alignment(StrokeAlignment::Inside)
            .build();

        display
            .bounding_box()
            .into_styled(border_stroke)
            .draw(&mut display)
            .ok();
            */

        /*
                let yoffset = 2;
                let side = 6;
        let thin_stroke = PrimitiveStyle::with_stroke(Rgb888::new(0, 30, 0), 1);

                // Draw a triangle.
                Triangle::new(
                    Point::new(side, side + yoffset),
                    Point::new(side + side, side + yoffset),
                    Point::new(side + side, yoffset),
                )
                .into_styled(thin_stroke)
                .draw(&mut display)
                .ok();
        */

        /*
        Rectangle::new(Point::new(0, 0), Size::new(3, 3))
            .into_styled(PrimitiveStyle::with_fill(Rgb888::new(0, 33, 33)))
            .draw(&mut display)
            .ok();
        */

        // Draw an R
    }

    let font_style = MonoTextStyleBuilder::new()
        .font(&FONT_4X6)
        .text_color(Rgb888::new(0, 22, 0))
        .underline()
        .build();

    let circle_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb888::new(0, 33, 33))
        .stroke_color(Rgb888::new(22, 0, 0))
        .stroke_width(1)
        .stroke_alignment(StrokeAlignment::Outside)
        .build();

    display.flush();

    let mut circle_x = 1;
    let mut circle_move = 1;

    loop {
        if circle_x <= 1 {
            circle_move = 1;
        } else if circle_x >= 10 {
            circle_move = -1;
        }
        circle_x += circle_move;

        display.clear(Rgb888::BLACK).ok();

        Text::with_alignment("Rust", Point::new(0, 6), font_style, Alignment::Left)
            .draw(&mut display)
            .ok();

        Circle::new(Point::new(circle_x, 10), 5)
            .into_styled(circle_style)
            .draw(&mut display)
            .ok();
        display.flush();

        delay.delay_ms(100_u16);
    }
}
