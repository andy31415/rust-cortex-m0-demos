#![no_std]
#![no_main]

use core::slice::Iter;

use rand::{Rng, SeedableRng};

use embedded_hal::spi::{Mode, Phase, Polarity};
use smart_leds::brightness;

use crate::hal::delay::Delay;

use cortex_m::Peripherals;
use hal::{spi::Spi, adc::{Adc, VTemp}};
use panic_rtt_target as _;

use cortex_m_rt::entry;
use smart_leds::SmartLedsWrite;
use stm32f0xx_hal as hal;

use crate::hal::{pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};
use smart_leds::RGB8;
use ws2812_blocking_spi::Ws2812BlockingWriter;

#[derive(Debug, Clone, Copy, Default)]
struct Point {
    x: usize,
    y: usize,
}

#[derive(Debug, Clone, Copy)]
struct PanelSize {
    rows: usize,
    columns: usize,
}

/// Given a "screen" XY coordinate, transform it into an index of a flat
/// memory buffer.
trait PointIndexMapping {
    /// Converts a XY point into an index in a LED array
    fn get_coordinate(self: &Self, point: &Point) -> Option<usize>;
}

/// Represents several LED panels correncted to each other.
///
/// Connection is assumed right to left.
struct LedPanels {
    panel_size: PanelSize,
    panels: usize,
}

impl LedPanels {
    /// Construct a new structure representing several 16x16 LED panels
    /// connected together.
    fn new_16x16(panels: usize) -> Self {
        Self {
            panel_size: PanelSize {
                rows: 16,
                columns: 16,
            },
            panels,
        }
    }
}

impl PointIndexMapping for LedPanels {
    fn get_coordinate(self: &Self, point: &Point) -> Option<usize> {
        // layout of panels:
        // left to write in descending oder, so x determines the
        // actual panel
        let panel = self.panels - point.x / self.panel_size.columns - 1;

        if panel >= self.panels {
            return None;
        }

        let x = point.x % self.panel_size.columns;

        // Order is: left to right for even rows, left to right for odd rows
        Some(
            if point.y % 2 == 1 {
                self.panel_size.columns - x - 1
            } else {
                x
            } 
            // move to the right panel
            + panel * self.panel_size.rows * self.panel_size.columns
            // place on the correct column
            + self.panel_size.columns * point.y,
        )
    }
}

const SNAKE_MAX_SIZE: usize = 128;

enum Direction {
    Up, Down, Left, Right,
}

enum SnakeMoveStyle {
    Move, Grow,
}

struct SnakeMove {
    point: Point,
    style: SnakeMoveStyle,
}

struct Snake {
    pub color: RGB8,
    current_size: usize,
    data: [Point; SNAKE_MAX_SIZE],
}

impl Snake {
    fn new(position: Point, color: RGB8) -> Self {
        let mut data = [Point::default(); SNAKE_MAX_SIZE];
        data[0] = position;

        Snake {
            color,
            data,
            current_size: 1,
        }
    }

    fn iter(&self) -> Iter<Point> {
        self.data[0..self.current_size].iter()
    }

    fn get_head(&self) -> Point {
        self.data[0]
    }

    /// Replaces the current head position
    /// 
    /// Tail will get erased and then the given head_position will become 
    /// the new head
    fn move_to(&mut self, m: &SnakeMove) {
        // first determine what to do with self.data
        match m.style  {
            SnakeMoveStyle::Move => {
                // Moving involves replacing all data
                self.data.copy_within(0..self.current_size-1, 1);
            },
            SnakeMoveStyle::Grow => {
                if self.current_size < SNAKE_MAX_SIZE {
                    self.data.copy_within(0..self.current_size, 1);
                    self.current_size += 1;
                }
            }
        }

        // new head can be updated
        self.data[0] = m.point;
    }

    fn size(&self) -> usize {
        self.current_size
    }
}

fn get_new_position(snake: &Snake, direction: &Direction) -> Point {
    let head = snake.get_head();

    // compute new head
    let head = match direction {
        Direction::Up => Point{x: head.x, y: head.y + 1},
        Direction::Down => Point{x: head.x, y: head.y.wrapping_sub(1) },
        Direction::Left => Point{x: head.x.wrapping_sub(1), y: head.y},
        Direction::Right => Point{x: head.x+1, y: head.y},
    };

    // resolve wrapping

    Point {
        x: head.x % 32,
        y: head.y % 16
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

    rprintln!("SClock: {} Hz", rcc.clocks.sysclk().0);
    rprintln!("PClock: {} Hz", rcc.clocks.pclk().0);

    //rprintln!("PClock: {} Hz", pac_peripherals.)

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
        4000.khz(),
        &mut rcc,
    );

    let spi = spi.into_8bit_width();

    const NUM_LEDS: usize = 16 * 16 * 2;
    let mut spi = Ws2812BlockingWriter::new(spi);

    let panel = LedPanels::new_16x16(2);

    // red snake
    let mut snake = Snake::new(Point { x: 8, y: 8 }, [0xFF_u8, 0xFF_u8, 0_u8].into());

    let mut snake_direction = Direction::Right;

    // TODO: find a random seed
    let mut adc = Adc::new(pac_peripherals.ADC, &mut rcc);

    let mut r = 0;
    for _i in 0..64 {
        r = r << 1;
        if VTemp::read(&mut adc, None) & 0x01 == 1 {
            r |= 1;
        }
    }

    rprintln!("Random seed: 0x{:08X}", r);
    let mut rnd = rand_chacha::ChaChaRng::seed_from_u64(r);

    rprintln!("Ready to run.");
    loop {
        let point = get_new_position(&snake, &snake_direction);
        snake.move_to(&SnakeMove{
            point,
            style: if snake.size() < 5 {SnakeMoveStyle::Grow } else {SnakeMoveStyle::Move }
        });


        // Implementing a tur
        match rnd.gen_range(0..10) {
            0 => {
                // Turn left
                snake_direction = match snake_direction {
                    Direction::Up => Direction::Left,
                    Direction::Left => Direction::Down,
                    Direction::Down => Direction::Right,
                    Direction::Right => Direction::Up,
                };
            }
            1 => {
                // Turn right
                snake_direction = match snake_direction {
                    Direction::Up => Direction::Right,
                    Direction::Left => Direction::Up,
                    Direction::Down => Direction::Left,
                    Direction::Right => Direction::Down,
                };
            }
            _ => { /* keep direction */}
        }



        // display snake
        let mut data = [RGB8::default(); NUM_LEDS];
        for point in snake.iter() {
            if let Some(idx) = panel.get_coordinate(point) {
                data[idx] = snake.color;
            } else {
                rprintln!("Error for index {:?}", point);
            }
        }
        spi.write(brightness(data.iter().cloned(), 10)).unwrap();

        delay.delay_ms(70_u32);
    }
}
