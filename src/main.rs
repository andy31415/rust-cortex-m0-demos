#![no_std]
#![no_main]

use core::slice::Iter;

use rand::{Rng, SeedableRng};

use embedded_hal::spi::{Mode, Phase, Polarity};
use rand_chacha::ChaCha20Rng;
use smart_leds::brightness;

use crate::hal::delay::Delay;

use cortex_m::Peripherals;
use hal::{
    adc::{Adc, VTemp},
    spi::Spi,
};
use panic_rtt_target as _;

use cortex_m_rt::entry;
use smart_leds::SmartLedsWrite;
use stm32f0xx_hal as hal;

use crate::hal::{pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};
use smart_leds::RGB8;
use ws2812_blocking_spi::Ws2812BlockingWriter;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
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

const NUM_LEDS: usize = 16 * 16 * 2;
const SNAKE_MAX_SIZE: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Direction {
    Up,
    Down,
    Left,
    Right,
}

enum SnakeMoveStyle {
    Move,
    Grow,
}

struct SnakeMove {
    point: Point,
    style: SnakeMoveStyle,
}

trait SnakeController {
    fn next_move(&mut self, snake: &Snake) -> SnakeMove;
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
        match m.style {
            SnakeMoveStyle::Move => {
                // Moving involves replacing all data
                self.data.copy_within(0..self.current_size - 1, 1);
            }
            SnakeMoveStyle::Grow if self.current_size >= SNAKE_MAX_SIZE => {
                self.data.copy_within(0..self.current_size - 1, 1);
            }
            SnakeMoveStyle::Grow => {
                self.data.copy_within(0..self.current_size, 1);
                self.current_size += 1;
            }
        }

        // new head can be updated
        self.data[0] = m.point;
    }

    fn move_step<CONTROLLER: SnakeController>(&mut self, controller: &mut CONTROLLER) {
        self.move_to(&controller.next_move(self));
    }


    fn size(&self) -> usize {
        self.current_size
    }
}

struct TestSnakeMover {
    direction: Direction,
    rnd: ChaCha20Rng
}

impl TestSnakeMover {
    fn new(direction: Direction, random_seed: u64) -> Self {
        Self {
            direction,
            rnd: rand_chacha::ChaChaRng::seed_from_u64(random_seed),
        }
    }
}

impl SnakeController for TestSnakeMover {

    fn next_move(&mut self, snake: &Snake) ->SnakeMove {
        let head = snake.get_head();

        // pick a random direction to turn to:
        self.direction = match self.rnd.gen_range(0_i32..10_i32) {
            0 => {
                // Turn left
                match self.direction {
                    Direction::Up => Direction::Left,
                    Direction::Left => Direction::Down,
                    Direction::Down => Direction::Right,
                    Direction::Right => Direction::Up,
                }
            }
            1 => {
                // Turn right
                match self.direction {
                    Direction::Up => Direction::Right,
                    Direction::Left => Direction::Up,
                    Direction::Down => Direction::Left,
                    Direction::Right => Direction::Down,
                }
            }
            _ => self.direction, // no turn
        };

        // compute new head
    let head = match self.direction {
        Direction::Up => Point {
            x: head.x,
            y: head.y + 1,
        },
        Direction::Down => Point {
            x: head.x,
            y: head.y.wrapping_sub(1),
        },
        Direction::Left => Point {
            x: head.x.wrapping_sub(1),
            y: head.y,
        },
        Direction::Right => Point {
            x: head.x + 1,
            y: head.y,
        },
    };

        // resolve wrapping

        let point = Point {
            x: head.x % 32,
            y: head.y % 16,
        };

        SnakeMove {
            point,
            style: if snake.size() < 15 {
                SnakeMoveStyle::Grow
            } else {
                SnakeMoveStyle::Move
            },
        }
    }
}

struct Game {
    snake_a: Snake,
    snake_a_controller: TestSnakeMover,

    snake_b: Snake,
    snake_b_controller: TestSnakeMover,
}

#[derive(PartialEq)]
enum Collision {
    None,
    Collision(Point),
}

impl Game {
    fn new(random_seed: u64) -> Self {
        let mut rng = rand_chacha::ChaChaRng::seed_from_u64(random_seed);
        Self {
            snake_a: Snake::new(Point { x: 8, y: 8 }, [0xFF_u8, 0xFF_u8, 0_u8].into()),
            snake_a_controller: TestSnakeMover::new(Direction::Right, rng.gen()),

            snake_b: Snake::new(Point { x: 25, y: 8 }, [0_u8, 0xFF_u8, 0xFF_u8].into()),
            snake_b_controller: TestSnakeMover::new(Direction::Left, rng.gen()),
        }
    }

    fn display<MAPPER: PointIndexMapping>(&self, panel: &MAPPER, data: &mut [RGB8; NUM_LEDS]) {
        for point in self.snake_a.iter() {
            if let Some(idx) = panel.get_coordinate(point) {
                data[idx] = self.snake_a.color;
            } else {
                rprintln!("Error for index {:?}", point);
            }
        }

        for point in self.snake_b.iter() {
            if let Some(idx) = panel.get_coordinate(point) {
                data[idx] = self.snake_b.color;
            } else {
                rprintln!("Error for index {:?}", point);
            }
        }
    }

    fn check_collision(&self) -> Collision {
        let head = self.snake_a.get_head();

        for tail_point in self.snake_a.iter().skip(1) {
            if head == *tail_point {
                return Collision::Collision(head)
            }
        }
        for tail_point in self.snake_b.iter() {
            if head == *tail_point {
                return Collision::Collision(head)
            }
        }

        let head = self.snake_b.get_head();
        
        for tail_point in self.snake_a.iter() {
            if head == *tail_point {
                return Collision::Collision(head)
            }
        }
        for tail_point in self.snake_b.iter().skip(1) {
            if head == *tail_point {
                return Collision::Collision(head)
            }
        }

        Collision::None
    }

    fn step(&mut self) {
        self.snake_a.move_step(&mut self.snake_a_controller);
        self.snake_b.move_step(&mut self.snake_b_controller);
    }


    fn reset(&mut self) {
       self.snake_a = Snake::new(Point { x: 8, y: 8 }, [0xFF_u8, 0xFF_u8, 0_u8].into());
       self.snake_b = Snake::new(Point { x: 25, y: 8 }, [0_u8, 0xFF_u8, 0xFF_u8].into());
    }
}

fn poor_random(adc: &mut Adc) -> u64 {
    let mut r = 0_u64;
    for _i in 0..64 {
        r = r << 1;
        if VTemp::read(adc, None) & 0x01 == 1 {
            r |= 1;
        }
    }
    r
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

    let mut spi = Ws2812BlockingWriter::new(spi);

    let panel = LedPanels::new_16x16(2);
    let mut adc = Adc::new(pac_peripherals.ADC, &mut rcc);
    let random_seed = poor_random(&mut adc);

    let mut game = Game::new(random_seed);

    rprintln!("Ready to run.");
    loop {
        game.step();

        // display snake
        let mut data = [RGB8::default(); NUM_LEDS];

        game.display(&panel, &mut data);

        let collision = game.check_collision();
        if let Collision::Collision(p) = collision {
           if let Some(idx) = panel.get_coordinate(&p) {
              data[idx] = [0xFF_u8, 0_u8, 0_u8].into();
           }
        }
        
        spi.write(brightness(data.iter().cloned(), 10)).unwrap();
        delay.delay_ms(70_u32);

        if collision != Collision::None {
            delay.delay_ms(2000_u32);
            game.reset();
        }
    }
}
