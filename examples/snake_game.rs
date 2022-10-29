#![no_std]
#![no_main]

use core::slice::Iter;

use rand::{Rng, SeedableRng};

use embedded_hal::{
    adc::Channel,
    spi::{Mode, Phase, Polarity},
};
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

impl Point {
    fn is_higher_than(&self, other: &Point) -> bool {
        self.y > other.y
    }

    fn is_lower_than(&self, other: &Point) -> bool {
        self.y < other.y
    }

    fn is_left_of(&self, other: &Point) -> bool {
        self.x < other.x
    }

    fn is_right_of(&self, other: &Point) -> bool {
        self.x > other.x
    }
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

impl Direction {
    fn left(&self) -> Direction {
        match self {
            Direction::Up => Direction::Left,
            Direction::Left => Direction::Down,
            Direction::Down => Direction::Right,
            Direction::Right => Direction::Up,
        }
    }

    fn right(&self) -> Direction {
        match self {
            Direction::Up => Direction::Right,
            Direction::Right => Direction::Down,
            Direction::Down => Direction::Left,
            Direction::Left => Direction::Up,
        }
    }
}

enum SnakeMoveStyle {
    Move,
    Grow,
}

struct SnakeMove {
    point: Point,
    style: SnakeMoveStyle,
}

/// Something the snake can eat
enum Powerup {
    Apple, // makes the snake grow
}

trait SnakeAiInfo {
    // where should the AI head to
    fn goal(&self) -> Point;

    // will it collide
    fn will_collide(&self, point: &Point) -> bool;
}

trait SnakeController {
    fn next_move(&mut self, snake: &Snake) -> SnakeMove;

    fn step(&mut self, snake: &Snake, ai_info: &dyn SnakeAiInfo, adc: &mut Adc);

    fn reset(&mut self);

    fn eat(&mut self, powerup: Powerup);
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

struct ComputerSnakeMover {
    start_direction: Direction,
    direction: Direction,
    rnd: ChaCha20Rng,
    target_size: usize,
}

impl ComputerSnakeMover {
    fn new(direction: Direction, random_seed: u64) -> Self {
        Self {
            direction,
            start_direction: direction,
            rnd: rand_chacha::ChaChaRng::seed_from_u64(random_seed),
            target_size: 5,
        }
    }

    fn turn_towards(&mut self, head: &Point, goal: &Point) -> Direction {
        let is_good_direction = |d: Direction| -> bool {
            ((d == Direction::Left) && goal.is_left_of(head))
                || ((d == Direction::Right) && goal.is_right_of(head))
                || ((d == Direction::Up) && goal.is_higher_than(head))
                || ((d == Direction::Down) && goal.is_lower_than(head))
        };

        // prefer to stay on the path
        if is_good_direction(self.direction) {
            return self.direction;
        }

        // if not a good direction, Exactly one of the remaining ones should be better
        // unless we have to turn around
        if is_good_direction(self.direction.left()) {
            return self.direction.left();
        } else if is_good_direction(self.direction.right()) {
            return self.direction.right();
        }

        // have to turn around. Pick randomly
        if self.rnd.gen_ratio(1, 2) {
            self.direction.left()
        } else {
            self.direction.right()
        }
    }
}

fn next_point(point: &Point, direction: Direction) -> Point {
    let point = match direction {
        Direction::Up => Point {
            x: point.x,
            y: point.y + 1,
        },
        Direction::Down => Point {
            x: point.x,
            y: point.y.wrapping_sub(1),
        },
        Direction::Left => Point {
            x: point.x.wrapping_sub(1),
            y: point.y,
        },
        Direction::Right => Point {
            x: point.x + 1,
            y: point.y,
        },
    };

    Point {
        x: point.x % 32,
        y: point.y % 16,
    }
}

impl SnakeController for ComputerSnakeMover {
    fn eat(&mut self, powerup: Powerup) {
        match powerup {
            Powerup::Apple => self.target_size += 4,
        }
    }

    fn reset(&mut self) {
        self.direction = self.start_direction;
        self.target_size = 5;
    }

    fn step(&mut self, snake: &Snake, ai_data: &dyn SnakeAiInfo, _adc: &mut Adc) {
        // TODO: are we heading towards an apple
        //       are we coliding?

        let mut target = self.direction;

        let head = snake.get_head();

        // HIGH chance to move towards apple
        if self.rnd.gen_ratio(8, 10) {
            target = self.turn_towards(&head, &ai_data.goal());

            if target != self.direction {
                rprintln!(
                    "Turned from {:?} to {:?} (heading from {:?} to {:?}",
                    self.direction,
                    target,
                    head,
                    ai_data.goal()
                );
            }
        }

        if ai_data.will_collide(&next_point(&head, target)) {
            // try one of keep direction or turn to avoid hitting something
            rprintln!("Avoiding collision");

            if !ai_data.will_collide(&next_point(&head, self.direction)) {
                rprintln!(" => Keeping direction");
                target = self.direction;
            } else {
                let choices = if self.rnd.gen_ratio(1, 2) {
                    [self.direction.left(), self.direction.right()]
                } else {
                    [self.direction.right(), self.direction.left()]
                };

                if !ai_data.will_collide(&next_point(&head, choices[0])) {
                    rprintln!(" => Turn {:?}", choices[0]);
                    target = choices[0];
                } else if !ai_data.will_collide(&next_point(&head, choices[1])) {
                    rprintln!(" => Turn {:?}", choices[1]);
                    target = choices[1];
                } else {
                    rprintln!(" => CANNOT AVOID!");
                    target = self.direction; // we will colide, just keep going straight
                }
            }
        }

        self.direction = target;
    }

    fn next_move(&mut self, snake: &Snake) -> SnakeMove {
        let point = next_point(&snake.get_head(), self.direction);

        SnakeMove {
            point,
            style: if snake.size() < self.target_size {
                SnakeMoveStyle::Grow
            } else {
                SnakeMoveStyle::Move
            },
        }
    }
}

struct JoystickMover<UdPin, LrPin> {
    start_direction: Direction,
    direction: Direction,
    ud_pin: UdPin,
    lr_pin: LrPin,
    target_size: usize,
}

impl<UdPin, LrPin> JoystickMover<UdPin, LrPin> {
    fn new(direction: Direction, lr_pin: LrPin, ud_pin: UdPin) -> Self {
        Self {
            start_direction: direction,
            direction,
            ud_pin,
            lr_pin,
            target_size: 5,
        }
    }
}

impl<UdPin, LrPin> SnakeController for JoystickMover<UdPin, LrPin>
where
    UdPin: Channel<Adc, ID = u8>,
    LrPin: Channel<Adc, ID = u8>,
{
    fn reset(&mut self) {
        self.direction = self.start_direction;
        self.target_size = 5;
    }

    fn step(&mut self, _snake: &Snake, _ai_data: &dyn SnakeAiInfo, adc: &mut Adc) {
        let ud = adc.read_abs_mv(&mut self.ud_pin);
        let lr = adc.read_abs_mv(&mut self.lr_pin);

        // find out which axes has the max deviation from 3.3v
        // == 1650 mv
        const MID_V: u16 = 1650;
        const DEADZONE: u16 = 500; // MV to conside deadzone

        let lr_diff = if lr > MID_V { lr - MID_V } else { MID_V - lr };
        let ud_diff = if ud > MID_V { ud - MID_V } else { MID_V - ud };

        let direction = if lr_diff > ud_diff {
            if lr_diff <= DEADZONE {
                None
            } else if lr < MID_V {
                Some(Direction::Left)
            } else {
                Some(Direction::Right)
            }
        } else {
            if ud_diff <= DEADZONE {
                None
            } else if ud < MID_V {
                Some(Direction::Down)
            } else {
                Some(Direction::Up)
            }
        };

        match direction {
            Some(Direction::Left) if self.direction != Direction::Right => {
                self.direction = direction.unwrap()
            }
            Some(Direction::Right) if self.direction != Direction::Left => {
                self.direction = direction.unwrap()
            }
            Some(Direction::Up) if self.direction != Direction::Down => {
                self.direction = direction.unwrap()
            }
            Some(Direction::Down) if self.direction != Direction::Up => {
                self.direction = direction.unwrap()
            }
            _ => {}
        }
    }

    fn next_move(&mut self, snake: &Snake) -> SnakeMove {
        let point = next_point(&snake.get_head(), self.direction);
        SnakeMove {
            point,
            style: if snake.size() < self.target_size {
                SnakeMoveStyle::Grow
            } else {
                SnakeMoveStyle::Move
            },
        }
    }

    fn eat(&mut self, powerup: Powerup) {
        match powerup {
            Powerup::Apple => self.target_size += 4,
        }
    }
}

const APPLE_COLOR: RGB8 = RGB8 {
    r: 0xFF_u8,
    g: 0_u8,
    b: 0x20_u8,
};

struct Game<ControllerA, ControllerB, GameRng> {
    rng: GameRng,
    snake_a: Snake,
    snake_a_controller: ControllerA,

    snake_b: Snake,
    snake_b_controller: ControllerB,

    apple: Point,
}

struct GameAiInfo<'a> {
    apple: Point,
    snake_a: &'a Snake,
    snake_b: &'a Snake,
}

impl SnakeAiInfo for GameAiInfo<'_> {
    fn goal(&self) -> Point {
        self.apple
    }

    fn will_collide(&self, point: &Point) -> bool {
        self.snake_a
            .iter()
            .chain(self.snake_b.iter())
            .find(|&&x| x == *point)
            .is_some()
    }
}

#[derive(PartialEq)]
enum Collision {
    None,
    Collision(Point),
}

impl<ControllerA, ControllerB, GameRng> Game<ControllerA, ControllerB, GameRng>
where
    ControllerA: SnakeController,
    ControllerB: SnakeController,
    GameRng: Rng,
{
    fn new(snake_a_controller: ControllerA, snake_b_controller: ControllerB, rng: GameRng) -> Self {
        let mut result = Self {
            rng,

            snake_a: Snake::new(Point { x: 8, y: 8 }, [0xFF_u8, 0xFF_u8, 0_u8].into()),
            snake_a_controller: snake_a_controller,

            snake_b: Snake::new(Point { x: 25, y: 8 }, [0_u8, 0xFF_u8, 0xFF_u8].into()),
            snake_b_controller: snake_b_controller,

            apple: Point { x: 0, y: 0 },
        };

        result.new_apple(); // position 0/0 is not ideal

        result
    }

    fn new_apple(&mut self) {
        loop {
            let pick = Point {
                x: self.rng.gen_range(0..32),
                y: self.rng.gen_range(0..16),
            };

            if !self.collides(pick) {
                self.apple = pick;
                break;
            }
        }
    }

    fn collides(&self, point: Point) -> bool {
        self.snake_a
            .iter()
            .chain(self.snake_b.iter())
            .find(|&&x| x == point)
            .is_some()
    }

    fn display<MAPPER: PointIndexMapping>(&self, panel: &MAPPER, data: &mut [RGB8; NUM_LEDS]) {
        for point in self.snake_a.iter() {
            data[panel.get_coordinate(point).unwrap()] = self.snake_a.color;
        }

        for point in self.snake_b.iter() {
            data[panel.get_coordinate(point).unwrap()] = self.snake_b.color;
        }

        data[panel.get_coordinate(&self.apple).unwrap()] = APPLE_COLOR;
    }

    fn check_collision(&self) -> Collision {
        let head = self.snake_a.get_head();

        for tail_point in self.snake_a.iter().skip(1) {
            if head == *tail_point {
                return Collision::Collision(head);
            }
        }
        for tail_point in self.snake_b.iter() {
            if head == *tail_point {
                return Collision::Collision(head);
            }
        }

        let head = self.snake_b.get_head();

        for tail_point in self.snake_a.iter() {
            if head == *tail_point {
                return Collision::Collision(head);
            }
        }
        for tail_point in self.snake_b.iter().skip(1) {
            if head == *tail_point {
                return Collision::Collision(head);
            }
        }

        Collision::None
    }

    fn step(&mut self, adc: &mut Adc) {
        let ai_info = GameAiInfo {
            apple: self.apple,
            snake_a: &self.snake_a,
            snake_b: &self.snake_b,
        };

        self.snake_a_controller.step(&self.snake_a, &ai_info, adc);
        self.snake_b_controller.step(&self.snake_b, &ai_info, adc);

        self.snake_a.move_step(&mut self.snake_a_controller);
        self.snake_b.move_step(&mut self.snake_b_controller);

        // check if an apple was eaten
        if self.snake_a.get_head() == self.apple {
            self.snake_a_controller.eat(Powerup::Apple);
            self.new_apple()
        } else if self.snake_b.get_head() == self.apple {
            self.snake_b_controller.eat(Powerup::Apple);
            self.new_apple()
        }
    }

    fn reset(&mut self) {
        self.snake_a = Snake::new(Point { x: 8, y: 7 }, [0xFF_u8, 0xFF_u8, 0_u8].into());
        self.snake_b = Snake::new(Point { x: 25, y: 9 }, [0_u8, 0xFF_u8, 0xFF_u8].into());

        self.snake_a_controller.reset();
        self.snake_b_controller.reset();

        self.new_apple();
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

    let mut delay = Delay::new(cortex_peripherals.SYST, &rcc);

    let gpioa = pac_peripherals.GPIOA.split(&mut rcc);

    let (sck, miso, mosi, a0, a1, a2, a3) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa5.into_alternate_af0(cs),
            gpioa.pa6.into_alternate_af0(cs),
            gpioa.pa7.into_alternate_af0(cs),
            // Analoog bits
            gpioa.pa0.into_analog(cs),
            gpioa.pa1.into_analog(cs),
            gpioa.pa2.into_analog(cs),
            gpioa.pa3.into_analog(cs),
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

    let controller_a = JoystickMover::new(Direction::Right, a0, a1);
    let controller_b = JoystickMover::new(Direction::Left, a2, a3);
    //let controller_a = ComputerSnakeMover::new(Direction::Right, poor_random(&mut adc));
    //let controller_b = ComputerSnakeMover::new(Direction::Left, poor_random(&mut adc));

    let mut game = Game::new(
        controller_a,
        controller_b,
        rand_chacha::ChaChaRng::seed_from_u64(poor_random(&mut adc)),
    );

    rprintln!("Ready to run.");
    loop {
        game.step(&mut adc);

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
