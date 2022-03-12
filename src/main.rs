#![no_std]
#![no_main]

use cortex_m::Peripherals;
use panic_rtt_target as _;

use cortex_m_rt::entry;
use stm32f0xx_hal as hal;

use crate::hal::{delay::Delay, i2c, pac, prelude::*};

use aht10::AHT10;
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
    let gpio = pac_peripherals.GPIOB.split(&mut rcc);

    let i2c = i2c::I2c::i2c1(
        pac_peripherals.I2C1,
        cortex_m::interrupt::free(move |cs| {
            (
                gpio.pb6.into_alternate_af1(cs),
                gpio.pb7.into_alternate_af1(cs),
            )
        }),
        100.khz(),
        &mut rcc,
    );

    let maybe_aht10 = AHT10::new(i2c, delay.clone());
    if let Err(err) = maybe_aht10 {
        rprintln!("I2C error: {:?}", err);
        loop {}
    }
    let mut aht10 = maybe_aht10.unwrap();

    loop {
        if let Ok((humidity, temp)) = aht10.read() {
            rprintln!(
                "Temperature {}C humidity {}%.",
                temp.celsius(),
                humidity.rh()
            );
            rprintln!(
                "RAW Temperature {} humidity {}.",
                temp.raw(),
                humidity.raw()
            );
        } else {
            rprintln!("Read error.");
        }
        delay.delay_ms(1000_u16);
    }
}
