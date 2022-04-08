#![no_std]
#![no_main]

use cortex_m::Peripherals;
use embedded_hal::adc::Channel;
use hal::adc::Adc;
use panic_rtt_target as _;

use cortex_m_rt::entry;
use stm32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*};

use rtt_target::{rprintln, rtt_init_print};

struct AdcReading<'a, CHANNEL> {
    pin: CHANNEL,
    name: &'a str,
    known_value: Option<u16>, // known value for ADc
    sensitivity_mv: u16,      // after how big of a delta to report a new value
}

impl<'a, CHANNEL> AdcReading<'a, CHANNEL>
where
    CHANNEL: Channel<Adc, ID = u8>,
{
    fn new(pin: CHANNEL, name: &'a str, sensitivity_mv: u16) -> Self {
        Self {
            pin,
            name,
            known_value: None,
            sensitivity_mv: sensitivity_mv,
        }
    }

    fn update(&mut self, adc: &mut Adc) {
        let current = adc.read_abs_mv(&mut self.pin);

        let should_update = match self.known_value {
            None => true,
            Some(known) if known > current => known - current >= self.sensitivity_mv,
            Some(known) => current - known >= self.sensitivity_mv,
        };

        if should_update {
            self.known_value = Some(current);
            rprintln!("{} = {}", self.name, current);
        }
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let mut pac_peripherals = pac::Peripherals::take().unwrap();

    let mut rcc = pac_peripherals
        .RCC
        .configure()
        .sysclk(48.mhz())
        .freeze(&mut pac_peripherals.FLASH);

    let gpio = pac_peripherals.GPIOA.split(&mut rcc);
    let (a0, a1) =
        cortex_m::interrupt::free(move |cs| (gpio.pa0.into_analog(cs), gpio.pa1.into_analog(cs)));

    rprintln!("Initialized. Ready to display values.");

    let mut adc = Adc::new(pac_peripherals.ADC, &mut rcc);
    adc.set_precision(hal::adc::AdcPrecision::B_12);

    let mut r0 = AdcReading::new(a0, "A0", 50);
    let mut r1 = AdcReading::new(a1, "A1", 50);

    loop {
        r0.update(&mut adc);
        r1.update(&mut adc);
    }
}
