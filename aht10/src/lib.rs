//! A platform agnostic driver to interface with the AHT10 temperature and humidity sensor.
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! Unfortunately, the AHT10 datasheet is somewhat underspecified. There's a
//! FIFO mode as well as command data bytes which are briefly mentioned, but
//! I've found no documentation describing what they mean. Caveat emptor.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2

#![deny(missing_docs)]
#![no_std]

extern crate embedded_hal as hal;

use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};

const I2C_ADDRESS: u8 = 0x38;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
#[repr(u8)]
enum Command {
    Calibrate = 0b1110_0001,
    GetRaw = 0b1010_1000,
    GetCT = 0b1010_1100,
    Reset = 0b1011_1010,
}

#[macro_use]
extern crate bitflags;

bitflags! {
    struct StatusFlags: u8 {
        const BUSY = (1 << 7);
        const MODE = ((1 << 6) | (1 << 5));
        const CRC = (1 << 4);
        const CALIBRATION_ENABLE = (1 << 3);
        const FIFO_ENABLE = (1 << 2);
        const FIFO_FULL = (1 << 1);
        const FIFO_EMPTY = (1 << 0);
    }
}

/// AHT10 Error
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Device is not calibrated
    Uncalibrated(),
    /// Underlying bus error.
    BusError(E),
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::BusError(e)
    }
}

/// AHT10 driver
pub struct AHT10<I2C, D> {
    i2c: I2C,
    delay: D,
}

/// Humidity reading from AHT10.
pub struct Humidity {
    h: u32,
}
impl Humidity {
    /// Humidity conveted to relative humidity.
    pub fn rh(&self) -> f32 {
        100.0 * (self.h as f32) / ((1 << 20) as f32)
    }
    /// Raw humidity reading.
    pub fn raw(&self) -> u32 {
        self.h
    }
}

/// Temperature reading from AHT10.
pub struct Temperature {
    t: u32,
}
impl Temperature {
    /// Temperature converted to celsius.
    pub fn celsius(&self) -> f32 {
        (200.0 * (self.t as f32) / ((1 << 20) as f32)) - 50.0
    }
    /// Raw temperature reading.
    pub fn raw(&self) -> u32 {
        self.t
    }
}

impl<I2C, D, E> AHT10<I2C, D>
where
    I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>,
    D: DelayMs<u16>,
{
    /// Creates a new AHT10 device from an I2C peripheral.
    pub fn new(i2c: I2C, delay: D) -> Result<Self, Error<E>> {
        let mut dev = AHT10 { i2c, delay };

        dev.i2c.write(I2C_ADDRESS, &[Command::Reset as u8])?;
        dev.delay.delay_ms(20);

        // MSB notes:
        // Bit 2 set => temperature is roughly doubled(?)
        // Bit 3 set => calibrated flag
        // Bit 4 => temperature is negative? (cyc mode?)
        dev.write_cmd(Command::Calibrate, 0x0800)?;
        dev.wait_to_be_ready()?;

        Ok(dev)
    }

    // Read a value from the sensor as the 'current status'.
    fn read_status(&mut self) -> Result<StatusFlags, E> {
        let buf: &mut [u8; 1] = &mut [0; 1];
        self.i2c.read(I2C_ADDRESS, buf)?;
        let status = StatusFlags { bits: buf[0] };
        Ok(status)
    }

    fn wait_to_be_ready(&mut self) -> Result<(), Error<E>> {
        loop {
            let status = self.read_status()?;
            if status.contains(StatusFlags::BUSY) {
                self.delay.delay_ms(10);
                continue;
            }

            if !status.contains(StatusFlags::CALIBRATION_ENABLE) {
                return Err(Error::Uncalibrated());
            }
            break;
        }
        Ok(())
    }

    /// Soft reset the sensor.
    pub fn reset(&mut self) -> Result<(), E> {
        self.write_cmd(Command::Reset, 0)?;
        self.delay.delay_ms(20);
        Ok(())
    }

    /// Read humidity and temperature.
    pub fn read(&mut self) -> Result<(Humidity, Temperature), Error<E>> {
        self.i2c
            .write(I2C_ADDRESS, &[Command::GetCT as u8, 0x33, 0])?;

        self.wait_to_be_ready()?;

        let buf: &mut [u8; 6] = &mut [0; 6];
        self.i2c.read(I2C_ADDRESS, buf)?;

        let hum = ((buf[1] as u32) << 12) | ((buf[2] as u32) << 4) | ((buf[3] as u32) >> 4);
        let temp = (((buf[3] as u32) & 0x0f) << 16) | ((buf[4] as u32) << 8) | (buf[5] as u32);
        Ok((Humidity { h: hum }, Temperature { t: temp }))
    }

    fn write_cmd(&mut self, cmd: Command, dat: u16) -> Result<(), E> {
        self.i2c.write(
            I2C_ADDRESS,
            &[cmd as u8, (dat >> 8) as u8, (dat & 0xff) as u8],
        )
    }
}
