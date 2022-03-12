# Repository information

A Program demo running on a Cortex-M0.
Sample boards available for example at 
[AliExpress search for STM32F030C8T6](https://www.aliexpress.com/wholesale?SearchText=stm32f030c8t6)

Extra hardware:
  - A SSD1306 screen, eg:
     - on [AliExpress SSD1306 OLED](https://www.aliexpress.com/wholesale?SearchText=ssd1306+oled+128x64)
     - [Google search](https://www.google.com/search?q=ssd1306+oled+128x64)

# Current branch information

This example builds upon the starter setup and adds:
  - SSD1306 display when the I2C display is connected to pins B6 and B7.
    - **NOTE** requires `--release` builds on CortexM0 due to board having little flash
  - Example on how to emable HEAP support in embedded rust. 
    - **NOTE** requires `rustup default nightly` currently.

# Project base

This project is based on the [cortex-m-quickstart](https://github.com/rust-embedded/cortex-m-quickstart)
example changed to compile for a `thumbv6m-none-eabi` target.

## Dependencies

To build embedded programs using this template you'll need:

``` console
$ rustup target add thumbv6m-none-eabi
$ cargo install probe-run
```

## More information

More information found at
  - [cortex-m-quickstart](https://github.com/rust-embedded/cortex-m-quickstart) documentation
  - [The embedded rust book](https://rust-embedded.github.io/book)
  - My example [video series](https://www.youtube.com/c/AndreiLitvinCa/videos) on Youtube
