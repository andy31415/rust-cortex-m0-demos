# Repository information

A Program demo running on a Cortex-M0.
Sample boards available for example at 
[AliExpress search for STM32F030C8T6](https://www.aliexpress.com/wholesale?SearchText=stm32f030c8t6)

# Current branch information

This is using PWM and driving a servo. Extra hardware used in the example:
  - A logic level shifter since servos require 5V. Sample [search](https://www.google.com/search?q=bidirectional+shifter+BSS138)
    for a BSS138 shifter
  - A SG90 Servo. See its [datasheet](https://www.google.com/search?q=SG90+datasheet)

Servo is connectedd to the A8 pin (through the logic level shifter).

Corresponding video: https://www.youtube.com/watch?v=fgChU18a5vs

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
