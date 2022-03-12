# Repository information

A Program demo running on a Cortex-M0.
Sample boards available for example at 
[AliExpress search for STM32F103C8T6](https://www.aliexpress.com/wholesale?SearchText=stm32f103c8t6)

# Current branch information

This is a starter project, just changed for:
  - Setup to compile on CortexM0
  - Setup to use [probe-run](https://docs.rs/crate/probe-run/latest)
  - Output log information via RTT (see [rtt-target](https://docs.rs/rtt-target/latest/rtt_target/) for docs) 
  - Blink the onboard LED

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
