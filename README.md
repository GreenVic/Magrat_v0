# "Magrat" STM32H7 Board

NOTE: This is an early revision of a board which I am in the process of revising. I'm publishing the design files for the sake of sharing now that I've tested many (but not all) of the board's main features, but there are plenty of improvements which should be made, and you should not treat this as a solid reference implementation or a finalized production-ready design.

With that disclaimer out of the way, "Magrat" is an STM32H7 board which aims to integrate enough external memory to run a minimal Linux system in a low-cost format. It is designed for single-sided PCA with parts from the LCSC catalog, so you should be able to order it 'as is' from JLCPCB when / if the parts are in stock.

This board should be able to run a mainline Linux kernel, and I've gotten one booting, but I'm still working on bringing up a fully-functional system, and I'm not quite sure that the microSD card socket works yet.

(TODO: add generated gerbers, BoM, pick-and-place files)

## Features

* STM32H7x3II: Cortex-M7, 480MHz, 2MiB Flash, 1MiB RAM

* 32MiB SDRAM (~80MHz top speed in this revision)

* 16MiB Quad-SPI Flash

* 50-pin ILI9341 TFT display connector (untested)

* USB full-speed (untested)

* microSD card socket (untested)

* UART, SPI, I2C headers.

## (TODO: more explanation)

Test programs and a bit more information can be found in the `firmware/` directory.

They're a little rough-around-the-edges right now, because I haven't had these boards for long and it's my first time using an STM32H7 chip.
