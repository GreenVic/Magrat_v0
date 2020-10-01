# "Magrat" Board Test Programs

## Overview

These are a few basic test programs to communicate with the devices included on a first-revision STM32 SBC that I call a "Magrat".

I'm already planning a second revision with a few improvements:

* Better high-speed bus design. The 166-MHz SDRAM works, but it can't operate faster than 80MHz with the current design.
* Break out all of the JTAG pins, and BOOT0. SWD is not the best option for STM32H7 chips.
* Unify GPIOs into one or two headers.

The board includes:

* An STM32H743II or STM32H753II microcontroller
* 32MiB of SDRAM
* 16MiB of QSPI Flash
* microSD card socket
* 3 LEDs
* 1 button
* microUSB-B socket
* 50-pin FPC connector for an ILI9341 display
* NS2009 touch screen interface

And it has the following GPIO connectors:

* SPI
* UART
* I2C
* SWD
* 6 generic I/O pins

You can find the board design files in [TODO]. There is a KiCAD project, and a .zip archive containing PCB/PCA files in a format that JLCPCB (and probably other web-based PCA services) should accept.

Currently, the following tests are provided:

* leds\_test: Toggle the on-board LEDs.
* button\_test: Toggle the on-board LEDs when the user button is pressed.
* uart\_test: Test the UART interface by echoing data from `RX` to `TX`.
* sdram\_test: Test reading / writing external SDRAM, print results.
* qspi\_test: Test reading / writing on the external QSPI Flash chip.
* osc\_test: Test the HSE and LSE oscillators.
* mmc\_test: Test reading / writing data to a microSD card. (Overwrites existing data)
* linux\_bootloader: Simple bootloader program to initialize memory and start an XIP Linux kernel located in QSPI Flash.

## Board Recovery

It's much easier to soft-brick STM32H7s than simpler STM32 lines, so if you're using open-source tools, you might get into a situation where it is difficult to re-flash them. If that happens:

* Un-plug the board.

* Hold the reset button down.

* Plug the board in and open an OpenOCD connection while holding the reset button. Ignore the errors.

* Open a connection to OpenOCD (while still holding NRST low): `telnet localhost 4444`.

* Run `reset halt`, and release the reset button before the "halt" command times out.

* Run `stm32h7x mass_erase 0` to erase the offending application in Flash bank 0.

* Next time, consider adding a startup delay before your application does something potentially dicey. :)

I've had to do this a couple of times, but these chips are pretty resilient. Normally your debugger would take care of this process for you, but when you debug STM32H7 chips over SWD, the NRST pin can interfere with some debugging operations. It's broken the point that simply running `reset halt` crashes the chip! So when you design your own STM32H7 boards, you might want to expose all of the JTAG pins.

So this would be a bit easier with ST's closed-source tooling, but nobody likes a quitter...or a developer who uses Windows :P

## Pin Configurations

The following pinouts are used for GPIO connections. `NC` = Not Connected, and directional signal names are from the microcontroller's perspective:

| Pin   | Function |
|-------|----------|
| PA0   | GPIO (UART header) |
| PA1   | Display (R2) |
| PA2   | GPIO (Marked on silkscreen) |
| PA3   | GPIO (Marked on silkscreen) |
| PA4   | `NC` |
| PA5   | `NC` |
| PA6   | `NC` |
| PA7   | SDRAM NWE |
| PA8   | Display (R6) |
| PA9   | Display (Tear Effect IRQ) |
| PA10  | USB ID |
| PA11  | USB D- |
| PA12  | USB D+ |
| PA13  | SWDIO (Debug) |
| PA14  | SWCLK (Debug) |
| PA15  | `NC` |
| PB0   | `NC` |
| PB1   | `NC` |
| PB2   | QSPI CLK |
| PB3   | `NC` |
| PB4   | `NC` |
| PB5   | `NC` |
| PB6   | QSPI NCS |
| PB7   | `NC` |
| PB8   | UART RX |
| PB9   | UART TX |
| PB10  | I2C SCL |
| PB11  | I2C SDA |
| PB12  | Display (SPI, NCS) |
| PB13  | Display (SPI, SCK) |
| PB14  | Display (SPI, SDI) |
| PB15  | Display (SPI, SDO) |
| PC0   | `NC` |
| PC1   | `NC` |
| PC2\_C | `NC` |
| PC3\_C | `NC` |
| PC4   | SDRAM SDNE0 |
| PC5   | SDRAM SDCKE0 |
| PC6   | LED2 |
| PC7   | LED3 |
| PC8   | SDMMC D0 |
| PC9   | SDMMC D1 |
| PC10  | SDMMC D2 |
| PC11  | SDMMC D3 |
| PC12  | SDMMC CLK |
| PC13  | Button1 |
| PC14  | 32.768KHz XTal in |
| PC15  | 32.768KHz XTal out |
| PD0   | SDRAM D2 |
| PD1   | SDRAM D3 |
| PD2   | SDMMC CMD |
| PD3   | Display (Mode select 0) |
| PD4   | Display (Mode select 1) |
| PD5   | Display (Mode select 2) |
| PD6   | Display (Mode select 3) |
| PD7   | `NC` |
| PD8   | SDRAM D13 |
| PD9   | SDRAM D14 |
| PD10  | SDRAM D15 |
| PD11  | NS2009 Touch input IRQ |
| PD12  | LED1 |
| PD13  | `NC` |
| PD14  | SDRAM D0 |
| PD15  | SDRAM D1 |
| PE0   | SDRAM NBL0 |
| PE1   | SDRAM NBL1 |
| PE2   | SPI SCK |
| PE3   | GPIO (SPI header) |
| PE4   | SPI NCS |
| PE5   | SPI SDI |
| PE6   | SPI SDO |
| PE7   | SDRAM D4 |
| PE8   | SDRAM D5 |
| PE9   | SDRAM D6 |
| PE10  | SDRAM D7 |
| PE11  | SDRAM D8 |
| PE12  | SDRAM D9 |
| PE13  | SDRAM D10 |
| PE14  | SDRAM D11 |
| PE15  | SDRAM D12 |
| PF0   | SDRAM A0 |
| PF1   | SDRAM A1 |
| PF2   | SDRAM A2 |
| PF3   | SDRAM A3 |
| PF4   | SDRAM A4 |
| PF5   | SDRAM A5 |
| PF6   | QSPI D3 |
| PF7   | QSPI D2 |
| PF8   | QSPI D0 |
| PF9   | QSPI D1 |
| PF10  | Display (Data enable) |
| PF11  | SDRAM (NRAS) |
| PF12  | SDRAM (A6) |
| PF13  | SDRAM (A7) |
| PF14  | SDRAM (A8) |
| PF15  | SDRAM (A9) |
| PG0   | SDRAM (A10) |
| PG1   | SDRAM (A11) |
| PG2   | SDRAM (A12) |
| PG3   | `NC` |
| PG4   | SDRAM (BA0) |
| PG5   | SDRAM (BA1) |
| PG6   | Display (R7) |
| PG7   | Display (Dot clock) |
| PG8   | SDRAM (CLK) |
| PG9   | `NC` |
| PG10  | Display (B2) |
| PG11  | Display (B3) |
| PG12  | `NC` |
| PG13  | `NC` |
| PG14  | `NC` |
| PG15  | SDRAM (NCAS) |
| PH0   | 8MHz XTal in |
| PH1   | 8MHz XTal out |
| PH2   | GPIO (Marked on silkscreen) |
| PH3   | GPIO (Marked on silkscreen) |
| PH4   | GPIO (Marked on silkscreen) |
| PH5   | GPIO (Marked on silkscreen) |
| PH6   | Display (PWM brightness control) |
| PH7   | GPIO (I2C header) |
| PH8   | Display (NRST) |
| PH9   | Display (R3) |
| PH10  | Display (R4) |
| PH11  | Display (R5) |
| PH12  | Display (Dat/Cmd) |
| PH13  | Display (G2) |
| PH14  | Display (G3) |
| PH15  | Display (G4) |
| PI0   | Display (G5) |
| PI1   | Display (G6) |
| PI2   | Display (G7) |
| PI3   | `NC` |
| PI4   | Display (B4) |
| PI5   | Display (B5) |
| PI6   | Display (B6) |
| PI7   | Display (B7) |
| PI8   | `NC` |
| PI9   | Display (VSYNC) |
| PI10  | Display (HSYNC) |
| PI11  | `NC` |
