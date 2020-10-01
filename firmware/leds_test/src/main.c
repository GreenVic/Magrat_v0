// Device header file.
#include "stm32h7xx.h"
// Standard library includes.
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _siitcm, _sidtcm, _sitcm, _sdtcm, _eitcm, _edtcm, _estack;

// Core system clock speed.
uint32_t SystemCoreClock = 64000000;
// Global systick counter.
volatile uint32_t systick = 0;

// Reset handler: set the stack pointer and branch to main().
__attribute__( ( naked ) ) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}

// SysTick interrupt handler.
void SysTick_IRQn_handler( void ) {
  ++systick;
}

// Simple blocking millisecond delay method.
void delay_ms( uint32_t ms ) {
  // Calculate the 'end of delay' tick value, then wait for it.
  uint32_t next = systick + ms;
  while ( systick < next ) { __WFI(); }
}

/**
 * Main program.
 */
int main( void ) {
  // TODO: Enable SRAM1, SRAM2, SRAM3.
  // Enable GPIOC, GPIOD peripherals.
  RCC->AHB4ENR  |=  ( RCC_AHB4ENR_GPIOCEN |
                      RCC_AHB4ENR_GPIODEN );

  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit.
  SCB->CPACR    |=  ( 0xF << 20 );

  // Configure SysTick to trigger every ms.
  SysTick_Config( SystemCoreClock / 1000 );

  // Set LED pins (C6, C7, D12) to push-pull output mode.
  GPIOC->MODER  &= ~( ( 3 << ( 6 * 2 ) ) | ( 3 << ( 7 * 2 ) ) );
  GPIOD->MODER  &= ~( 3 << ( 12 * 2 ) );
  GPIOC->MODER  |=  ( ( 1 << ( 6 * 2 ) ) | ( 1 << ( 7 * 2 ) ) );
  GPIOD->MODER  |=  ( 1 << ( 12 * 2 ) );

  // Pull LED3 high, LED1 / LED2 low.
  GPIOC->ODR    &= ~( 1 << 6 );
  GPIOC->ODR    |=  ( 1 << 7 );
  GPIOD->ODR    &= ~( 1 << 12 );

  // Done; blink the LEDs every half-second.
  while( 1 ) {
    delay_ms( 500 );
    GPIOC->ODR ^=  ( ( 1 << 6 ) | ( 1 << 7 ) );
    GPIOD->ODR ^=  ( 1 << 12 );
  }
  return 0; // lol
}
