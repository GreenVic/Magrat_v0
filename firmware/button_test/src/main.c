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

// Interrupt handler for EXTI13.
void EXTI15_10_IRQn_handler( void ) {
  if ( EXTI->PR1 & ( 1 << 13 ) ) {
    // Acknowledge the interrupt.
    EXTI->PR1 |= ( 1 << 13 );
    // Toggle LED pins.
    GPIOC->ODR ^=  ( ( 1 << 6 ) | ( 1 << 7 ) );
    GPIOD->ODR ^=  ( 1 << 12 );
  }
}

/**
 * Main program.
 */
int main( void ) {
  // TODO: Enable SRAM1, SRAM2, SRAM3.
  // Enable GPIOC, GPIOD, SYSCFG peripherals.
  RCC->AHB4ENR  |=  ( RCC_AHB4ENR_GPIOCEN |
                      RCC_AHB4ENR_GPIODEN );
  RCC->APB4ENR  |=  ( RCC_APB4ENR_SYSCFGEN );

  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  memcpy( &_sitcm, &_siitcm, ( ( void* )&_eitcm - ( void* )&_sitcm ) );
  memcpy( &_sdtcm, &_sidtcm, ( ( void* )&_edtcm - ( void* )&_sdtcm ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit.
  SCB->CPACR    |=  ( 0xF << 20 );

  // Set LED pins (C6, C7, D12) to push-pull output mode.
  GPIOC->MODER  &= ~( ( 3 << ( 6 * 2 ) ) | ( 3 << ( 7 * 2 ) ) );
  GPIOD->MODER  &= ~( 3 << ( 12 * 2 ) );
  GPIOC->MODER  |=  ( ( 1 << ( 6 * 2 ) ) | ( 1 << ( 7 * 2 ) ) );
  GPIOD->MODER  |=  ( 1 << ( 12 * 2 ) );
  // Configure button pin (C13) for a falling-edge input.
  GPIOC->MODER  &= ~( 3 << ( 13 * 2 ) );
  GPIOC->PUPDR  |=  ( 1 << ( 13 * 2 ) );
  SYSCFG->EXTICR[ 3 ] |=  ( SYSCFG_EXTICR4_EXTI13_PC );
  EXTI->IMR1 |=  ( 1 << 13 );
  EXTI->RTSR1 &= ~( 1 << 13 );
  EXTI->FTSR1 |=  ( 1 << 13 );
  EXTI->PR1 |= ( 1 << 13 );

  // Pull LED3 high, LED1 / LED2 low.
  GPIOC->ODR    &= ~( 1 << 6 );
  GPIOC->ODR    |=  ( 1 << 7 );
  GPIOD->ODR    &= ~( 1 << 12 );

  // Enable the EXTI10-15 interrupt handler.
  // Use 4 bits for 'priority' and 0 bits for 'subpriority'.
  NVIC_SetPriorityGrouping( 0 );
  // Enable the EXTI0 interrupt.
  uint32_t btn_pri_encoding = NVIC_EncodePriority( 0, 3, 0 );
  NVIC_SetPriority( EXTI15_10_IRQn, btn_pri_encoding );
  NVIC_EnableIRQ( EXTI15_10_IRQn );

  // Done; wait.
  while( 1 ) { __WFI(); }
  return 0; // lol
}
