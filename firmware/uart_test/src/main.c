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

// UART4 interrupt handler.
void UART4_IRQn_handler( void ) {
  // 'Receive register not empty' interrupt.
  if ( UART4->ISR & USART_ISR_RXNE_RXFNE ) {
    // Echo the received character back over the TX line.
    // (Assume that this won't overflow the TX FIFO, because
    //  it won't trigger faster than the baud rate.)
    UART4->TDR = UART4->RDR;
  }
}

/**
 * Main program.
 */
int main( void ) {
  // TODO: Enable SRAM1, SRAM2, SRAM3.
  // Enable GPIOB and UART4 peripherals.
  RCC->AHB4ENR  |=  ( RCC_AHB4ENR_GPIOBEN );
  RCC->APB1LENR |=  ( RCC_APB1LENR_UART4EN );

  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  memcpy( &_sitcm, &_siitcm, ( ( void* )&_eitcm - ( void* )&_sitcm ) );
  memcpy( &_sdtcm, &_sidtcm, ( ( void* )&_edtcm - ( void* )&_sdtcm ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit.
  SCB->CPACR      |=  ( 0xF << 20 );

  // Set PLL1 'P' clock speed to 480MHz = ( 64MHz * ( N / M ) / P ).
  // But ( 64MHz / M ) <= 2Mhz, so P = 1, M = 32, N = 240.
  // Or, for 240MHz, M = 32, N = 64, P = 2
  // First, enable the PLL1_P output and set HSI as clock input.
  RCC->PLLCKSELR  &= ~( RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1 );
  RCC->PLLCKSELR  |=  ( 32 << RCC_PLLCKSELR_DIVM1_Pos );
  // Set 4 wait states in Flash with a 'programming delay' of 2.
  FLASH->ACR       =  ( ( 5 << FLASH_ACR_LATENCY_Pos ) |
                        ( 2 << FLASH_ACR_WRHIGHFREQ_Pos ) );
  // Set the PLL1 dividers register.
  RCC->PLL1DIVR   &= ~( RCC_PLL1DIVR_N1 | RCC_PLL1DIVR_P1 );
  //RCC->PLL1DIVR   |=  ( 129 << RCC_PLL1DIVR_N1_Pos );
  RCC->PLL1DIVR   |=  ( 239 << RCC_PLL1DIVR_N1_Pos |
                        1 << RCC_PLL1DIVR_P1_Pos );
  // Enable the PLL and wait for it to be ready.
  RCC->CR         |=  ( RCC_CR_PLL1ON );
  while( !( RCC->CR & RCC_CR_PLL1RDY ) ) {};
  // Switch to the PLL as the main clock source.
  RCC->CFGR       |=  ( RCC_CFGR_SW_PLL1 );
  while( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL1 ) {};
  // Clock speed is now 240MHz.
  SystemCoreClock  = 240000000;

  // Set UART pins (B8, B9) to alt. func. 8, medium-speed.
  GPIOB->MODER    &= ~( ( 3 << ( 8 * 2 ) ) | ( 3 << ( 9 * 2 ) ) );
  GPIOB->MODER    |=  ( ( 2 << ( 8 * 2 ) ) | ( 2 << ( 9 * 2 ) ) );
  GPIOB->OSPEEDR  |=  ( ( 1 << ( 8 * 2 ) ) | ( 1 << ( 9 * 2 ) ) );
  GPIOB->AFR[ 1 ] |=  ( ( 8 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 8 << ( ( 9 - 8 ) * 4 ) ) );

  // Enable the UART4 peripheral.
  UART4->BRR       =  ( SystemCoreClock / 115200 );
  UART4->CR1      |=  ( USART_CR1_UE |
                        USART_CR1_TE |
                        USART_CR1_RE |
                        USART_CR1_RXNEIE_RXFNEIE );

  // Enable the UART4 interrupt handler.
  // Use 4 bits for 'priority' and 0 bits for 'subpriority'.
  NVIC_SetPriorityGrouping( 0 );
  // Enable the EXTI0 interrupt.
  uint32_t uart_pri_encoding = NVIC_EncodePriority( 0, 3, 0 );
  NVIC_SetPriority( UART4_IRQn, uart_pri_encoding );
  NVIC_EnableIRQ( UART4_IRQn );

  // Done; wait.
  while( 1 ) {
    // Wait for an interrupt.
    __WFI();
  }
  return 0; // lol
}
