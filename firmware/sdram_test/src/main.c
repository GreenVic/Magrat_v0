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

// System call to support standard library print functions.
int _write( int handle, char* data, int size ) {
  int count = size;
  while( count-- ) {
    while( !( UART4->ISR & USART_ISR_TXE_TXFNF ) ) {};
    UART4->TDR = *data++;
  }
  return size;
}

/**
 * Main program.
 */
int main( void ) {
  // TODO: Enable SRAM1, SRAM2, SRAM3.
  // Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG,
  // FMC, and UART4 peripherals.
  RCC->AHB3ENR  |=  ( RCC_AHB3ENR_FMCEN );
  RCC->AHB4ENR  |=  ( RCC_AHB4ENR_GPIOAEN |
                      RCC_AHB4ENR_GPIOBEN |
                      RCC_AHB4ENR_GPIOCEN |
                      RCC_AHB4ENR_GPIODEN |
                      RCC_AHB4ENR_GPIOEEN |
                      RCC_AHB4ENR_GPIOFEN |
                      RCC_AHB4ENR_GPIOGEN );
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

  SysTick_Config( SystemCoreClock / 1000 );

  // Set UART pins (B8, B9) to alt. func. 8, medium-speed.
  GPIOB->MODER    &= ~( ( 3 << ( 8 * 2 ) ) | ( 3 << ( 9 * 2 ) ) );
  GPIOB->MODER    |=  ( ( 2 << ( 8 * 2 ) ) | ( 2 << ( 9 * 2 ) ) );
  GPIOB->OSPEEDR  |=  ( ( 1 << ( 8 * 2 ) ) | ( 1 << ( 9 * 2 ) ) );
  GPIOB->AFR[ 1 ] |=  ( ( 8 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 8 << ( ( 9 - 8 ) * 4 ) ) );

  // Set SDRAM pins to high-speed alt. func. mode.
  GPIOA->MODER    &= ~( 3 << ( 7 * 2 ) );
  GPIOA->MODER    |=  ( 2 << ( 7 * 2 ) );
  GPIOA->OSPEEDR  |=  ( 3 << ( 7 * 2 ) );
  GPIOA->AFR[ 0 ] |=  ( 12 << ( 7 * 4 ) );
  GPIOC->MODER    &= ~( ( 3 << ( 4 * 2 ) ) | ( 3 << ( 5 * 2 ) ) );
  GPIOC->MODER    |=  ( ( 2 << ( 4 * 2 ) ) | ( 2 << ( 5 * 2 ) ) );
  GPIOC->OSPEEDR  |=  ( ( 3 << ( 4 * 2 ) ) | ( 3 << ( 5 * 2 ) ) );
  GPIOC->AFR[ 0 ] |=  ( ( 12 << ( 4 * 4 ) ) | ( 12 << ( 5 * 4 ) ) );
  GPIOD->MODER    &= ~( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 8 * 2 ) ) | ( 3 << ( 9 * 2 ) ) |
                        ( 3 << ( 10 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOD->MODER    |=  ( ( 2 << ( 0 * 2 ) ) | ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 8 * 2 ) ) | ( 2 << ( 9 * 2 ) ) |
                        ( 2 << ( 10 * 2 ) ) | ( 2 << ( 14 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOD->OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 8 * 2 ) ) | ( 3 << ( 9 * 2 ) ) |
                        ( 3 << ( 10 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOD->AFR[ 0 ] |=  ( ( 12 << ( 0 * 4 ) ) | ( 12 << ( 1 * 4 ) ) );
  GPIOD->AFR[ 1 ] |=  ( ( 12 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 9 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 10 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 14 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
  GPIOE->MODER    &= ~( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 7 * 2 ) ) | ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 9 * 2 ) ) | ( 3 << ( 10 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) | ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOE->MODER    |=  ( ( 2 << ( 0 * 2 ) ) | ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 7 * 2 ) ) | ( 2 << ( 8 * 2 ) ) |
                        ( 2 << ( 9 * 2 ) ) | ( 2 << ( 10 * 2 ) ) |
                        ( 2 << ( 11 * 2 ) ) | ( 2 << ( 12 * 2 ) ) |
                        ( 2 << ( 13 * 2 ) ) | ( 2 << ( 14 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOE->OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 7 * 2 ) ) | ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 9 * 2 ) ) | ( 3 << ( 10 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) | ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOE->AFR[ 0 ] |=  ( ( 12 << ( 0 * 4 ) ) | ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 7 * 4 ) ) );
  GPIOE->AFR[ 1 ] |=  ( ( 12 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 9 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 10 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 11 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 12 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 13 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 14 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
  GPIOF->MODER    &= ~( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) | ( 3 << ( 3 * 2 ) ) |
                        ( 3 << ( 4 * 2 ) ) | ( 3 << ( 5 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) | ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOF->MODER    |=  ( ( 2 << ( 0 * 2 ) ) | ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 2 * 2 ) ) | ( 2 << ( 3 * 2 ) ) |
                        ( 2 << ( 4 * 2 ) ) | ( 2 << ( 5 * 2 ) ) |
                        ( 2 << ( 11 * 2 ) ) | ( 2 << ( 12 * 2 ) ) |
                        ( 2 << ( 13 * 2 ) ) | ( 2 << ( 14 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOF->OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) | ( 3 << ( 3 * 2 ) ) |
                        ( 3 << ( 4 * 2 ) ) | ( 3 << ( 5 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) | ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOF->AFR[ 0 ] |=  ( ( 12 << ( 0 * 4 ) ) | ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 2 * 4 ) ) | ( 12 << ( 3 * 4 ) ) |
                        ( 12 << ( 4 * 4 ) ) | ( 12 << ( 5 * 4 ) ) );
  GPIOF->AFR[ 1 ] |=  ( ( 12 << ( ( 11 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 12 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 13 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 14 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
  GPIOG->MODER    &= ~( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) | ( 3 << ( 4 * 2 ) ) |
                        ( 3 << ( 5 * 2 ) ) | ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOG->MODER    |=  ( ( 2 << ( 0 * 2 ) ) | ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 2 * 2 ) ) | ( 2 << ( 4 * 2 ) ) |
                        ( 2 << ( 5 * 2 ) ) | ( 2 << ( 8 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOG->OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) | ( 3 << ( 4 * 2 ) ) |
                        ( 3 << ( 5 * 2 ) ) | ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOG->AFR[ 0 ] |=  ( ( 12 << ( 0 * 4 ) ) | ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 2 * 4 ) ) | ( 12 << ( 4 * 4 ) ) |
                        ( 12 << ( 5 * 4 ) ) );
  GPIOG->AFR[ 1 ] |=  ( ( 12 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );

  // Enable the UART4 peripheral.
  UART4->BRR       =  ( SystemCoreClock / 115200 );
  UART4->CR1      |=  ( USART_CR1_UE | USART_CR1_TE );

  printf( "Configure FMC...\r\n" );

  // Enable the FMC peripheral.
  FMC_Bank1_R->BTCR[ 0 ]    |=  ( FMC_BCR1_FMCEN );

  // Remap SDRAM1 to 0x60000000.
  FMC_Bank1_R->BTCR[ 0 ]    |=  ( 1 << FMC_BCR1_BMAP_Pos );

  // Enable the SDRAM bank in the FMC peripheral.
  // FMC Bank 5, SDRAM1 Bank 0 = MT48LC16M16A2
  // Control Register (see datasheet for these values):
  //   * 16-bit memory width.
  //   * 9 column address bits.
  //   * 13 row address bits.
  //   * 4 internal banks.
  //   * 2 cycles of CAS latency.
  //   * Writes enabled.
  //   * SDCLK = SYSCLK * 2
  FMC_Bank5_6_R->SDCR[ 0 ]  =  ( ( 1 << FMC_SDCRx_NC_Pos ) |
                                 ( 2 << FMC_SDCRx_NR_Pos ) |
                                 ( 1 << FMC_SDCRx_MWID_Pos ) |
                                 ( FMC_SDCRx_NB ) |
                                 ( 2 << FMC_SDCRx_CAS_Pos ) |
                                 ( 3 << FMC_SDCRx_SDCLK_Pos ) );

  // Timing Register:
  // At 240MHz / 2, one clock cycle = 8.33ns. @480MHz/3, 6.25ns.
  // CAS Latency: 18ns (configured in SDCR)
  // Row-to-column delay: 18ns
  // Row precharge delay: 18ns
  // Recovery delay: 2 cycles
  // Row cycle delay: 67ns
  // Self refresh time: 42ns
  // Exit self-refresh delay: 67ns
  // "Load mode register" to active delay: 2 cycles
  FMC_Bank5_6_R->SDTR[ 0 ]  =  ( ( 2 << FMC_SDTRx_TMRD_Pos ) |
                                 ( 8 << FMC_SDTRx_TXSR_Pos ) |
                                 ( 5 << FMC_SDTRx_TRAS_Pos ) |
                                 ( 8 << FMC_SDTRx_TRC_Pos ) |
                                 ( 2 << FMC_SDTRx_TWR_Pos ) |
                                 ( 2 << FMC_SDTRx_TRP_Pos ) |
                                 ( 2 << FMC_SDTRx_TRCD_Pos ) );
  //FMC_Bank5_6_R->SDTR[ 0 ]  =  0xFFFFFFFF;

  // Configrue 'target bank' bits in SDCMR, and
  // set SDCMR 'MODE' bits to 1 to start the SDRAM clock.
  /*
  FMC_Bank5_6_R->SDCMR     |=  ( FMC_SDCMR_CTB1 );
  FMC_Bank5_6_R->SDCMR     |=  ( 1 << FMC_SDCMR_MODE_Pos );
  */
  FMC_Bank5_6_R->SDCMR     =  ( ( 1 << FMC_SDCMR_MODE_Pos ) |
                                FMC_SDCMR_CTB1 );

  // Wait at least 100us for the SDRAM power-up delay.
  delay_ms( 2 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_Bank5_6_R->SDSR & 0x00000020 ) {};

  // Set 'MODE' bits to 2 to issue a 'precharge all' command.
  /*
  FMC_Bank5_6_R->SDCMR     &= ~( FMC_SDCMR_MODE );
  FMC_Bank5_6_R->SDCMR     |=  ( 2 << FMC_SDCMR_MODE_Pos );
  */
  FMC_Bank5_6_R->SDCMR     =  ( ( 2 << FMC_SDCMR_MODE_Pos ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_Bank5_6_R->SDSR & 0x00000020 ) {};

  // Set 'MODE' bits to 3 and set # of consecutive auto-refreshes (2).
  /*
  //FMC_Bank5_6_R->SDCMR     |=  ( 1 << FMC_SDCMR_NRFS_Pos );
  FMC_Bank5_6_R->SDCMR     |=  ( 7 << FMC_SDCMR_NRFS_Pos );
  FMC_Bank5_6_R->SDCMR     &= ~( FMC_SDCMR_MODE );
  FMC_Bank5_6_R->SDCMR     |=  ( 3 << FMC_SDCMR_MODE_Pos );
  */
  FMC_Bank5_6_R->SDCMR     =  ( ( 1 << FMC_SDCMR_NRFS_Pos ) |
                                ( 3 << FMC_SDCMR_MODE_Pos ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_Bank5_6_R->SDSR & 0x00000020 ) {};

  // Send 'Load Mode Register' command.
  //   * Burst length: 1
  //   * Burst type: Sequential
  //   * CAS latency: 2 cycles
  //   * Operating mode: Standard
  //   * Write burst: Same as read
  //FMC_Bank5_6_R->SDCMR     |=  ( 0x020 << FMC_SDCMR_MRD_Pos );
  //FMC_Bank5_6_R->SDCMR     &= ~( FMC_SDCMR_MODE );
  //FMC_Bank5_6_R->SDCMR     |=  ( 4 << FMC_SDCMR_MODE_Pos );
  FMC_Bank5_6_R->SDCMR     =  ( ( 0x020 << FMC_SDCMR_MRD_Pos ) |
                                ( 4 << FMC_SDCMR_MODE_Pos ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_Bank5_6_R->SDSR & 0x00000020 ) {};

  // Configure SDRTR Refresh Timing Register:
  // 7.8us * 120MHz - 20 = 916.
  FMC_Bank5_6_R->SDRTR     |=  ( 916 << FMC_SDRTR_COUNT_Pos );

  // Make sure writes are enabled.
  FMC_Bank5_6_R->SDCR[ 0 ] &= ~( FMC_SDCRx_WP );

  printf( "Done.\r\n" );

  // Test external RAM reads and writes.
  uint32_t* sdram  = ( uint32_t* )0x60000000;
  uint16_t* sdramh = ( uint16_t* )0x60000000;
  uint8_t*  sdramb = ( uint8_t*  )0x60000000;
  printf( "RAM[0]: 0x%08lX (Uninitialized)\r\n", sdram[ 0 ] );
  sdram[ 0 ] = 0x01234567;
  sdram[ 1 ] = 0x89ABCDEF;
  printf( "RAM[0]: 0x%02X (Byte)\r\n", sdramb[ 0 ] );
  printf( "RAM[0]: 0x%04X (Halfword)\r\n", sdramh[ 0 ] );
  printf( "RAM[0]: 0x%08lX (Word)\r\n", sdram[ 0 ] );
  printf( "RAM[4]: 0x%08lX\r\n", sdram[ 1 ] );

  // Done; infinite loop.
  while( 1 ) {};
  return 0; // lol
}
