#include "global.h"
#include "qspi.h"

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
  // Enable GPIOB, GPIOF, QSPI, UART4
  RCC->AHB3ENR  |=  ( RCC_AHB3ENR_QSPIEN );
  RCC->AHB4ENR  |=  ( RCC_AHB4ENR_GPIOBEN |
                      RCC_AHB4ENR_GPIOFEN );
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
  // Or, for 240MHz, M = 32, N = 240, P = 2
  // First, enable the PLL1_P output and set HSI as clock input.
  RCC->PLLCKSELR  &= ~( RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1 );
  RCC->PLLCKSELR  |=  ( 32 << RCC_PLLCKSELR_DIVM1_Pos );
  // Set 4 wait states in Flash with a 'programming delay' of 2.
  FLASH->ACR       =  ( ( 5 << FLASH_ACR_LATENCY_Pos ) |
                        ( 2 << FLASH_ACR_WRHIGHFREQ_Pos ) );
  // Set the PLL1 dividers register.
  RCC->PLL1DIVR   &= ~( RCC_PLL1DIVR_N1 | RCC_PLL1DIVR_P1 );
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

  // Set QSPI pins (B2, B6, F6-9) to high-speed alt. func. mode.
  GPIOB->MODER    &= ~( ( 3 << ( 2 * 2 ) ) | ( 3 << ( 6 * 2 ) ) );
  GPIOB->MODER    |=  ( ( 2 << ( 2 * 2 ) ) | ( 2 << ( 6 * 2 ) ) );
  GPIOB->OSPEEDR  |=  ( ( 3 << ( 2 * 2 ) ) | ( 3 << ( 6 * 2 ) ) );
  GPIOB->AFR[ 0 ] |=  ( ( 9 << ( 2 * 4 ) ) | ( 10 << ( 6 * 4 ) ) );
  GPIOF->MODER    &= ~( ( 3 << ( 6 * 2 ) ) | ( 3 << ( 7 * 2 ) ) |
                        ( 3 << ( 8 * 2 ) ) | ( 3 << ( 9 * 2 ) ) );
  GPIOF->MODER    |=  ( ( 2 << ( 6 * 2 ) ) | ( 2 << ( 7 * 2 ) ) |
                        ( 2 << ( 8 * 2 ) ) | ( 2 << ( 9 * 2 ) ) );
  GPIOF->OSPEEDR  |=  ( ( 3 << ( 6 * 2 ) ) | ( 3 << ( 7 * 2 ) ) |
                        ( 3 << ( 8 * 2 ) ) | ( 3 << ( 9 * 2 ) ) );
  GPIOF->PUPDR    |=  ( ( 1 << ( 6 * 2 ) ) | ( 1 << ( 7 * 2 ) ) );
  GPIOF->AFR[ 0 ] |=  ( ( 9 << ( 6 * 4 ) ) | ( 9 << ( 7 * 4 ) ) );
  GPIOF->AFR[ 1 ] |=  ( ( 10 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 10 << ( ( 9 - 8 ) * 4 ) ) );

  // Enable the UART4 peripheral.
  UART4->BRR       =  ( SystemCoreClock / 115200 );
  UART4->CR1      |=  ( USART_CR1_UE | USART_CR1_TE );

  printf( "Configure QSPI...\r\n" );

  // QSPI peripheral setup.
  // Flash size: 16MiB = 2^(23+1) bytes.
  QUADSPI->DCR    |=  ( 23 << QUADSPI_DCR_FSIZE_Pos );
  // Set 24-bit addressing.
  QUADSPI->CCR    |=  ( 2 << QUADSPI_CCR_ADSIZE_Pos );
  // Set clock prescaler to 240 / ( 3 + 1 ) = 60MHz.
  QUADSPI->CR     |=  ( ( 3 << QUADSPI_CR_PRESCALER_Pos ) |
                        QUADSPI_CR_SSHIFT );

  // Send 'enable writes' command.
  qspi_wen( 1 );

  // Wait for the 'write enable latch' to be set.
  qspi_reg_wait( 0x05, 0x03, 0x02, 1 );

  // Send 'write volatile status register 2' to 0x02.
  // Clear instruction, mode and transaction phases.
  QUADSPI->CCR  &= ~( QUADSPI_CCR_INSTRUCTION | QUADSPI_CCR_FMODE |
                      QUADSPI_CCR_IMODE | QUADSPI_CCR_DMODE |
                      QUADSPI_CCR_ADMODE );
  // Set 1-wire instruction and data modes, and auto-polling mode.
  QUADSPI->CCR  |=  ( ( 1 << QUADSPI_CCR_IMODE_Pos ) |
                      ( 1 << QUADSPI_CCR_DMODE_Pos ) );
  // Enable the peripheral.
  QUADSPI->CR   |=  ( QUADSPI_CR_EN );
  // Send information.
  QUADSPI->CCR  |=  ( 0x31 << QUADSPI_CCR_INSTRUCTION_Pos );
  QUADSPI->DR    =  ( 0x02 );
  // Wait for the transaction to finish.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  // Disable the peripheral.
  QUADSPI->CR   &= ~( QUADSPI_CR_EN );

  // Wait for the register write to finish.
  qspi_reg_wait( 0x05, 0x01, 0x00, 1 );

  // Make sure that QSPI mode is enabled.
  qspi_reg_wait( 0x35, 0x02, 0x02, 1 );

  printf( "Done.\r\n" );

  // Test external Flash sector erase.
  //printf( "Erase sector...\r\n" );
  //qspi_erase_sector( 0 );
  //printf( "Done.\r\n" );

  // Test external Flash writes.
  //printf( "Write words...\r\n" );
  //qspi_write_word( 0x000000, 0x01234567 );
  //qspi_write_word( 0x000004, 0x89ABCDEF );
  //qspi_write_word( 0x00000C, 0xCABAFABA );
  //printf( "Done.\r\n" );

  // Test external Flash reads.
  QUADSPI->CCR  &= ~( QUADSPI_CCR_INSTRUCTION | QUADSPI_CCR_FMODE |
                      QUADSPI_CCR_IMODE | QUADSPI_CCR_DMODE |
                      QUADSPI_CCR_ADMODE );
  QUADSPI->CCR |= ( 3 << QUADSPI_CCR_FMODE_Pos |
                    3 << QUADSPI_CCR_ADMODE_Pos |
                    3 << QUADSPI_CCR_DMODE_Pos |
                    1 << QUADSPI_CCR_IMODE_Pos |
                    0xEB << QUADSPI_CCR_INSTRUCTION_Pos |
                    6 << QUADSPI_CCR_DCYC_Pos );
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );

  // Add a dummy cycle; if memory-mapped access is attempted
  // immediately after enabling the peripheral, it seems to fail.
  // I'm not sure why, but adding one nop instruction seems to fix it.
  __asm( "NOP" );
  delay_ms( 1 );

  // Test reading values from memory-mapped Flash.
  int val = *( ( uint32_t* ) 0x90000000 );
  printf( "QSPI[0]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000002 );
  printf( "QSPI[2]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000004 );
  printf( "QSPI[4]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000008 );
  printf( "QSPI[8]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x9000000D );
  printf( "QSPI[13]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000100 );
  printf( "QSPI[+]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000000 );
  printf( "QSPI[0]: 0x%08X\r\n", val );

  // Done; infinite loop.
  while( 1 ) {};
  return 0; // lol
}
