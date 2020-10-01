#include "qspi.h"

void qspi_wen( uint8_t dw ) {
  // Disable the peripheral.
  QUADSPI->CR   &= ~( QUADSPI_CR_EN );
  // Clear the instruction, mode, and transaction phases.
  QUADSPI->CCR  &= ~( QUADSPI_CCR_INSTRUCTION | QUADSPI_CCR_FMODE |
                      QUADSPI_CCR_IMODE | QUADSPI_CCR_DMODE |
                      QUADSPI_CCR_ADMODE );
  // Set N-wire instruction mode.
  QUADSPI->CCR  |=  ( dw << QUADSPI_CCR_IMODE_Pos );
  // Enable the peripheral and send the 'write enable' command.
  QUADSPI->CR   |=  ( QUADSPI_CR_EN );
  QUADSPI->CCR  |=  ( 0x06 << QUADSPI_CCR_INSTRUCTION_Pos );
  // Wait for the transaction to finish.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  // Disable the peripheral.
  QUADSPI->CR   &= ~( QUADSPI_CR_EN );
  // Wait until 'writes enabled' is set in the config register.
  qspi_reg_wait( 0x05, 0x03, 0x02, dw );
}

void qspi_reg_wait( uint8_t reg, uint32_t msk,
                    uint32_t mat, uint8_t dw ) {
  // Set the 'mask', 'match', and 'polling interval' values.
  QUADSPI->PSMKR = msk;
  QUADSPI->PSMAR = mat;
  QUADSPI->PIR   = 0x100;
  // Set the 'auto-stop' bit to end the transaction after a match.
  QUADSPI->CR   |=  ( QUADSPI_CR_APMS );
  // Clear instruction, mode and transaction phases.
  QUADSPI->CCR  &= ~( QUADSPI_CCR_INSTRUCTION | QUADSPI_CCR_FMODE |
                      QUADSPI_CCR_IMODE | QUADSPI_CCR_DMODE |
                      QUADSPI_CCR_ADMODE );
  // Set N-wire instruction and data modes, and auto-polling mode.
  QUADSPI->CCR  |=  ( ( dw << QUADSPI_CCR_IMODE_Pos ) |
                      ( dw << QUADSPI_CCR_DMODE_Pos ) |
                      ( 2 << QUADSPI_CCR_FMODE_Pos ) );
  // Enable the peripheral.
  QUADSPI->CR   |=  ( QUADSPI_CR_EN );
  // Set the given 'read register' instruction to start polling.
  QUADSPI->CCR  |=  ( reg << QUADSPI_CCR_INSTRUCTION_Pos );
  // Wait for a match.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  // Acknowledge the 'status match flag.'
  QUADSPI->FCR |=  ( QUADSPI_FCR_CSMF );
  // Disable the peripheral.
  QUADSPI->CR   &= ~( QUADSPI_CR_EN );
}

void qspi_erase_sector( uint32_t snum ) {
  // Send 'enable writes' command.
  qspi_wen( 1 );
  // Erase the sector, and wait for the operation to complete.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CCR  &= ~( QUADSPI_CCR_INSTRUCTION | QUADSPI_CCR_FMODE |
                      QUADSPI_CCR_IMODE | QUADSPI_CCR_DMODE |
                      QUADSPI_CCR_ADMODE );
  QUADSPI->CCR |=  ( ( 1 << QUADSPI_CCR_IMODE_Pos ) |
                     ( 1 << QUADSPI_CCR_ADMODE_Pos ) );
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );
  // 0x20 is the "sector erase" command.
  QUADSPI->CCR |=  ( 0x20 << QUADSPI_CCR_INSTRUCTION_Pos );
  // The address is equal to the sector number * 4KB.
  QUADSPI->AR   =  ( snum * 0x1000 );
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  // Disable the peripheral once the transaction is complete.
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  // Wait for the 'write in progress' bit to clear.
  qspi_reg_wait( 0x05, 0x01, 0x00, 1 );
}

void qspi_write_word( uint32_t addr, uint32_t data ) {
  // Send 'enable writes' command.
  qspi_wen( 1 );
  // Write the word of data.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CCR  &= ~( QUADSPI_CCR_INSTRUCTION |
                      QUADSPI_CCR_FMODE |
                      QUADSPI_CCR_IMODE |
                      QUADSPI_CCR_DMODE |
                      QUADSPI_CCR_ADMODE );
  QUADSPI->CCR |=  ( ( 1 << QUADSPI_CCR_IMODE_Pos ) |
                     ( 1 << QUADSPI_CCR_ADMODE_Pos ) |
                     ( 3 << QUADSPI_CCR_DMODE_Pos ) );
  // Set data length (3 + 1 = 4 bytes).
  QUADSPI->DLR = 3;
  // Enable the peripheral and set instruction, address, data.
  QUADSPI->CR  |=  ( QUADSPI_CR_EN );
  QUADSPI->CCR |=  ( 0x32 << QUADSPI_CCR_INSTRUCTION_Pos );
  QUADSPI->AR   =  ( addr );
  QUADSPI->DR   =  ( data );
  // Wait for the transaction to complete, and disable the peripheral.
  while ( QUADSPI->SR & QUADSPI_SR_BUSY ) {};
  QUADSPI->CR  &= ~( QUADSPI_CR_EN );
  // Clear the data length register.
  QUADSPI->DLR = 0;
  // Wait for the 'write in progress' bit to clear.
  qspi_reg_wait( 0x05, 0x01, 0x00, 1 );
}
