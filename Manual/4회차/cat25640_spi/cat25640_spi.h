#ifndef __CAT25640_SPI_H__
#define __CAT25640_SPI_H__

#include "stm32f4xx_hal.h"

// https://www.onsemi.com/pub/Collateral/CAT25640-D.PDF

// Instruction Set (P.6, Table 9)
#define EEPROM_WREN     0x06    // Enable Write Operations
#define EEPROM_WRDI     0x04    // Disable Write Operations
#define EEPROM_RDSR     0x05    // Read Status Register
#define EEPROM_WRSR     0x01    // Read Data from Memory
#define EEPROM_READ     0x03    // Read Data from Memory
#define EEPROM_WRITE    0x02    // Write Data to Memory

struct _EEPROM{

  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef* csPort;
  uint16_t csPin;
  
};

extern struct _EEPROM _eeprom;

extern void EEPROM_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* csPort, uint16_t csPin);
extern void EEPROM_writeBytes(uint16_t address, uint8_t* data, uint8_t len);
extern void EEPROM_readBytes(uint16_t address, uint8_t* data, uint8_t len);
#endif