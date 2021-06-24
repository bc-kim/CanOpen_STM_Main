#include "cat25640_spi.h"

// https://www.onsemi.com/pub/Collateral/CAT25640-D.PDF

struct _EEPROM _eeprom;

void EEPROM_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* csPort, uint16_t csPin){
  _eeprom.hspi = hspi;
  _eeprom.csPort = csPort;
  _eeprom.csPin = csPin;
}

// EEPROM Write Operation (page 8)
void EEPROM_writeBytes(uint16_t address, uint8_t* data, uint8_t len){

  uint8_t cmd;
  uint8_t header[3];
  
  //The WREN instruction must be sent prior to any WRITE or WRSR instruction.
  cmd = EEPROM_WREN;
  HAL_GPIO_WritePin(_eeprom.csPort, _eeprom.csPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_eeprom.hspi, &cmd, len, 100);
  HAL_GPIO_WritePin(_eeprom.csPort, _eeprom.csPin, GPIO_PIN_SET);
  
  // Once the WEL bit is set, the user may execute a write sequence,
  // by sending a WRITE instructionn, a 16 bit address and data
  // Page 6, Figure 5
  header[0] = EEPROM_WRITE;
  header[1] = (uint8_t)(address >> 8);
  header[2] = (uint8_t)address;
  
  HAL_GPIO_WritePin(_eeprom.csPort, _eeprom.csPin, GPIO_PIN_RESET); // Start SPI Frame
  HAL_SPI_Transmit(_eeprom.hspi, header, 3, 100);
  
  // After sending the first data byte to the CAT25640, the host may continue sending data, up to a total of 64 bytes
  if(len > 64) len = 64;
  HAL_SPI_Transmit(_eeprom.hspi, data, len, 100);
  HAL_GPIO_WritePin(_eeprom.csPort, _eeprom.csPin, GPIO_PIN_SET); // End SPI Frame
  
}

// EEPROM Read Operation (page 11)
void EEPROM_readBytes(uint16_t address, uint8_t* data, uint8_t len){

  uint8_t header[3];
  
  // To read from memory, the host sends a READ instruction followed by a 16bit address
  // Page 11, Figure 9
  header[0] = EEPROM_READ;
  header[1] = (uint8_t)(address >> 8);
  header[2] = (uint8_t)address;
  
  HAL_GPIO_WritePin(_eeprom.csPort, _eeprom.csPin, GPIO_PIN_RESET); // Start SPI Frame
  HAL_SPI_Transmit(_eeprom.hspi, header, 3, 100);
  
  // After receiving the last address bit, the CAT25640 will respond by shifting out data on the SO pin
  HAL_SPI_Receive(_eeprom.hspi, data, len, 100);
  HAL_GPIO_WritePin(_eeprom.csPort, _eeprom.csPin, GPIO_PIN_SET); // End SPI Frame

}