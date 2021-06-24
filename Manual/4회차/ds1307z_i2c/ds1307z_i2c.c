#include "ds1307z_i2c.h"

struct _DS1307Z _ds1307z;


// https://datasheets.maximintegrated.com/en/ds/DS1307.pdf

void DS1307Z_Init(I2C_HandleTypeDef* hi2c){
  _ds1307z.hi2c = hi2c;
}

void DS1307Z_writeTime(uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute, uint8_t second){

  uint8_t sendData[8];
  uint8_t year8 = year - 2000;
  
  // Page 8, Table 2 
  // Set Data
  sendData[0] = 0x00;
  sendData[1] = (second / 10) << 4 | (second % 10);
  sendData[2] = (minute / 10) << 4 | (minute % 10);
  sendData[3] = (hour / 10) << 4 | (hour % 10);
  sendData[5] = (date / 10) << 4 | (date % 10);
  sendData[6] = (month / 10) << 4 | (month % 10);
  sendData[7] = (year8 / 10) << 4 | (year8 % 10);
  
  HAL_I2C_Master_Transmit(_ds1307z.hi2c, DS1307Z_ADDR << 1, sendData, 8, 100);
  
}

void DS1307Z_readTime(){
  
  uint8_t addr = 0x00;
  uint8_t readData[7];
  
  HAL_I2C_Master_Transmit(_ds1307z.hi2c, DS1307Z_ADDR << 1, &addr, 1, 100);
  
  HAL_I2C_Master_Receive(_ds1307z.hi2c, DS1307Z_ADDR << 1, readData, 7, 100);
  
  _ds1307z.second = ((readData[0] & 0x70) >> 4)*10 + (readData[0] & 0x0F);
  _ds1307z.minute = (readData[1] >> 4)*10 + (readData[1] & 0x0F);
  _ds1307z.hour = ((readData[2] & 0x30) >> 4)*10 + (readData[2] & 0x0F);
  _ds1307z.date = (readData[4] >> 4)*10 + (readData[4] & 0x0F);
  _ds1307z.month = (readData[5] >> 4)*10 + (readData[5] & 0x0F);
  _ds1307z.year = 2000 + (readData[6] >> 4)*10 + (readData[6] & 0x0F);

}
