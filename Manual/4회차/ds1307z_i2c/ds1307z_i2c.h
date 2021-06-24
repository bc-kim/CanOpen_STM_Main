#ifndef __DS1307Z_I2C_H__
#define __DS1307Z_I2C_H__

#include "stm32f4xx_hal.h"

// https://datasheets.maximintegrated.com/en/ds/DS1307.pdf

#define DS1307Z_ADDR    0x68

struct _DS1307Z{
  I2C_HandleTypeDef* hi2c;
  uint8_t month, date, hour, minute, second;
  uint16_t year;
};

extern struct _DS1307Z _ds1307z;

extern void DS1307Z_Init(I2C_HandleTypeDef* hi2c);
extern void DS1307Z_writeTime(uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute, uint8_t seconds);
extern void DS1307Z_readTime();

#endif