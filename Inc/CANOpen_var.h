#ifndef __CANOPEN_VAR_H__
#define __CANOPEN_VAR_H__
#include <stdint.h>
#include <CANOpen.h>
#include "stm32f4xx_hal.h"
// Constant values

#define NumOfNode 3

typedef enum _Input_Status{
  Init,
  Button1,
  Button2,
  Button3,
  Init_state,
}Input_Status;

extern uint8_t Ctrl_Mode;
// Related to the CAN communication in low level
extern CAN_HandleTypeDef CanHandle;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];

extern uint8_t RealTime;
extern uint8_t Timer_Flag;
extern uint16_t Load_bf[3];
extern uint16_t Load[3];
extern uint8_t Node[8];
extern uint8_t Input;
extern int16_t torque;
extern int32_t position;
extern int32_t velocity;
extern int32_t target_velocity;

extern CO_PDOStruct tpdo2;
extern CO_PDOStruct rpdo3;

extern uint16_t Load_pos[3];

//extern uint8_t Button1;
//extern uint8_t Button2;
//extern uint8_t Button3;

extern uint16_t adcValue[3];
extern int32_t Pos[3];
#ifdef __cplusplus
#endif
#endif /* __CANOPEN_VAR_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
