#include "main.h"
#include "CANOPEN_var.h"
#include "CANOpen.h"

uint8_t RxData[8];
CAN_RxHeaderTypeDef RxHeader;

extern uint8_t Ctrl_Mode = 0;
extern uint8_t Input = 0;
extern uint8_t RealTime = 0;
extern uint8_t Timer_Flag = 0;
extern int16_t torque = 0;
extern int32_t position = 0;
extern int32_t velocity = 0;
extern int32_t target_velocity = 0;
extern uint8_t Node[8] = {1,0,0,0,0,0,0,0};
extern uint16_t Load_bf[3]={0x00,0x00,0x00};
extern uint16_t Load[3] = {0x00, 0x00, 0x00};
extern uint16_t adcValue[3] = {0,0,0};
extern int32_t Pos[3] = {0x00,0x00,0x00};
extern uint16_t Load_pos[3] = {0x00,0x00,0x00};
