#include "main.h"
#include "CANOPEN_var.h"
#include "CANOpen.h"

uint8_t RxData[8];
CAN_RxHeaderTypeDef RxHeader;

extern Control_Mode Ctrl_Mode = Profile_pos;
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
extern int32_t Torque[3] = {0x00, 0x00, 0x00};
extern uint16_t Load_pos[3] = {0x00,0x00,0x00};
extern uint8_t data = 0;
extern uint8_t len = 0;

extern float A_Flexor = 1.0;
extern float A_Extensor = 1.0;
extern float B_Flexor = 1.0;
extern float B_Extensor = 1.0;

extern float Target_T[2] = {1.0, 1.0};
extern float Tension_error[2] = {1.0, 1.0};
extern float Tension_error_before[2] = {1.0, 1.0};
extern int32_t Target_Vel[2] = {1.0, 1.0};
extern float kp[2] = {1.0, 1.0};
extern float kd[2] = {1.0, 1.0};

extern int32_t Pos_ubound[2] = {1, 2};
extern float T_ubound[2] = {1.0, 2.0};
extern int32_t Pos_lbound[2] = {1, 2};

extern uint16_t Exp_Result = 500;
extern uint8_t Exp_finished = 1;