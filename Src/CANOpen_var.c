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
extern uint8_t Node[8] = {1,3,0,0,0,0,0,0};
extern uint16_t Load_bf[3]={0x00,0x00,0x00};
extern uint16_t Load[3] = {0x00, 0x00, 0x00};
extern uint16_t adcValue[3] = {0,0,0};
extern int32_t Pos[3] = {0x00,0x00,0x00};
extern int16_t Torque[3] = {0x00, 0x00, 0x00};
extern uint16_t Load_pos[3] = {0x00,0x00,0x00};
extern uint8_t data = 0;
extern uint8_t len = 0;

extern float A_Flexor = 0.002;
extern float A_Extensor = 0.03;
extern float B_Flexor = 50;
extern float B_Extensor = 50;

extern float Target_T[2] = {150, 1.0};
extern float Tension_error[2] = {1.0, 1.0};
extern float Tension_error_before[2] = {1.0, 1.0};
extern int32_t Target_Vel[2] = {1.0, 1.0};
extern float kp[2] = {15.0, 15.0};
extern float kd[2] = {3.0, 3.0};

extern int32_t Pos_ubound[2] = {22000, 22000};
extern int32_t Vel_ubound[2] = {3000,3000};
extern float T_ubound[2] = {1000.0, 2000.0};
extern float T_lbound[2] = {50.0, 40.0};
extern int32_t Pos_lbound[2] = {-10, -10};
extern uint8_t Force_CO[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

extern uint16_t Exp_Result = 500;
extern uint8_t Exp_finished = 1;