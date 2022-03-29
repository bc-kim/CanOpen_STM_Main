#include "main.h"
#include "CANOPEN_var.h"
#include "CANOpen.h"

uint8_t RxData[8];
CAN_RxHeaderTypeDef RxHeader;

extern Control_Mode Ctrl_Mode = Profile_pos;
//extern CO_MOTOR motor_[4];
extern uint8_t Input = 0;
extern uint8_t RealTime = 0;
extern uint8_t Timer_Flag = 0;
extern int16_t torque = 0;
extern int32_t position = 18000;
extern int32_t velocity = 0;
extern int32_t target_velocity = 0;
extern uint8_t Node[8] = {1,2,3,4,0,0,0,0};
extern uint16_t Load_bf[3]={0x00,0x00,0x00};
extern uint16_t Load[3] = {0x00, 0x00, 0x00};
extern uint16_t adcValue[3] = {0,0,0};
extern int32_t Pos[4] = {0x00,0x00,0x00,0x00};
extern int32_t Vel[4] = {0x00, 0x00, 0x00, 0x00};
extern int16_t Torque[4] = {0x00, 0x00, 0x00, 0x00};
extern uint16_t State[4] = {0, 0, 0, 0};
extern uint16_t Load_pos[3] = {0x00,0x00,0x00};
extern uint8_t data = 0;
extern uint8_t len = 0;
extern uint8_t Force_Reader = 0;
extern uint16_t StatusWord[4]={0x00,0x00,0x00,0x00};
extern uint16_t ControlWord[4]={0x00,0x00,0x00,0x00};

extern int32_t Desired_PV[4] = {0x00, 0x00, 0x00, 0x00};
extern int32_t Desired_start_PV[4] = {0x00, 0x00, 0x00, 0x00};
extern uint16_t Desired_A[4] = {0x00, 0x00, 0x00, 0x00};
extern uint16_t Desired_A_Prev[4] = {0x00, 0x00, 0x00, 0x00};
extern uint16_t Desired_start_A[4] = {0x00, 0x00, 0x00, 0x00};
extern int16_t Desired_F[4] = {0x00, 0x00, 0x00, 0x00};
extern int16_t Desired_start_F[4] = {0x00, 0x00, 0x00, 0x00};
extern int32_t Desired_input_PV[4] = {0, 0, 0, 0};
extern int16_t Desired_input_F[4] = {0, 0, 0, 0};
extern uint16_t Desired_input_A[4] = {0, 0, 0, 0};

extern uint16_t Duration_motor[4] = {1000, 1000, 1000, 1000};
extern uint8_t Pos_lim_flag[4] = {0,0,0,0};

extern float A_Flexor = 0.002;
extern float A_Extensor = 0.01;
//extern float A_Flexor = 0;
//extern float A_Extensor = 0;

extern float B_Flexor = 50;
extern float B_Extensor = 90;

extern float Target_T[2] = {350, 1.0};
extern float Tension_error[2] = {1.0, 1.0};
extern float Tension_error_before[2] = {1.0, 1.0};
extern int32_t Target_Vel[4] = {1.0, 1.0, 1, 1};
extern int32_t Target_Pos[4] = {1.0, 1.0, 1, 1};
extern int16_t Target_Tor[4] = {1, 1, 1, 1};
extern int32_t Pos_check[4] = {1,1,1,1};

extern float kp[2] = {15.0, 15.0};
extern float kd[2] = {3.0, 3.0};

extern int32_t Pos_ubound[4] = {200000, 18000, 20000,25000};
extern int32_t Pos_ubound_hard[4] = {220000, 20000, 22000, 28000};
extern int32_t Pos_lbound[4] = {-50, -50, -50, -50};
extern int32_t Pos_lbound_hard[4] = {-80, -80, -80, -80};
extern int32_t Vel_ubound[4] = {3000,3000,3000, 3000};
extern float T_ubound[2] = {1000.0, 2000.0};
extern float T_lbound[2] = {50.0, 40.0};
extern uint8_t Force_CO[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
extern uint8_t Read_OD_Data[4] = {0x00, 0x00, 0x00, 0x00};
extern uint8_t Motor_Status = 0;

extern uint16_t Exp_Result = 500;
extern uint8_t Exp_finished = 1;
extern uint8_t Direction = 1;

extern Control_Mode Con_Mode[4]={Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos};
extern Control_Mode Con_Mode_input[4]={Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos};
extern Control_Mode Con_Mode_Prev[4]={Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos};

extern uint8_t Pos_limit_flag[4] = {0, 0, 0, 0};
extern Control_Mode Con_ModePreset_M1[20] = {Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos,Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos};
extern uint16_t DesiredForcePreset_M1[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
extern Control_Mode Con_ModePreset_M2[20] = {Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos,Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos};
extern uint16_t DesiredForcePreset_M2[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
extern Control_Mode Con_ModePreset_M3[20] = {Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos,Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos};
extern uint16_t DesiredForcePreset_M3[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
extern Control_Mode Con_ModePreset_M4[20] = {Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos,Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos, Cyclic_sync_pos};
extern uint16_t DesiredForcePreset_M4[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

extern int32_t DesiredValuePreset_M1[20] = {0,200000,0,0,200000,0,0,200000,0,0,200000,0,0,200000,0,0,200000,0,0,200000};
extern int32_t DesiredValuePreset_M2[20] = {12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000};
extern int32_t DesiredValuePreset_M3[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
extern int32_t DesiredValuePreset_M4[20] = {0,0,0,18000,0,0,18000,0,0,18000,0,0,18000,0,0,18000,0,0,18000,0};
