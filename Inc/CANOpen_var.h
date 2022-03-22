#ifndef __CANOPEN_VAR_H__
#define __CANOPEN_VAR_H__
#include <stdint.h>
#include <CANOpen.h>
#include "stm32f4xx_hal.h"
// Constant values

#define NumOfNode 4

typedef enum SDO_OP{
  Set_OP_mode,
  Device_Control,
}SDO_OP;

typedef enum StateMachine
{
  SM_Ready_to_switch_on,
  SM_Switched_on,
  SM_Operation_enabled,
  SM_Quick_Stop,
  SM_Halt,
  SM_Unknown,
}StateMachine;

typedef enum Control_word
{
  Shut_down,
  Switch_on,
  Disable_voltage,
  Quick_stop,
  Disable_operation,
  Enable_operation,
  Fault_reset,
} Control_word;

typedef enum NMT_OP{
  Start_Remote_node,
  Enter_Pre_operation,
  Stop_Remote_node,
  Reset_node,
  Reset_Communication,
  Ask_NMT,
}NMT_OP;

typedef enum NMT_Status
{
  Operation_not_worked,
  Initialization,
  Pre_operation,
  Stopped,
  Operational,
} NMT_Status;

typedef enum Input_Status{
  Init,
  Button1,
  Button2,
  Button3,
  Init_state,
  RT_on,
  Reached,
}Input_Status;

extern int32_t Pos_ubound_hard[4];
extern int32_t Pos_lbound_hard[4];
extern Control_Mode Ctrl_Mode;
extern Control_Mode Con_Mode[4];
extern Control_Mode Con_Mode_Prev[4];
extern Control_Mode Con_Mode_input[4];
extern CAN_HandleTypeDef CanHandle;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];
extern uint8_t data;
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
extern uint8_t len;
extern uint16_t Load_pos[3];

extern uint16_t adcValue[3];
extern int32_t Pos[4];
extern int32_t Vel[4];
extern int16_t Torque[4];
extern uint16_t State[4];
extern uint8_t Force_Reader;
extern uint16_t StatusWord[4];
extern uint16_t ControlWord[4];

extern int32_t Desired_PV[4];
extern int32_t Desired_start_PV[4];
extern uint16_t Desired_A[4];
extern uint16_t Desired_A_Prev[4];
extern uint16_t Desired_start_A[4];
extern int16_t Desired_F[4];
extern int16_t Desired_start_F[4];
extern uint16_t Duration_motor [4];
extern uint8_t Pos_lim_flag[4];
extern int32_t Desired_input_PV[4];
extern int16_t Desired_input_F[4];
extern uint16_t Desired_input_A[4];

extern float A_Flexor;
extern float A_Extensor;
extern float B_Flexor;
extern float B_Extensor;
extern uint8_t Direction;
extern uint8_t Read_OD_Data[4];
extern uint8_t Motor_Status;

extern float Target_T[2];
extern float Tension_error[2];
extern float Tension_error_before[2];
extern int32_t Target_Vel[4];
extern int16_t Target_Tor[4];
extern int32_t Target_Pos[4];
extern int32_t Pos_check[4];

extern int32_t Vel_ubound[4];
extern float kp[2];
extern float kd[2];

extern int32_t Pos_ubound[4];
extern float T_ubound[2];
extern float T_lbound[2];
extern int32_t Pos_lbound[4];
extern uint8_t Force_CO[6];

extern uint8_t Pos_limit_flag[4];
extern Control_Mode Con_ModePreset_M1[10];
extern uint16_t DesiredForcePreset_M1[10];
extern int32_t DesiredValuePreset_M1[10];

extern Control_Mode Con_ModePreset_M2[10];
extern uint16_t DesiredForcePreset_M2[10];
extern int32_t DesiredValuePreset_M2[10];

extern Control_Mode Con_ModePreset_M3[10];
extern uint16_t DesiredForcePreset_M3[10];
extern int32_t DesiredValuePreset_M3[10];

extern Control_Mode Con_ModePreset_M4[10];
extern uint16_t DesiredForcePreset_M4[10];
extern int32_t DesiredValuePreset_M4[10];


//extern CO_MOTOR motor_[4];

#ifdef __cplusplus
#endif
#endif /* __CANOPEN_VAR_H__ */

    /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
