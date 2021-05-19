#ifndef __CANOPEN_BATCH_H__
#define __CANOPEN_BATCH_H__

#include "CANOpen.h"
#include <CANOpen_var.h>

extern CO_Status Init_CAN();
extern void Enter_RT_CAN(Control_Mode Ctrl_Mode);
void Zero_Pos(uint8_t time_out);
void Reset_MotorDriver();
extern void CAN_Device_Control(Control_word controlword, uint8_t Node);
extern enum NMT_Status CAN_NMT(NMT_OP mode, uint8_t Node, uint8_t Delay);
extern enum Control_Mode CAN_Check_ControlMode(uint8_t Node);
extern enum StateMachine CAN_Device_Status(uint8_t Node, uint8_t Delay);
extern void CAN_Set_ControlMode(Control_Mode controlmode, uint8_t Node);
// uint8_t CAN_NMT_CTR();
// 0x22 PDO Objects
extern CO_PDOStruct readPDO_0x22;
extern CO_PDOStruct sendPDO_0x22;
extern int32_t OD_0x22_target_position; // Position Control target value
extern int32_t OD_0x22_actual_position; // Position Control actual value

// 0x23 PDO Objects
extern CO_PDOStruct readPDO_0x23;
extern CO_PDOStruct sendPDO_0x23;
extern int32_t OD_0x23_target_position; // Position Control target value
extern int32_t OD_0x23_actual_position; // Position Control actual value

// 0x31 PDO Objects
extern CO_PDOStruct readPDO_0x31;
extern CO_PDOStruct sendPDO_0x31;
extern int32_t OD_0x31_actual_speed; // Speed Control actual value
extern int32_t OD_0x31_target_speed; // Speed Control target value

extern void CANOpen_batch_Init();
extern void CANOpen_batch_0x22();
extern void CANOpen_batch_0x23();
extern void CANOpen_batch_0x31();
extern void CANOpen_batch_Start();

#endif