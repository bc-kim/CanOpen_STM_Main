#include "CANOpen_batch.h"
#include <string.h>
CO_PDOStruct readPDO_0x22;
CO_PDOStruct sendPDO_0x22;
int32_t OD_0x22_target_position;
int32_t OD_0x22_actual_position;

CO_PDOStruct readPDO_0x23;
CO_PDOStruct sendPDO_0x23;
int32_t OD_0x23_target_position;
int32_t OD_0x23_actual_position;

CO_PDOStruct readPDO_0x31;
CO_PDOStruct sendPDO_0x31;
int32_t OD_0x31_actual_speed;
int32_t OD_0x31_target_speed;

static uint16_t cobID;
static uint8_t txData[8];

CO_Status Init_CAN(uint8_t Delay)
{
  uint8_t Status[8] = {
      0,
  };
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    /* Initialize the motor dirvers
    // Shut down ->  Switch Down -> Enable Operation*/
    if (i < NumOfNode)
    {
      if (CAN_NMT(Start_Remote_node, Node[i], Delay) == Operational)
      {
        CAN_Device_Control(Shut_down, Node[i]);
        HAL_Delay(Delay);
        CAN_Device_Control(Switch_on, Node[i]);
        HAL_Delay(Delay);
        CAN_Device_Control(Enable_operation, Node[i]);
        Status[i] = 1;
      }
      HAL_Delay(Delay);
    }
    else
    {
      Status[i] = 1;
    }
    sum = sum + Status[i];
  }
  if (sum == 8) // If all the nodes initialization is done well, the 'sum' value would be 8.
  {
    return CO_OK;
  }
  else
  {
    HAL_Delay(500);
    return CO_ERROR;
  }
}

void Enter_RT_CAN(Control_Mode Ctrl_Mode, uint8_t Delay)
{
  uint8_t Status[8] = {
      0,
  };
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    /* Initialize the motor dirvers
    // Shut down ->  Switch Down -> Enable Operation*/
    if (i < NumOfNode)
    {
      CAN_Set_ControlMode(Ctrl_Mode, Node[i]);
      if (CAN_Check_ControlMode(Node[i]) == Ctrl_Mode)
      {
        Status[i] = 1;
      }
    }
    else
    {
      Status[i] = 1;
    }
    sum = sum + Status[i];
  }
  if (sum == 8) // If all the nodes initialization is done well, the 'sum' value would be 8.
  {
    RealTime = 1;
  }
  else
  {
  }
}

void Zero_Pos(uint8_t Delay){
  uint8_t Status[8] = {
      0,
  };
  uint8_t sum = 0;
  // Change to the position mode.
  for (uint8_t i = 0; i < 8; i++)
  {
    if (i < NumOfNode)
    {
      CAN_Set_ControlMode(Cyclic_sync_pos, Node[i]);
      if (CAN_Check_ControlMode(Node[i]) == Cyclic_sync_pos)
      {
        Status[i] = 1;
      }
    }
    else
    {
      Status[i] = 1;
    }
    sum = sum + Status[i];
    HAL_Delay(Delay);
  }
  if (sum == 8) // If all the nodes initialization is done well, the 'sum' value would be 8.
  {
    for (uint8_t i = 0; i < 8; i++) // Go to zero position
    {
      if (i < NumOfNode)
      {
        CAN_Set_TargetValue(Cyclic_sync_pos,0x00000000,Node[i]);
      }
      HAL_Delay(Delay);
    }
  }
}

void Reset_MotorDriver(uint8_t Delay){
  for (uint8_t i = 0; i < 8; i++)
  {
    if (i < NumOfNode)
    {
      CAN_Device_Control(Shut_down,Node[i]);
      HAL_Delay(Delay);
    }
  }
  // Reset the node
  for (uint8_t i = 0; i < 8; i++)
  {
    if (i < NumOfNode)
    {
      CAN_NMT(Reset_node,Node[i],10);
      HAL_Delay(Delay);
    }
  }
  CANOpen_sendSync();
}

Control_Mode CAN_Check_ControlMode(uint8_t Node){
  if (Operational == CAN_NMT(Ask_NMT, Node, 10))
  {
    if (SM_Operation_enabled==CAN_Device_Status(Node, 10))
    {
      uint8_t check_data[4];
      CANOpen_readOD(Node, 0x6061, 0x00, check_data, &len, 1000);
      memcpy(&data,check_data,len);
      switch (data)
      {
      case 0x01:
        return Profile_pos;
        break;
      case 0x03:
        return Profile_vel;
        break;
      case 0x08:
        return Cyclic_sync_pos;
        break;
      case 0x09:
        return Cyclic_sync_vel;
        break;
      case 0xF7:
        return voltage;
        break;
      default:
        return error;
        break;
      }
    }
    else{
      return error;
    }
  }
  else {
    return error;
  }
}

void CAN_Set_ControlMode(Control_Mode controlmode, uint8_t Node)
{
  uint16_t mode;
  switch (controlmode)
  {
  case Profile_pos:
    mode = 0x01;    break;
  case Profile_vel:
    mode = 0x03;    break;
  case Cyclic_sync_pos:
    mode = 0x08;    break;
  case Cyclic_sync_vel:
    mode = 0x09;    break;
  case voltage:
    mode = 0xF7;    break;
  default:
    break;
  }
  CANOpen_writeOD_int8(Node, 0x6060, 0x00, mode, 1000);
}

void CAN_Device_Control(Control_word controlword, uint8_t Node)
{
  uint16_t control_word;
  switch (controlword)
  {
  case Shut_down:
   control_word = 0x06; break;
  case Switch_on:
   control_word = 0x07; break;
  case Enable_operation:
   control_word = 0x0F; break;
  case Fault_reset:
   control_word = 0x08; break;
  default: break;
  }
  CANOpen_writeOD_uint16(Node, 0x6040, 0x00, control_word, 1000);
}

void CAN_Set_TargetValue(Control_Mode controlmode, uint32_t data, uint8_t Node)
{
  switch (controlmode)
  {
  case Cyclic_sync_pos:
    CANOpen_writeOD_uint32(Node, 0x607A, 0x00, data, 1000);
    break;
  case Cyclic_sync_vel:
    CANOpen_writeOD_uint32(Node, 0x60FF, 0x00, data, 1000);
    break;
  default:
    break;
  }
}

StateMachine CAN_Device_Status(uint8_t Node, uint8_t Delay)
{
  uint8_t check_data[4];
  CANOpen_readOD(Node, 0x6041, 0x00, check_data, &len, 1000);
  //check_data_l = (uint8_t) check_data;
  //check_data_h = (check_data-check_data_l)<<8;
  //data = (uint16_t) check_data_l;
  memcpy(&data,check_data,len);
  if (data == 0x21 || data == 0x31 || data == 0xa1 || data == 0xb1) //Ready to switch on
  {
    return SM_Ready_to_switch_on;
  }
  else if (data == 0x23 || data == 0x33 || data== 0xa3 || data == 0xb3) //switched on
  {
    return SM_Switched_on;
  }
  else if (data == 0x27 || data == 0x37 || data == 0xa7 || data == 0xb7) // Operation enable
  {
    return SM_Operation_enabled;
  }
  return SM_Unknown;
}

NMT_Status CAN_NMT(NMT_OP mode, uint8_t Node, uint8_t Delay)
{
  //NMT_Status status;
  switch (mode)
  {
  case Start_Remote_node:
    cobID = 0x000;
    txData[0] = 0x01;
    txData[1] = 0x00 | Node;
    CANOpen_sendFrame(cobID,txData,2);
    HAL_Delay(Delay);
    cobID = 0x700 | Node;
    CANOpen_RemoteRequest(cobID);
    HAL_Delay(Delay);
    if (RxData[0] == 0x05 || RxData[0] == 0x85)
    {
      return Operational;
    }
    else
    {
      return Operation_not_worked;
    }
    break;
  case Enter_Pre_operation:
    cobID = 0x000;
    txData[0] = 0x80;
    txData[1] = 0x00 | Node;
    CANOpen_sendFrame(cobID, txData, 2);
    HAL_Delay(Delay);
    cobID = 0x700 | Node;
    CANOpen_RemoteRequest(cobID);
    HAL_Delay(Delay);
    if (RxData[0] == 0x7F || RxData[0] == 0xFF)
    {
      return Pre_operation;
    }
    else
    {
      return Operation_not_worked;
    }
    break;
  case Stop_Remote_node:
    cobID = 0x000;
    txData[0] = 0x02;
    txData[1] = 0x00 | Node;
    CANOpen_sendFrame(cobID, txData, 2);
    HAL_Delay(Delay);
    cobID = 0x700 | Node;
    CANOpen_RemoteRequest(cobID);
    HAL_Delay(Delay);
    if (RxData[0] == 0x04 || RxData[0] == 0x84)
    {
      return Stopped;
    }
    else
    {
      return Operation_not_worked;
    }
    break;
  case Reset_node:
    cobID = 0x000;
    txData[0] = 0x81;
    txData[1] = 0x00 | Node;
    HAL_Delay(Delay);
    CANOpen_sendFrame(cobID, txData, 2);
    cobID = 0x700 | Node;
    CANOpen_RemoteRequest(cobID);
    HAL_Delay(Delay);
    if (RxData[0] == 0x00)
    {
      return Initialization;
    }
    else
    {
      return Operation_not_worked;
    }
    break;
  case Reset_Communication:
    cobID = 0x000;
    txData[0] = 0x82;
    txData[1] = 0x00 | Node;
    CANOpen_sendFrame(cobID, txData, 2);
    HAL_Delay(Delay);
    cobID = 0x700 | Node;
    CANOpen_RemoteRequest(cobID);
    HAL_Delay(Delay);
    if (RxData[0] == 0x00)
    {
      return Initialization;
    }
    else
    {
      return Operation_not_worked;
    }
    break;
  case Ask_NMT:
    cobID = 0x700|Node;
    CANOpen_RemoteRequest(cobID);
    HAL_Delay(Delay);
    if (RxData[0] == 0x05 || RxData[0] == 0x85)
    {
      return Operational;
    }
    else if (RxData[0] == 0x7F || RxData[0] == 0xFF)
    {
      return Pre_operation;
    }
    else if (RxData[0] == 0x04 || RxData[0] == 0x84)
    {
      return Stopped;
    }
    else if (RxData[0] == 0x00)
    {
      return Initialization;
    }
    else
    {
      return Operation_not_worked;
    }
  default:
    return Operation_not_worked;
    break;
  }
}

void CANOpen_batch_Init(){
  CANOpen_writeOD_uint8(0x22, 0x2000, 0x04, 0x00, 100); // OD[2000, 04] = 0x00 (Set Motor driver to Stop state) [legacy]
  CANOpen_writeOD_uint8(0x23, 0x2000, 0x04, 0x00, 100); // OD[2000, 04] = 0x00 (Set Motor driver to Stop state) [legacy]
  CANOpen_writeOD_uint8(0x31, 0x2040, 0x00, 0x00, 100); // OD[2040, 00] = 0x00 (Set Motor driver to Stop state)
}

void CANOpen_batch_0x22(){
  // [[ 0x22 ]]
  // Set TPDO
  // 1. OD[1800, 01] = 0x80000180 + Node ID (Disable TPDO)
  // 2. OD[1A00 ,00] = 0x00 (Disable TPDO Mapping)
  // 3. OD[1A00, 01] = 0x20010220 (Mapping OD[2001, 02] to first entry)
  // 4. OD[1A00, 00] = 0x01 (Enable TPDO Mapping)
  // 5. OD[1800, 01] = 0x180 + Node ID (Enable TPDO)
  CANOpen_writeOD_uint32(0x22, 0x1800, 0x01, 0x22 | 0x80000180, 100);
  CANOpen_writeOD_uint8(0x22, 0x1A00, 0x00, 0, 100);
  CANOpen_writeOD_uint32(0x22, 0x1A00, 0x01, 0x20010220, 100); 
  CANOpen_writeOD_uint8(0x22, 0x1A00, 0x00, 1, 100);
  CANOpen_writeOD_uint32(0x22, 0x1800, 0x01, 0x22 | 0x180, 100);
  
  // Set RPDO
  // 1. OD[1400, 01] = 0x80000200 + Node ID (Disable RPDO)
  // 2. OD[1600 ,00] = 0x00 (Disable RPDO Mapping)
  // 3. OD[1600, 01] = 0x20000220 (Mapping OD[2000, 02] to first entry)
  // 4. OD[1600, 00] = 0x01 (Enable RPDO Mapping)
  // 5. OD[1400, 01] = 0x200 + Node ID (Enable RPDO)  
  CANOpen_writeOD_uint32(0x22, 0x1400, 0x01, 0x22 | 0x80000200, 100);
  CANOpen_writeOD_uint8(0x22, 0x1600, 0x00, 0, 100);
  CANOpen_writeOD_uint32(0x22, 0x1600, 0x01, 0x20000220, 100); 
  CANOpen_writeOD_uint8(0x22, 0x1600, 0x00, 1, 100);
  CANOpen_writeOD_uint32(0x22, 0x1400, 0x01, 0x22 | 0x200, 100);   
  
  CANOpen_mappingPDO_init(&sendPDO_0x22);
  CANOpen_mappingPDO_int32(&sendPDO_0x22, &OD_0x22_target_position);
  
  CANOpen_mappingPDO_init(&readPDO_0x22);
  CANOpen_mappingPDO_int32(&readPDO_0x22, &OD_0x22_actual_position);
}

void CANOpen_batch_0x23(){
    // [[ 0x23 ]]
  // 1. OD[1800, 01] = 0x80000180 + Node ID (Disable TPDO)
  // 2. OD[1A00 ,00] = 0x00 (Disable TPDO Mapping)
  // 3. OD[1A00, 01] = 0x20010220 (Mapping OD[2001, 02] to first entry)
  // 4. OD[1A00, 00] = 0x01 (Enable TPDO Mapping)
  // 5. OD[1800, 01] = 0x180 + Node ID (Enable TPDO)
  CANOpen_writeOD_uint32(0x23, 0x1800, 0x01, 0x23 | 0x80000180, 100);
  CANOpen_writeOD_uint8(0x23, 0x1A00, 0x00, 0, 100);
  CANOpen_writeOD_uint32(0x23, 0x1A00, 0x01, 0x20010220, 100); 
  CANOpen_writeOD_uint8(0x23, 0x1A00, 0x00, 1, 100);
  CANOpen_writeOD_uint32(0x23, 0x1800, 0x01, 0x23 | 0x180, 100);

  // 1. OD[1400, 01] = 0x80000200 + Node ID (Disable RPDO)
  // 2. OD[1600 ,00] = 0x00 (Disable RPDO Mapping)
  // 3. OD[1600, 01] = 0x20000220 (Mapping OD[2000, 02] to first entry)
  // 4. OD[1600, 00] = 0x01 (Enable RPDO Mapping)
  // 5. OD[1400, 01] = 0x200 + Node ID (Enable RPDO)    
  CANOpen_writeOD_uint32(0x23, 0x1400, 0x01, 0x23 | 0x80000200, 100);
  CANOpen_writeOD_uint8(0x23, 0x1600, 0x00, 0, 100);
  CANOpen_writeOD_uint32(0x23, 0x1600, 0x01, 0x20000220, 100); 
  CANOpen_writeOD_uint8(0x23, 0x1600, 0x00, 1, 100);
  CANOpen_writeOD_uint32(0x23, 0x1400, 0x01, 0x23 | 0x200, 100);  
  
  CANOpen_mappingPDO_init(&sendPDO_0x23);
  CANOpen_mappingPDO_int32(&sendPDO_0x23, &OD_0x23_target_position);
  
  CANOpen_mappingPDO_init(&readPDO_0x23);
  CANOpen_mappingPDO_int32(&readPDO_0x23, &OD_0x23_actual_position);
}

void CANOpen_batch_0x31(){
  CANOpen_mappingPDO_init(&sendPDO_0x31);
  CANOpen_mappingPDO_int32(&sendPDO_0x31, &OD_0x31_target_speed);
  
  CANOpen_mappingPDO_init(&readPDO_0x31);
  CANOpen_mappingPDO_int32(&readPDO_0x31, &OD_0x31_actual_speed);
}

void CANOpen_batch_Start(){
  CANOpen_writeOD_uint8(0x22, 0x2000, 0x04, 0x04, 100); // OD[2000, 04] = 0x00 (Set Motor driver to Position state) [legacy]
  CANOpen_writeOD_uint8(0x23, 0x2000, 0x04, 0x04, 100); // OD[2000, 04] = 0x00 (Set Motor driver to Position state) [legacy]
  CANOpen_writeOD_uint8(0x31, 0x2040, 0x00, 0x02, 100); // OD[2040, 00] = 0x00 (Set Motor driver to Speed state)
}