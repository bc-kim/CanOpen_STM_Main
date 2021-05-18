#include "CANOpen_batch.h"

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

void Init_CAN(){
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
      Status[i] = CAN_BC_Enable_CTR(255, Node[i]);
    }
    else
    {
      Status[i] = 1;
    }
    sum = sum + Status[i];
  }
  if (sum == 8) // If all the nodes initialization is done well, the 'sum' value would be 8.
  {
    
  }
  else
  {
    
    HAL_Delay(500);
  }
}

void Enter_RT_CAN(uint8_t Ctrl_Mode){

}

void Zero_Pos(uint8_t time_out){

}

void Reset_MotorDriver(){

}

uint8_t CAN_BC_Enable_CTR(uint8_t mode, uint8_t Node){
    return 1;
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