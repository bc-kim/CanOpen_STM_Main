#ifndef __CANOPENSLIM_H__
#define __CANOPENSLIM_H__

#include "CANOpen_hw_appl.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum CO_Status{
  CO_OK,
  CO_TIMEOUT,
  CO_ERROR,
}CO_Status;

typedef enum CO_NMT{
  CO_RESET,
  CO_COMMRESET,
  CO_STOPPED,
  CO_PREOP,
  CO_OP
}CO_NMT;

#define CO_PDOSTR_LEN        10
typedef struct CO_PDOStruct{
  void* data[CO_PDOSTR_LEN];
  uint8_t bitlen[CO_PDOSTR_LEN];
  uint8_t mappinglen;
}CO_PDOStruct;

typedef enum PDO_Status
{
  PDO_CV_Waiting,
  PDO_CV_Received, // PDO_CV
  PDO_CV_Converged,
  PDO_DV_Updated,
  PDO_DV_Sent,
  PDO_CV_NotConverged,
  PDO_error,
} PDO_Status;

typedef enum Control_Mode
{
  error,
  Profile_pos,
  Profile_vel,
  Cyclic_sync_pos,
  Cyclic_sync_vel,
  Voltage,
  Homing,
  Cyclic_sync_tor,
  Admittance,
} Control_Mode;

typedef struct CO_MOTOR
{
  uint8_t id;
  CO_PDOStruct TPDO[4];
  CO_PDOStruct RPDO[4];
  PDO_Status PDO_Status;
  Control_Mode Con_Mode[20];
  int32_t DesiredValue[20];
  uint16_t DesiredForce[20];
} CO_MOTOR;

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
/*****************************************************************************/
// CANOpen Interface Functions
/*****************************************************************************/
extern CO_Status CANOpen_sendSync();
extern CO_Status CANOpen_sendTpdoRTR(uint8_t nodeId, uint8_t channel);

extern CO_Status CANOpen_NMT(CO_NMT state, uint8_t id);

extern CO_Status CANOpen_writeOD(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint8_t* data, uint8_t len, uint16_t timeout);
extern CO_Status CANOpen_writeOD_float(uint8_t nodeId, uint16_t Index, uint8_t subIndex, float data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_uint32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint32_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_int32(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int32_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_uint16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint16_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_int16(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int16_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_uint8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint8_t data, uint16_t timeout);
extern CO_Status CANOpen_writeOD_int8(uint8_t nodeId, uint16_t Index, uint8_t subIndex, int8_t data, uint16_t timeout);

extern CO_Status CANOpen_readOD(uint8_t nodeId, uint16_t Index, uint8_t subIndex, uint8_t* data, uint8_t* len, uint16_t timeout);

extern void CANOpen_mappingPDO_init(CO_PDOStruct* pdo_struct);
extern void CANOpen_mappingPDO_float(CO_PDOStruct* pdo_struct, float* data);
extern void CANOpen_mappingPDO_uint32(CO_PDOStruct* pdo_struct, uint32_t* data);
extern void CANOpen_mappingPDO_int32(CO_PDOStruct* pdo_struct, int32_t* data);
extern void CANOpen_mappingPDO_uint16(CO_PDOStruct* pdo_struct, uint16_t* data);
extern void CANOpen_mappingPDO_int16(CO_PDOStruct* pdo_struct, int16_t* data);
extern void CANOpen_mappingPDO_uint8(CO_PDOStruct* pdo_struct, uint8_t* data);
extern void CANOpen_mappingPDO_int8(CO_PDOStruct* pdo_struct, int8_t* data);

extern CO_Status CANOpen_sendPDO(uint8_t nodeId, uint8_t channel, CO_PDOStruct* pdo_struct);
extern CO_Status CANOpen_readPDO(uint8_t nodeId, uint8_t channel, CO_PDOStruct* pdo_struct, uint16_t timeout);

/*****************************************************************************/
// User Install Functions
/*****************************************************************************/
extern void CANOpen_init();
extern void CANOpen_addRxBuffer(uint16_t cobID, uint8_t* data);
extern void CANOpen_timerLoop();

#ifdef __cplusplus
}
#endif

#endif