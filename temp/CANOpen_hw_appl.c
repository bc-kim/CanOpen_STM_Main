#include "CANOpen_hw_appl.h"
#include "stm32f4xx_hal.h"
#include "CANOpen_var.h"

extern CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef   TxHeader;
uint32_t              TxMailbox;

void CANOpen_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len){
  
  TxHeader.StdId = cobID;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = len;
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
  while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox) == 1);

}

void CANOpen_RemoteRequest(uint16_t cobID)
{
  TxHeader.StdId = cobID;
  TxHeader.RTR = CAN_RTR_REMOTE;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 0;
  uint8_t data[8];
  data[0] = 0x00;
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    ;
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
  while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox) == 1)
    ;
}