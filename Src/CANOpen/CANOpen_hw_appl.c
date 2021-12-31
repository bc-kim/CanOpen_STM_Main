#include "CANOpen_hw_appl.h"

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "CANOpen_var.h"
/* USER CODE END Includes */

/* Private typedef */
/* USER CODE BEGIN PD */
extern CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef   TxHeader;
uint32_t              TxMailbox;
/* USER CODE END PD */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */  


void CANOpen_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len, uint8_t rtr){

  /* USER CODE BEGIN 1 */
  TxHeader.StdId = cobID;
  TxHeader.RTR = rtr == 1 ? CAN_RTR_REMOTE : CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = len;
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
  while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox) == 1);
  /* USER CODE END 1 */  

}

int CANOpen_mutexLock(){

  /* USER CODE BEGIN 2 */
  return 0;
  /* USER CODE END 2 */  

}

int CANOpen_mutexUnlock(){

  /* USER CODE BEGIN 3 */
  return 0;
  /* USER CODE END 3 */  

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */ 