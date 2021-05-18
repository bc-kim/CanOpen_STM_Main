#ifndef __CAN_BC_SDO_H__
#define __CAN_BC_SDO_H__

static uint8_t CS_SDO;
static uint8_t Index_SDO[2];
static uint8_t SubIndex_SDO;
static uint8_t Data_SDO[4];

int CAN_BC_SDO_CTR_Delay(uint8_t SDO_CTR, uint8_t SDO_CTR2, uint8_t node, uint8_t Data_SDO_extern[4], uint8_t Delay);
void CAN_BC_SDO(CAN_HandleTypeDef *hcan, uint8_t node, uint8_t Index_SDO[2], uint8_t SubIndex_SDO, uint8_t Data_SDO[4],uint8_t CS_SDO); // Reset a node and then make the node operational in NMT. 

#ifdef __cplusplus
#endif

#endif 