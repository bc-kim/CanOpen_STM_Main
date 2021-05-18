#ifndef __CAN_BC_NMT_H__
#define __CAN_BC_NMT_H__

static uint8_t CS_NMT;
static void BC_CAN_Filter_Init(void);
int CAN_BC_NMT(CAN_HandleTypeDef *hcan, uint8_t node, uint8_t cs); // Reset a node and then make the node operational in NMT. 
int CAN_BC_NMT_CTR(uint8_t NMT_CTR, uint8_t node);
int CAN_BC_NMT_CTR_Delay(uint8_t NMT_CTR, uint8_t node, uint8_t Delay);
void CAN_BC_Send_ADC(void); // Send Loadcell value.

#ifdef __cplusplus
#endif

#endif 