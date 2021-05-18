#ifndef __CAN_BC_PDO_H__
#define __CAN_BC_PDO_H__

static uint8_t CS_PDO;
static uint8_t Index_PDO[2];
static uint8_t SubIndex_PDO;
static uint8_t Data_PDO[4];

int CAN_BC_PDO(CAN_HandleTypeDef* hcan, uint8_t node, uint8_t Data_PDO[4]);
int CAN_BC_PDO_CTR(uint8_t PDO_CTR, uint8_t Operationmode, uint8_t node, uint8_t Data_PDO_extern[4]); 
void BC_Hex_array(int32_t Data_input);
void CAN_BC_Sync(void);
void CAN_BC_Motor_Control(void);
void CAN_Save_Load(void);
void Motor_BC_Control(uint8_t number_of_node, uint8_t Delay);

#ifdef __cplusplus
#endif

#endif 