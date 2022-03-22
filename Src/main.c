/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CANOPEN_var.h"
#include "CANOpen.h"
#include "math.h"
#include "CANOpen_batch.h"
#include "CAN_HLC.h"
#include "CAN_NMT.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
CO_PDOStruct Tpdo1_N1;
CO_PDOStruct Tpdo2_N1;
CO_PDOStruct Tpdo3_N1;
CO_PDOStruct Tpdo4_N1;
CO_PDOStruct Rpdo1_N1;
CO_PDOStruct Rpdo2_N1;
CO_PDOStruct Rpdo3_N1;
CO_PDOStruct Rpdo4_N1;

CO_MOTOR motor_[4];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // HAL_Can interrupt
{
  uint8_t node;
  if (hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
    if (RxHeader.StdId == 0x51) // Save the motion profile
    {
    }

    else if (RxHeader.StdId > 0x480 && RxHeader.StdId < 0x500) // PDO#4
    {
      node = RxHeader.StdId - 0x480;
      memcpy(&Pos_check[node - 1], &RxData[0], 4);
      // Pos_limit_flag[node-1] = Pos_Limit(Pos_check[node-1],Pos_lbound_hard[node-1],Pos_ubound_hard[node-1]);
      if (Pos_check[node - 1] < Pos_lbound_hard[node - 1])
      {
        Pos_limit_flag[node - 1] = 1;
      }
      else if (Pos_check[node - 1] > Pos_ubound_hard[node - 1])
      {
        Pos_limit_flag[node - 1] = 2;
      }
      CANOpen_addRxBuffer(RxHeader.StdId, RxData);
    }
    else if (RxHeader.StdId > 0x180 && RxHeader.StdId < 0x200)
    {
      node = RxHeader.StdId - 0x180;
      motor_[node - 1].PDO_Status = PDO_CV_Received;
      CANOpen_addRxBuffer(RxHeader.StdId, RxData);
    }

    else if (RxHeader.StdId > 0x280 && RxHeader.StdId < 0x300)
    {
      node = RxHeader.StdId - 0x280;
      motor_[node - 1].PDO_Status = PDO_CV_Received;
      CANOpen_addRxBuffer(RxHeader.StdId, RxData);
    }

    else if (RxHeader.StdId > 0x380 && RxHeader.StdId < 0x400)
    {
      node = RxHeader.StdId - 0x380;
      motor_[node - 1].PDO_Status = PDO_CV_Received;
      CANOpen_addRxBuffer(RxHeader.StdId, RxData);
    }

    else
    {
      CANOpen_addRxBuffer(RxHeader.StdId, RxData);
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    CANOpen_timerLoop();
  }
  else if (htim == &htim3)
  {
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13) // Blue switch
  {
    Input = Button1;
  }

  else if (GPIO_Pin == GPIO_PIN_15) // Lower switch (Far from STM)
  {
    Input = Button2;
    RealTime = 0; // Stop the RT control mode.
  }

  else if (GPIO_Pin == GPIO_PIN_12) // Upper switch (close to STM)
  {
    Input = Button3;
    RealTime = 0; // Stop the RT control mode.
  }
}

uint8_t UserButton = 0;
uint8_t j = 1;
// static uint8_t j = 1;를 밖에 빼서 확인용.
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint8_t i;
  uint8_t adc_read = 1;
  uint8_t Flag_RT = 0;
  static int32_t DesiredValue[4] = {0,0,0,0};
  static int32_t DesiredValue_prev[4] = {0,0,0,0};

  uint8_t Automotive = 1; // if automotive is 1, the motor is controlled automatically else the motor is controlled manually.

  if (hadc == &hadc1)
  {
    adcValue[0] = ADC1->JDR1;
    adcValue[1] = ADC2->JDR1;
    adcValue[2] = ADC3->JDR1;

    memcpy(&Force_CO[0], &adcValue[0], 2);
    memcpy(&Force_CO[2], &adcValue[1], 2);
    memcpy(&Force_CO[4], &adcValue[2], 2);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    Flag_RT = 1;
    if (Input == RT_on)
    {
      CANOpen_sendFrame(0x51, Force_CO, 6, 0); // SendForceData
      
      for (i = 1; i < 5; i++)
      {
        // 1.1 UpdateDesiredCM/Values in automotive case
        if (Automotive)
        {
          if (motor_[i - 1].PDO_Status == PDO_DV_Sent)
          {
            // 1.1.1 SetControlMode
            Con_Mode_Prev[i - 1] = Con_Mode[i - 1];
            Con_Mode[i - 1] = motor_[i - 1].Con_Mode[j - 1];
            // 1.1.2 SetDesiredValue
            if (Con_Mode[i - 1] == Admittance)
            {
              Desired_A[i - 1] = motor_[i - 1].DesiredForce[j - 1];
              Tension_error[i - 1] = (float)(Desired_A[i - 1] - adcValue[adc_read]);
              DesiredValue_prev[i - 1] = DesiredValue[i - 1];
              DesiredValue[i - 1] = (int32_t)(kp[i - 1] * Tension_error[i - 1] + kd[i - 1] * (Tension_error[i - 1] - Tension_error_before[i - 1]));
              memcpy(&Tension_error_before[i - 1], &Tension_error[i - 1], 4);
            }
            else
            {
              DesiredValue_prev[i - 1] = DesiredValue[i - 1];
              DesiredValue[i - 1] = motor_[i - 1].DesiredValue[j - 1];
//              DesiredValue_Global[i - 1] = motor_[i - 1].DesiredValue[j - 1];
            }
            motor_[i - 1].PDO_Status = PDO_DV_Updated;
          }
        }

        // 1.2 UpdateDesiredCM/Values in manual case
        else if (Automotive == 0)
        {
          if (UserButton == 1)
          {
            // 1.2.1 SetControlMode
            Con_Mode_Prev[i - 1] = Con_Mode[i - 1];
            Con_Mode[i - 1] = Con_Mode_input[i - 1];
            if (i == 4)
            {
              UserButton = 0;
            }
            // 1.2.2 SetDesiredValue
            if (Con_Mode[i - 1] == Admittance)
            {
              Desired_A[i - 1] = Desired_input_A[i - 1];
              Tension_error[i - 1] = (float)(Desired_input_A[i - 1] - adcValue[adc_read]);
              DesiredValue_prev[i - 1] = DesiredValue[i - 1];
              DesiredValue[i - 1] = (int32_t)(kp[i - 1] * Tension_error[i - 1] + kd[i - 1] * (Tension_error[i - 1] - Tension_error_before[i - 1]));
              memcpy(&Tension_error_before[i - 1], &Tension_error[i - 1], 4);
            }
            else
            {
              Desired_PV[i - 1] = Desired_input_PV[i - 1];
              DesiredValue_prev[i - 1] = DesiredValue[i - 1];
              DesiredValue[i - 1] = Desired_PV[i - 1];
            }
            motor_[i - 1].PDO_Status = PDO_DV_Updated;
          }
        }

        if (motor_[i - 1].PDO_Status == PDO_DV_Updated)
        {
          // 2. CANSetControlMode
          if (j == 1)
          {
            CAN_Set_ControlMode(Con_Mode[i - 1], Node[i - 1]);
          }
          else 
          {
            if (Con_Mode[i - 1] != Con_Mode_Prev[i - 1])
            {
              CAN_Set_ControlMode(Con_Mode[i - 1], Node[i - 1]); //
            }
          }

          // 3. AskCurrentValue
          CAN_Ask_CurrentValue(Con_Mode[i - 1], Node[i - 1]); // Ask the current value until the motor reaches to the target value.
          motor_[i - 1].PDO_Status = PDO_CV_Waiting;
        }

        // 4. Check the convergence

        if (motor_[i - 1].PDO_Status == PDO_CV_Received || motor_[i - 1].PDO_Status == PDO_CV_NotConverged)
        {
          if (j == 1)
          {
            motor_[i - 1].PDO_Status = PDO_CV_Converged;
          }
          else
          {
            motor_[i - 1].PDO_Status = CAN_Check_Convergence(Con_Mode[i - 1], Node[i - 1], DesiredValue_prev[i - 1], &motor_[i - 1]);
          }
        }
        CAN_Send_DesiredValue(Con_Mode[i - 1], Node[i - 1], DesiredValue[i - 1], &motor_[i - 1]);
        motor_[i - 1].PDO_Status = PDO_DV_Sent;
        // 5. SendDesiredValue
        if (motor_[0].PDO_Status == PDO_CV_Converged && motor_[1].PDO_Status == PDO_CV_Converged && motor_[2].PDO_Status == PDO_CV_Converged && motor_[3].PDO_Status == PDO_CV_Converged)
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
          j++;
        }
      }
      if (Flag_RT)
      {
        CANOpen_sendSync();
      }
    }
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // [[ 1. Init CAN ]]
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  // [[ 2. Init CAN timerloop ]]
  HAL_TIM_Base_Start_IT(&htim2);

  // [[ 3. Init PDO mapping ]]
  CANOpen_init();
  for (uint8_t i = 0; i < 4; i++)
  {
    CANOpen_mappingPDO_init(&motor_[i].TPDO[0]);
    CANOpen_mappingPDO_int32(&motor_[i].TPDO[0], &Pos[i]);

    CANOpen_mappingPDO_init(&motor_[i].TPDO[1]);
    CANOpen_mappingPDO_int32(&motor_[i].TPDO[1], &Vel[i]);

    CANOpen_mappingPDO_init(&motor_[i].TPDO[2]);
    CANOpen_mappingPDO_int16(&motor_[i].TPDO[2], &Torque[i]);

    CANOpen_mappingPDO_init(&motor_[i].TPDO[3]);
    CANOpen_mappingPDO_int32(&motor_[i].TPDO[3], &Pos_check[i]);
    CANOpen_mappingPDO_uint16(&motor_[i].TPDO[3], &StatusWord[i]);

    CANOpen_mappingPDO_init(&motor_[i].RPDO[0]);
    CANOpen_mappingPDO_uint16(&motor_[i].RPDO[0], &ControlWord[i]);

    CANOpen_mappingPDO_init(&motor_[i].RPDO[1]);
    CANOpen_mappingPDO_int32(&motor_[i].RPDO[1], &Target_Pos[i]);

    CANOpen_mappingPDO_init(&motor_[i].RPDO[2]);
    CANOpen_mappingPDO_int32(&motor_[i].RPDO[2], &Target_Vel[i]);

    CANOpen_mappingPDO_init(&motor_[i].RPDO[3]);
    CANOpen_mappingPDO_int16(&motor_[i].RPDO[3], &Target_Tor[i]);

  }
  memcpy(&motor_[0].Con_Mode[0], &Con_ModePresetM1[0], 40);
  memcpy(&motor_[0].DesiredForce[0], &DesiredForcePresetM1[0], 20);
  memcpy(&motor_[0].DesiredValue[0], &DesiredValuePresetM1[0], 40);

  memcpy(&motor_[1].Con_Mode[0], &Con_ModePresetM2[0], 40);
  memcpy(&motor_[1].DesiredForce[0], &DesiredForcePresetM2[0], 20);
  memcpy(&motor_[1].DesiredValue[0], &DesiredValuePresetM2[0], 40);

  memcpy(&motor_[2].Con_Mode[0], &Con_ModePresetM3[0], 40);
  memcpy(&motor_[2].DesiredForce[0], &DesiredForcePresetM3[0], 20);
  memcpy(&motor_[2].DesiredValue[0], &DesiredValuePresetM3[0], 40);

  memcpy(&motor_[3].Con_Mode[0], &Con_ModePresetM4[0], 40);
  memcpy(&motor_[3].DesiredForce[0], &DesiredForcePresetM4[0], 20);
  memcpy(&motor_[3].DesiredValue[0], &DesiredValuePresetM4[0], 40);

  Ctrl_Mode = Profile_vel;

  // [[4 . Start Main timer ]]
  HAL_TIM_Base_Start(&htim1);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);
  HAL_ADCEx_InjectedStart(&hadc3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (!RealTime)
    {
      switch (Input)
      {
      case Init:
        // CO_status = Init_CAN(100);
        if (Init_CAN(50) == CO_OK)
        {
          Input = Init_state;
          break;
        }
      case Button1: // Blue switch : Start RT control
        // Enter_RT_CAN(Profile_vel, 50);
        Input = RT_on;
        break;
      case Button2: // Lower switch (close to STM) //
        for (uint8_t i = 0; i < 4; i++)
        {
          CAN_Set_ControlMode(Cyclic_sync_pos, Node[i]);       // Set to the position control mode
          CAN_Set_TargetValue(Cyclic_sync_pos, 0x00, Node[i]); // Go to initial position.
        }
        Input = Init_state;
        break;
      case Button3: // Upper switch (Far from STM) -- Emergency Reset : Stop the controller
        Reset_MotorDriver(50);
        Input = Init_state;
        break;
      }
    }
    for (uint8_t i = 0; i < 4; i++)
    {
      if (Pos_limit_flag[i] == 1) // lbound
      {
        CAN_Set_ControlMode(Cyclic_sync_pos, Node[i - 1]);                    // 1
        CAN_Set_TargetValue(Cyclic_sync_pos, Pos_lbound[i - 1], Node[i - 1]); // Go to initial position.
        CAN_Set_ControlMode(Con_Mode[i - 1], Node[i - 1]);                    // 1
      }
      else if (Pos_limit_flag[i] == 2) // ubound
      {
        CAN_Set_ControlMode(Cyclic_sync_pos, i);                              // 2
        CAN_Set_TargetValue(Cyclic_sync_pos, Pos_ubound[i - 1], Node[i - 1]); // Go to initial position.
        CAN_Set_ControlMode(Con_Mode[i - 1], Node[i - 1]);                    // 1
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
   */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
   */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
   */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 14999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 119;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 44999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
/*extern void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}*/
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
