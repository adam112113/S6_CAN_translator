/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f3xx_hal.h"
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Heinzmann CAN message IDs
#define HEINZ_VOLT_CURR_ID  0x1AF0FE11
#define HEINZ_CAPACITY_ID   0x1AF0FC11
#define HEINZ_STATUS_ID     0x1AF0FB11
#define HEINZ_TEMPS_ID      0x1AF0F111
#define HEINZ_ERROR_ID      0x1AF0EE11

// Sample fallback values
#define DEFAULT_SOC         100
#define DEFAULT_SOH         100
#define DEFAULT_FULL_CAP    10000  // in mAh
#define DEFAULT_CHARGE_CYCLES 0

// GPIO pin for user interaction (e.g. PA0 with pull-up)
#define START_BUTTON_PIN GPIO_PIN_0
#define START_BUTTON_PORT GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rxHeader;
CAN_TxHeaderTypeDef txHeader;
uint8_t rxData[8];
uint8_t txData[8];
uint32_t txMailbox;

uint32_t emusBaseID = 0x180;

// volatile bool system_started = false;
uint32_t last_voltage_send = 0;
uint32_t last_capacity_send = 0;
uint32_t last_status_send = 0;
uint32_t last_temp_send = 0;
uint32_t last_error_send = 0;

// Dynamic data from EMUS status messages
uint8_t emus_status_byte = 0x00;
uint32_t emus_max_discharge_mA = 0;
uint16_t emus_max_charge_mA = 0;
int16_t emus_avgTemp = 0;
int16_t emus_maxTemp = 0;
int16_t emus_minTemp = 0;
uint16_t emus_errorCode = 0;
uint8_t emus_soc = 100;
uint8_t emus_soh = 100;
uint16_t emus_remCap = 0;
uint16_t emus_fullCap = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void send_heinzmann_voltage_current(uint16_t packV, uint16_t termV, int16_t curr);
void send_heinzmann_status(uint8_t statusByte, uint32_t dischargeMax, uint16_t chargeMax);
void send_heinzmann_capacity(uint16_t remCap, uint8_t soc, uint8_t soh, uint16_t fullCap, uint16_t cycles);
void send_heinzmann_temperature(int16_t avgTemp, int16_t maxTemp, int16_t minTemp);
void send_heinzmann_error(uint16_t errorCode);

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void safe_CAN_send(CAN_TxHeaderTypeDef* header, uint8_t* data);

void send_raw_echo_response(uint8_t *data, uint8_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); // toggle PA1
//	  HAL_Delay(500);                        // 500ms delay

//	  if (!system_started)
//	  {
//		  if (HAL_GPIO_ReadPin(START_BUTTON_PORT, START_BUTTON_PIN) == GPIO_PIN_RESET)
//		  {
//			  system_started = true;
//		  }
//	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

       canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
       canfilterconfig.FilterBank = 10;  // anything between 0 to SlaveStartFilterBank
       canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO1;
       canfilterconfig.FilterIdHigh = 0x0000;
       canfilterconfig.FilterIdLow = 0x0000;
       canfilterconfig.FilterMaskIdHigh = 0x0000;
       canfilterconfig.FilterMaskIdLow = 0x0000;
       canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
       canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;


       HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void send_heinzmann_voltage_current(uint16_t packV, uint16_t termV, int16_t curr)
{
    txHeader.IDE = CAN_ID_EXT;
    txHeader.ExtId = HEINZ_VOLT_CURR_ID;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 6;

    txData[0] = packV & 0xFF;
    txData[1] = (packV >> 8) & 0xFF;
    txData[2] = termV & 0xFF;
    txData[3] = (termV >> 8) & 0xFF;
    txData[4] = curr & 0xFF;
    txData[5] = (curr >> 8) & 0xFF;

    safe_CAN_send(&txHeader, txData);
//    HAL_StatusTypeDef txStatus = HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
//    if (txStatus != HAL_OK) {
//        // Transmission failed — turn ON LED
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//    }
//    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}

void send_heinzmann_capacity(uint16_t remCap, uint8_t soc, uint8_t soh, uint16_t fullCap, uint16_t cycles)
{
    txHeader.ExtId = HEINZ_CAPACITY_ID;
    txHeader.DLC = 8;

    txData[0] = remCap & 0xFF;
    txData[1] = (remCap >> 8) & 0xFF;
    txData[2] = soc;
    txData[3] = soh;
    txData[4] = fullCap & 0xFF;
    txData[5] = (fullCap >> 8) & 0xFF;
    txData[6] = cycles & 0xFF;
    txData[7] = (cycles >> 8) & 0xFF;

    safe_CAN_send(&txHeader, txData);
//    HAL_StatusTypeDef txStatus = HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
//    if (txStatus != HAL_OK) {
//        // Transmission failed — turn ON LED
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//    }
//    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}


void send_heinzmann_status(uint8_t statusByte, uint32_t dischargeMax, uint16_t chargeMax)
{
    txHeader.ExtId = HEINZ_STATUS_ID;
    txHeader.DLC = 8;

    txData[0] = statusByte;
    txData[1] = 0x00;
    txData[2] = dischargeMax & 0xFF;
    txData[3] = (dischargeMax >> 8) & 0xFF;
    txData[4] = (dischargeMax >> 16) & 0xFF;
    txData[5] = (dischargeMax >> 24) & 0xFF;
    txData[6] = chargeMax & 0xFF;
    txData[7] = (chargeMax >> 8) & 0xFF;

    safe_CAN_send(&txHeader, txData);
//    HAL_StatusTypeDef txStatus = HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
//    if (txStatus != HAL_OK) {
//        // Transmission failed — turn ON LED
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//    }
//    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}

void send_heinzmann_temperature(int16_t avgTemp, int16_t maxTemp, int16_t minTemp)
{
    txHeader.ExtId = HEINZ_TEMPS_ID;
    txHeader.DLC = 8;

    txData[0] = avgTemp & 0xFF;
    txData[1] = (avgTemp >> 8) & 0xFF;
    txData[2] = maxTemp & 0xFF;
    txData[3] = (maxTemp >> 8) & 0xFF;
    txData[4] = minTemp & 0xFF;
    txData[5] = (minTemp >> 8) & 0xFF;
    txData[6] = 0x00;
    txData[7] = 0x00;

    safe_CAN_send(&txHeader, txData);
//    HAL_StatusTypeDef txStatus = HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
//    if (txStatus != HAL_OK) {
//    	// Transmission failed — turn ON LED
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//    }
}

void send_heinzmann_error(uint16_t errorCode)
{
    txHeader.ExtId = HEINZ_ERROR_ID;
    txHeader.DLC = 8;

    txData[0] = errorCode & 0xFF;
    txData[1] = (errorCode >> 8) & 0xFF;
    txData[2] = 0x00;
    txData[3] = 0x00;
    txData[4] = 0x00;
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;

    safe_CAN_send(&txHeader, txData);
//    HAL_StatusTypeDef txStatus = HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
//    if (txStatus != HAL_OK) {
//    	// Transmission failed — turn ON LED
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//    }
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    uint32_t now = HAL_GetTick();

    if (rxHeader.IDE == CAN_ID_STD) {
        switch (rxHeader.StdId) {
            case 0x180:
                uint16_t voltage = (rxData[1] << 8) | rxData[0];
                int16_t current = (int16_t)((rxData[3] << 8) | rxData[2]);
                if ((int32_t)(now - last_voltage_send) >= 10) {
                    send_heinzmann_voltage_current(voltage * 10, voltage * 10, current * 10);
                    last_voltage_send = now;
                }
                break;

            case 0x186:
                emus_max_discharge_mA = ((rxData[5] << 8) | rxData[4]) * 100;
                emus_max_charge_mA = ((rxData[7] << 8) | rxData[6]) * 100;
                break;

            case 0x187:
                emus_remCap = (rxData[1] << 8) | rxData[0];
                emus_soc = rxData[2];
                emus_soh = rxData[3];
                emus_fullCap = (rxData[5] << 8) | rxData[4];
                if ((int32_t)(now - last_capacity_send) >= 100) {
                    send_heinzmann_capacity(emus_remCap, emus_soc, emus_soh, emus_fullCap, 0);
                    last_capacity_send = now;
                }
                break;

            case 0x188:
                emus_avgTemp = (int16_t)((rxData[1] << 8) | rxData[0]);
                emus_maxTemp = (int16_t)((rxData[3] << 8) | rxData[2]);
                emus_minTemp = (int16_t)((rxData[5] << 8) | rxData[4]);
                if ((int32_t)(now - last_temp_send) >= 100) {
                    send_heinzmann_temperature(emus_avgTemp, emus_maxTemp, emus_minTemp);
                    last_temp_send = now;
                }
                break;

            case 0x181:
                emus_status_byte = rxData[0];
                if ((int32_t)(now - last_status_send) >= 100) {
                    send_heinzmann_status(emus_status_byte, emus_max_discharge_mA, emus_max_charge_mA);
                    last_status_send = now;
                }
                break;

            case 0x18A:
                emus_errorCode = (rxData[1] << 8) | rxData[0];
                if ((int32_t)(now - last_error_send) >= 100) {
                    send_heinzmann_error(emus_errorCode);
                    last_error_send = now;
                }
                break;

            default:
                break;
        }
    }
}

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData);
//    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//
////    if (!system_started) return;
//
//    uint32_t now = HAL_GetTick();
//
//    if (rxHeader.IDE == CAN_ID_STD && rxHeader.StdId == emusBaseID) {
//
//        uint16_t emusVoltage = (rxData[1] << 8) | rxData[0]; // 0.01V
//        uint16_t emusCurrent = (rxData[3] << 8) | rxData[2]; // 0.1A
//
//        uint16_t packV = emusVoltage * 10;   // → 0.001V
//        int16_t curr = (int16_t)(emusCurrent * 10);  // → 0.01A
//
////        emus_avgTemp = (int16_t)((rxData[1] << 8) | rxData[0]);
////        emus_maxTemp = (int16_t)((rxData[3] << 8) | rxData[2]);
////        emus_minTemp = (int16_t)((rxData[5] << 8) | rxData[4]);
//
//        if (now - last_voltage_send >= 10)
//        {
//        	send_heinzmann_voltage_current(packV, packV, curr);
//        	last_voltage_send = now;
//        }
//
//        if (now - last_capacity_send >= 100)
//        {
//        	send_heinzmann_capacity(10000, DEFAULT_SOC, DEFAULT_SOH, DEFAULT_FULL_CAP, DEFAULT_CHARGE_CYCLES);
//        	last_capacity_send = now;
//        }
//
//        if (now - last_status_send >= 100) {
//            send_heinzmann_status(emus_status_byte, emus_max_discharge_mA, emus_max_charge_mA);
//            last_status_send = now;
//        }
//
////        if (now - last_temp_send >= 100) {
////            send_heinzmann_temperature(emus_avgTemp, emus_maxTemp, emus_minTemp);
////            last_temp_send = now;
////        }
//
//        if (now - last_error_send >= 100) {
//            send_heinzmann_error(emus_errorCode);
//            last_error_send = now;
//        }
//
//    	// For debugging purposes
////        send_raw_echo_response(rxData, rxHeader.DLC);
//    }
//
//    if (rxHeader.IDE == CAN_ID_EXT && rxHeader.ExtId == 0x18FF51E5) {
//        emus_status_byte = rxData[0];
//    }
//
//    if (rxHeader.IDE == CAN_ID_EXT && rxHeader.ExtId == 0x18FF50E5) {
//        uint16_t maxDischarge = (rxData[5] << 8) | rxData[4];
//        uint16_t maxCharge = (rxData[7] << 8) | rxData[6];
//
//        emus_max_discharge_mA = maxDischarge * 100;
//        emus_max_charge_mA = maxCharge * 100;
//    }
//
//    if (rxHeader.IDE == CAN_ID_STD && rxHeader.StdId == (emusBaseID + 8))
//    {
//    	emus_avgTemp = (int16_t)((rxData[1] << 8) | rxData[0]);
//    	emus_maxTemp = (int16_t)((rxData[3] << 8) | rxData[2]);
//    	emus_minTemp = (int16_t)((rxData[5] << 8) | rxData[4]);
//
//    	if ((int32_t)(now - last_temp_send) >= 100)
//    	{
//    		send_heinzmann_temperature(emus_avgTemp, emus_maxTemp, emus_minTemp);
//    		last_temp_send = now;
//    	}
//    }
//
//    if (rxHeader.IDE == CAN_ID_EXT && rxHeader.ExtId == 0x18FF53E5) {
//        emus_errorCode = (rxData[1] << 8) | rxData[0];
//    }
//}

void safe_CAN_send(CAN_TxHeaderTypeDef* header, uint8_t* data)
{
    HAL_StatusTypeDef result;
    uint32_t timeout = HAL_GetTick() + 10;  // 10 ms timeout window

    // Wait for at least one free mailbox
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        if (HAL_GetTick() > timeout)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // indicate mailbox congestion
            return;
        }
    }

    // Attempt to send
    result = HAL_CAN_AddTxMessage(&hcan, header, data, &txMailbox);
    if (result != HAL_OK)
    {
        // Transmission failed for other reasons
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    }
}
//void safe_CAN_send(CAN_TxHeaderTypeDef* header, uint8_t* data)
//{
//    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
//    {
//        HAL_CAN_AddTxMessage(&hcan, header, data, &txMailbox);
//    }
//    else
//    {
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//    }
//}

// This function is only for debugging the CAN bus
void send_raw_echo_response(uint8_t *data, uint8_t length)
{
	 txHeader.IDE = CAN_ID_EXT;
	 txHeader.ExtId = 0x1AF0F000;  // Dummy test ID (can change)
	 txHeader.RTR = CAN_RTR_DATA;
	 txHeader.DLC = length;

	 // Copy rxData to txData
	 for (uint8_t i = 0; i < length; i++)
	 {
		 txData[i] = data[i];
	 }

	 if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK)
	 {
		 Error_Handler();  // or toggle LED to show error
	 }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
