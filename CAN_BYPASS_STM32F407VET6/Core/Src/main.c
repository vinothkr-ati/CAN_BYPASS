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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


//*********     CAN 1    ************************************************************

CAN_RxHeaderTypeDef CAN1RX_Header;
uint8_t CAN1_Queue_TX[14];
uint8_t CAN1RX_DATA[8];
volatile uint32_t  CAN1RX_ID_Queue = 0;
volatile uint32_t  CAN1RX_IDE_DATA = 0;
volatile uint32_t  CAN1RX_DLC_DATA = 0;
volatile uint32_t  CAN1RX_Std_ID_DATA = 0;
volatile uint32_t  CAN1RX_Ext_ID_DATA = 0;
volatile uint32_t  Last_Drive_Command_Tick = 0;  //  default should be 1 important


//**********    CAN 2     **********************************************************
#define CAN2TX_StdID 500
uint8_t CAN2TX[8];
CAN_RxHeaderTypeDef CAN2RX_Header;
uint8_t CAN2_Queue_TX[14];

volatile uint8_t CAN2RX_DATA[8];
volatile uint32_t  CAN2RX_ID_Queue = 0;
volatile uint32_t  CAN2RX_IDE_DATA = 0;
volatile uint32_t  CAN2RX_DLC_DATA = 0;
volatile uint32_t  CAN2RX_Std_ID_DATA = 0;
volatile uint32_t  CAN2RX_Ext_ID_DATA = 0;
// CAN2 RX Timeout
volatile uint8_t  CAN2_Timeout_Flag = 0;  //
volatile uint32_t Last_CAN2_RX_Tick = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* Definitions for CAN1RX_Data_Pro */
osThreadId_t CAN1RX_Data_ProHandle;
const osThreadAttr_t CAN1RX_Data_Pro_attributes = {
  .name = "CAN1RX_Data_Pro",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN2RX_Data_Pro */
osThreadId_t CAN2RX_Data_ProHandle;
const osThreadAttr_t CAN2RX_Data_Pro_attributes = {
  .name = "CAN2RX_Data_Pro",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN1RX_Data_Queue */
osMessageQueueId_t CAN1RX_Data_QueueHandle;
const osMessageQueueAttr_t CAN1RX_Data_Queue_attributes = {
  .name = "CAN1RX_Data_Queue"
};
/* Definitions for CAN2RX_Data_Queue */
osMessageQueueId_t CAN2RX_Data_QueueHandle;
const osMessageQueueAttr_t CAN2RX_Data_Queue_attributes = {
  .name = "CAN2RX_Data_Queue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
void CAN1RX_Data_Process_Func(void *argument);
void CAN2RX_Data_Process_Func(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


HAL_StatusTypeDef CAN_TransmitMessage_Std_ID(CAN_HandleTypeDef *hcan,uint32_t Std_ID, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox = 0;
    HAL_StatusTypeDef status;

    // Configure TxHeadervolatile uint16_t  Left_End_Stop_Steering  = 0;
    volatile uint16_t  Right_End_Stop_Steering = 0;
    TxHeader.StdId = Std_ID;           // Standard Identifier

    TxHeader.IDE = CAN_ID_STD;        // Standard ID

    TxHeader.RTR = CAN_RTR_DATA;      // Data frame
    TxHeader.DLC = len;               // Data Length Code
    TxHeader.TransmitGlobalTime = DISABLE;

    // Request transmission
    status = HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);

    if (status != HAL_OK) {
        // Transmission request failed
        return status;
    }

    // Wait until the transmission is complete
    while (HAL_CAN_IsTxMessagePending(hcan, TxMailbox));

    // Transmission successful, mailbox is freed automatically

    return HAL_OK;
}




/*
 * STD ID
 */
HAL_StatusTypeDef CAN_TransmitMessage_Ext_ID(CAN_HandleTypeDef *hcan, uint32_t Ext_ID, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox = 0;
    HAL_StatusTypeDef status;

    // Configure TxHeader
            // Standard Identifier
    TxHeader.ExtId = Ext_ID;            // Extended Identifier (not used in this case)

    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;      // Data frame
    TxHeader.DLC = len;               // Data Length Code
    TxHeader.TransmitGlobalTime = DISABLE;

    // Request transmission
    status = HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);

    if (status != HAL_OK) {
        // Transmission request failed
        return status;
    }

    // Wait until the transmission is complete
    while (HAL_CAN_IsTxMessagePending(hcan, TxMailbox));

    // Transmission successful, mailbox is freed automatically
    return HAL_OK;
}




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{


    if (hcan->Instance == CAN1)
       {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1RX_Header, CAN1RX_DATA);
            // Process CAN1 Data here
            CAN1_Queue_TX[0] = CAN1RX_DATA[0];
			CAN1_Queue_TX[1] = CAN1RX_DATA[1];
			CAN1_Queue_TX[2] = CAN1RX_DATA[2];
			CAN1_Queue_TX[3] = CAN1RX_DATA[3];

			CAN1_Queue_TX[4] = CAN1RX_DATA[4];
			CAN1_Queue_TX[5] = CAN1RX_DATA[5];
			CAN1_Queue_TX[6] = CAN1RX_DATA[6];
			CAN1_Queue_TX[7] = CAN1RX_DATA[7];

			//		CAN1_RX_DLC = RxHeader.DLC;

			CAN1RX_IDE_DATA = CAN1RX_Header.IDE;
			if(CAN1RX_IDE_DATA == 0) {
				CAN1RX_Std_ID_DATA = CAN1RX_Header.StdId;
				CAN1_Queue_TX[8]  =  (CAN1RX_Std_ID_DATA >> 0)  & 0xFF;
				CAN1_Queue_TX[9]  =  (CAN1RX_Std_ID_DATA >> 8)  & 0xFF;
				CAN1_Queue_TX[10] =  (CAN1RX_Std_ID_DATA >> 16) & 0xFF;
				CAN1_Queue_TX[11] =  (CAN1RX_Std_ID_DATA >> 24) & 0xFF;
			}else {
				CAN1RX_Ext_ID_DATA = CAN1RX_Header.ExtId;
				CAN1_Queue_TX[8] =  (CAN1RX_Ext_ID_DATA >> 0)  & 0xFF;
				CAN1_Queue_TX[9] = (CAN1RX_Ext_ID_DATA >> 8)  & 0xFF;
				CAN1_Queue_TX[10] = (CAN1RX_Ext_ID_DATA >> 16) & 0xFF;
				CAN1_Queue_TX[11] = (CAN1RX_Ext_ID_DATA >> 24) & 0xFF;
			}
			CAN1_Queue_TX[12] = (uint8_t)CAN1RX_IDE_DATA;// IDE
			CAN1_Queue_TX[13] = (uint8_t)CAN1RX_Header.DLC;;// DLC
			 // Attempt to send data to the queue in a non-blocking way
			osStatus_t status = osMessageQueuePut(CAN1RX_Data_QueueHandle, CAN1_Queue_TX, 0, 0);  // No timeout, no blocking
			if (status != osOK)
			{
			  // Queue full, data not sent; could handle this or log it
			}


       }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	 if (hcan->Instance == CAN2)
	 {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2RX_Header, CAN2RX_DATA);
		// Process CAN2 Data here
		CAN2_Queue_TX[0] = CAN2RX_DATA[0];
		CAN2_Queue_TX[1] = CAN2RX_DATA[1];
		CAN2_Queue_TX[2] = CAN2RX_DATA[2];
		CAN2_Queue_TX[3] = CAN2RX_DATA[3];

		CAN2_Queue_TX[4] = CAN2RX_DATA[4];
		CAN2_Queue_TX[5] = CAN2RX_DATA[5];
		CAN2_Queue_TX[6] = CAN2RX_DATA[6];
		CAN2_Queue_TX[7] = CAN2RX_DATA[7];

		//		CAN1_RX_DLC = RxHeader.DLC;

		CAN2RX_IDE_DATA = CAN2RX_Header.IDE;
		if(CAN2RX_IDE_DATA == 0) {
			CAN2RX_Std_ID_DATA = CAN2RX_Header.StdId;
			CAN2_Queue_TX[8]  =  (CAN2RX_Std_ID_DATA >> 0)  & 0xFF;
			CAN2_Queue_TX[9]  =  (CAN2RX_Std_ID_DATA >> 8)  & 0xFF;
			CAN2_Queue_TX[10] =  (CAN2RX_Std_ID_DATA >> 16) & 0xFF;
			CAN2_Queue_TX[11] =  (CAN2RX_Std_ID_DATA >> 24) & 0xFF;
		}else {
			CAN2RX_Ext_ID_DATA = CAN2RX_Header.ExtId;
			CAN2_Queue_TX[8] =  (CAN2RX_Ext_ID_DATA >> 0)  & 0xFF;
			CAN2_Queue_TX[9] =  (CAN2RX_Ext_ID_DATA >> 8)  & 0xFF;
			CAN2_Queue_TX[10] = (CAN2RX_Ext_ID_DATA >> 16) & 0xFF;
			CAN2_Queue_TX[11] = (CAN2RX_Ext_ID_DATA >> 24) & 0xFF;
		}
		CAN2_Queue_TX[12] = (uint8_t)CAN2RX_IDE_DATA;// IDE
		CAN2_Queue_TX[13] = (uint8_t)CAN2RX_Header.DLC;;// DLC
		 // Attempt to send data to the queue in a non-blocking way
		osStatus_t status = osMessageQueuePut(CAN2RX_Data_QueueHandle, CAN2_Queue_TX, 0, 0);  // No timeout, no blocking
		if (status != osOK)
		{
		  // Queue full, data not sent; could handle this or log it
		}

		//
		// Reset timeout timer
		Last_CAN2_RX_Tick = HAL_GetTick();
						CAN2_Timeout_Flag = 0;

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */


	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);



	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//  this now
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);//  this now
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CAN1RX_Data_Queue */
  CAN1RX_Data_QueueHandle = osMessageQueueNew (50, sizeof(CAN1_Queue_TX), &CAN1RX_Data_Queue_attributes);

  /* creation of CAN2RX_Data_Queue */
  CAN2RX_Data_QueueHandle = osMessageQueueNew (50, sizeof(CAN2_Queue_TX), &CAN2RX_Data_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CAN1RX_Data_Pro */
  CAN1RX_Data_ProHandle = osThreadNew(CAN1RX_Data_Process_Func, NULL, &CAN1RX_Data_Pro_attributes);

  /* creation of CAN2RX_Data_Pro */
  CAN2RX_Data_ProHandle = osThreadNew(CAN2RX_Data_Process_Func, NULL, &CAN2RX_Data_Pro_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef can1FilterConfig;
   can1FilterConfig.FilterBank = 0;                           // Bank 0 for CAN1
   can1FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   can1FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   can1FilterConfig.FilterIdHigh = 0x0000;
   can1FilterConfig.FilterIdLow = 0x0000;
   can1FilterConfig.FilterMaskIdHigh = 0x0000;
   can1FilterConfig.FilterMaskIdLow = 0x0000;
   can1FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  // ⬅️ FIFO0 for CAN1
   can1FilterConfig.FilterActivation = ENABLE;
   can1FilterConfig.SlaveStartFilterBank = 14;                // Split point for CAN2
   HAL_CAN_ConfigFilter(&hcan1, &can1FilterConfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */


  CAN_FilterTypeDef can2FilterConfig;
  can2FilterConfig.FilterBank = 14;                          // Bank 14 for CAN2
  can2FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  can2FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  can2FilterConfig.FilterIdHigh = 0x0000;
  can2FilterConfig.FilterIdLow = 0x0000;
  can2FilterConfig.FilterMaskIdHigh = 0x0000;
  can2FilterConfig.FilterMaskIdLow = 0x0000;
  can2FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;  // ⬅️ FIFO1 for CAN2
  can2FilterConfig.FilterActivation = ENABLE;
  can2FilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &can2FilterConfig);           // ⬅️ Use hcan1 always in CAN2
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_CAN1RX_Data_Process_Func */
/**
  * @brief  Function implementing the CAN1RX_Data_Pro thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CAN1RX_Data_Process_Func */
void CAN1RX_Data_Process_Func(void *argument)
{
  /* USER CODE BEGIN 5 */

	uint8_t Received_CAN1RX[14];
	osStatus_t Status_CAN1;

  /* Infinite loop */
  for(;;)
  {
	  // Wait for and receive data from the queue
			  Status_CAN1 = osMessageQueueGet(CAN1RX_Data_QueueHandle, Received_CAN1RX, NULL, osWaitForever);
			  if (Status_CAN1 == osOK)  {
				 // Process Received_CAN1RX data
				 // Indicate processing by toggling an LED, etc.

				 // Reconstruct uint32_t from bb (Little-endian format)
				CAN1RX_ID_Queue = 	( Received_CAN1RX[8]  <<  0 ) |
									( Received_CAN1RX[9]  <<  8 ) |
									( Received_CAN1RX[10] << 16 ) |
									( Received_CAN1RX[11] << 24 );



				uint8_t CAN1_Data[8] = {Received_CAN1RX[0],Received_CAN1RX[1],Received_CAN1RX[2],Received_CAN1RX[3],
						Received_CAN1RX[4],Received_CAN1RX[5],Received_CAN1RX[6],Received_CAN1RX[7]};


				if(Received_CAN1RX[12] == 0)  {
					if (CAN_TransmitMessage_Std_ID(&hcan2, CAN1RX_ID_Queue, CAN1_Data, Received_CAN1RX[13]) == HAL_OK)
					{
					// Transmission successful
					// HAL_UART_Transmit(&huart1, CAN_ACK, 3,100);
					}
					else
					{
					// Transmission failed
					}
				} else {
						if (CAN_TransmitMessage_Ext_ID(&hcan2, CAN1RX_ID_Queue, CAN1_Data, Received_CAN1RX[13]) == HAL_OK)
						{
						// Transmission successful
						// HAL_UART_Transmit(&huart1, CAN_ACK, 3,100);
						}
						else
						{
						// Transmission failed
						}
				}

			 }

		     osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CAN2RX_Data_Process_Func */
/**
* @brief Function implementing the CAN2RX_Data_Pro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN2RX_Data_Process_Func */
void CAN2RX_Data_Process_Func(void *argument)
{
  /* USER CODE BEGIN CAN2RX_Data_Process_Func */

	 uint8_t Received_CAN2RX[14];
				 osStatus_t Status_CAN2;
  /* Infinite loop */
  for(;;)
  {
	  // Wait for and receive data from the queue
	 	 	 Status_CAN2 = osMessageQueueGet(CAN2RX_Data_QueueHandle, Received_CAN2RX, NULL, osWaitForever);
	 	 	 if (Status_CAN2 == osOK)  {
	 	 		 // Process Received_CAN1RX data
	 	 		 // Indicate processing by toggling an LED, etc.

	 	 		 // Reconstruct uint32_t from bb (Little-endian format)
	 	 		CAN2RX_ID_Queue =   (Received_CAN2RX[8] << 0)  |
	 	 							(Received_CAN2RX[9] << 8)  |
	 	 							(Received_CAN2RX[10] << 16) |
	 	 							(Received_CAN2RX[11] << 24);

	 	 		uint8_t CAN2_Data[8] = {Received_CAN2RX[0],Received_CAN2RX[1],Received_CAN2RX[2],Received_CAN2RX[3],
	 	 							Received_CAN2RX[4],Received_CAN2RX[5],Received_CAN2RX[6],Received_CAN2RX[7]};


	 	 					if(Received_CAN2RX[12] == 0)  {
	 	 						if (CAN_TransmitMessage_Std_ID(&hcan1, CAN2RX_ID_Queue, CAN2_Data, Received_CAN2RX[13]) == HAL_OK)
	 	 						{
	 	 						// Transmission successful
	 	 						// HAL_UART_Transmit(&huart1, CAN_ACK, 3,100);
	 	 						}
	 	 						else
	 	 						{
	 	 						// Transmission failed
	 	 						}
	 	 					} else {
	 	 							if (CAN_TransmitMessage_Ext_ID(&hcan1, CAN2RX_ID_Queue, CAN2_Data, Received_CAN2RX[13]) == HAL_OK)
	 	 							{
	 	 							// Transmission successful
	 	 							// HAL_UART_Transmit(&huart1, CAN_ACK, 3,100);
	 	 							}
	 	 							else
	 	 							{
	 	 							// Transmission failed
	 	 							}
	 	 					}


	 	 	 }
	 	     osDelay(1);
  }
  /* USER CODE END CAN2RX_Data_Process_Func */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
