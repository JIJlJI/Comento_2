/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pmic_mp5475.h"
#include "eeprom_25lc256.h"
#include "CAN_task.h"
#include "UART_task.h"
#include <stdio.h>
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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osThreadId_t i2cMonitorTaskHandle;
osThreadId_t spiEepromTaskHandle;
osThreadId_t dtcTaskhandle;
osThreadId_t canTaskHandle;
osThreadId_t uartTaskHandle;

osMutexId_t eepromMutexHandle;
osSemaphoreId_t i2cRxDoneSemaphoreHandle;
osSemaphoreId_t spiTxDoneSemaphoreHandle;
osSemaphoreId_t canTxDoneSemaphoreHandle;
osMessageQueueId_t CanQueueHandle;
osMutexId_t CommMutexHandleHandle;
osMessageQueueId_t dtcProcessingQueueHandle;
osMessageQueueId_t dtcQueueHandle;


volatile SystemState_t g_SystemState = SYS_STATE_INIT;
volatile I2C_State_t g_i2c_state = I2C_STATE_IDLE;

uint8_t g_i2c_rx_buffer[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void Activate_Motor_And_Valve(void) { };
void Deactivate_Motor_And_Valve(void) { };

/* USER CODE BEGIN PFP */
int __io_putchar(int ch);
void MX_FREERTOS_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void BrakeControlTask(void *argument);
void I2cMonitorTask(void *argument);
void SpiEepromTask(void *argument);
void DtcProcessingTask(void *argument);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) { if (hi2c->Instance == I2C1) g_i2c_state = I2C_STATE_IDLE; }
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) { if (hi2c->Instance == I2C1) pmic_i2c_dma_rx_callback_handler(); }
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) { if (hspi->Instance == SPI1) osSemaphoreRelease(spiTxDoneSemaphoreHandle); }
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { osSemaphoreRelease(canTxDoneSemaphoreHandle); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { osSemaphoreRelease(canTxDoneSemaphoreHandle); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { osSemaphoreRelease(canTxDoneSemaphoreHandle); }
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { CAN_RxCallback(hcan); }
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef canfilterconfig = {0};
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  CAN_Init();           // CAN 시작 + 인터럽트 활성화
  EEPROM_ReadDTC();     // 부팅 시 EEPROM에서 DTC 복원
  printf("ECU System Initialized.\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  eepromMutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  i2cRxDoneSemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  spiTxDoneSemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  canTxDoneSemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  CanQueueHandle = osMessageQueueNew(8, sizeof(uint8_t[8]), NULL);
  dtcProcessingQueueHandle = osMessageQueueNew(8, sizeof(DtcEvent_t), NULL);
  dtcQueueHandle = osMessageQueueNew(8, sizeof(uint32_t), NULL);


  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  const osThreadAttr_t i2cAttr  = { .stack_size = 256 * 4, .priority = osPriorityHigh };     /// 우선순위 반영
  const osThreadAttr_t spiAttr  = { .stack_size = 256 * 4, .priority = osPriorityAboveNormal };
  const osThreadAttr_t dtcAttr = { .stack_size = 256 * 4, .priority = osPriorityAboveNormal };
  const osThreadAttr_t canAttr  = { .stack_size = 256 * 4, .priority = osPriorityNormal };
  const osThreadAttr_t uartAttr = { .stack_size = 256 * 4, .priority = osPriorityBelowNormal };
  const osThreadAttr_t brakeCtrlAttr = { .name = "BrakeControl", .stack_size = 256 * 4, .priority = osPriorityHigh };


  i2cMonitorTaskHandle = osThreadNew(I2cMonitorTask , NULL, &i2cAttr);
  spiEepromTaskHandle  = osThreadNew(SpiEepromTask  , NULL, &spiAttr);
  dtcTaskhandle 	   = osThreadNew(DtcProcessingTask, NULL, &dtcAttr);
  canTaskHandle        = osThreadNew(CAN_StartTask, NULL, &canAttr);
  uartTaskHandle       = osThreadNew(UART_StartTask, NULL, &uartAttr);
  osThreadNew(BrakeControlTask, NULL, &brakeCtrlAttr);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.Mode = UART_MODE_TX_RX;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void I2cMonitorTask(void *argument) {
  for(;;) {
    pmic_request_fault_read_dma();   // PMIC 상태를 DMA로 읽기
    osDelay(200);
  }
}

void SpiEepromTask(void *argument) {
  for(;;) {
    osMutexAcquire(eepromMutexHandle, osWaitForever);
    EEPROM_WriteDTC();   // EEPROM에 DTC 백업
    osMutexRelease(eepromMutexHandle);
    osDelay(5000);
  }
}

void DtcProcessingTask(void *argument) {
    DtcEvent_t event;
    for(;;) {
        if (osMessageQueueGet(dtcProcessingQueueHandle, &event, NULL, osWaitForever) == osOK) {
            if (event.command == DTC_EVENT_WRITE) {
                osMutexAcquire(eepromMutexHandle, osWaitForever);
                EEPROM_LogNewDTC(event.dtc_code);                   // Fault 즉시 EEPROM 저장
                osMutexRelease(eepromMutexHandle);
            }
        }
    }
}

// 브레이크 페달 신호를 감지하고 시스템 상태를 관리하는 Task
void BrakeControlTask(void *argument) {
  for(;;) {
    bool isPedalPressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);   // PC13 핀에 브레이크 페달 신호가 연결되어 있다고 가정
    bool isPressureZero = true;                // 압력을 확인하는 별도의 함수 구현 필요
    if (g_SystemState == SYS_STATE_FAULT) {    // fault 상태일 경우 모터, 밸브를 모두 deactivate 시킴
        Deactivate_Motor_And_Valve();
    }
    else if (isPedalPressed) {
      // 페달이 밟히면 즉시 브레이킹 상태로 전환
      if(g_SystemState != SYS_STATE_BRAKING) {
          g_SystemState = SYS_STATE_BRAKING;
      }
    }
    else {
      if (g_SystemState == SYS_STATE_BRAKING && isPressureZero) {            // 페달을 뗐고, 압력이 0이며, 현재 브레이킹 상태일 때만 IDLE로 복귀
          g_SystemState = SYS_STATE_IDLE;
      }
    }
    osDelay(20);
  }
}

// SW 다운로드 신호를 처리하는 GPIO 인터럽트 콜백 함수
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {   				      // P0에 다운로드 시그널 연결 가정
        if (g_SystemState == SYS_STATE_BRAKING) {         // 브레이크 작동 중 상태에서는 다운로드 시그널 무시
            return;
        }
        if (g_SystemState == SYS_STATE_IDLE) {            // 브레이크 대기 상태에서만 다운로드
            g_SystemState = SYS_STATE_DOWNLOAD_READY;
        }
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
