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
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
osThreadId_t i2cMonitorTaskHandle;
osThreadId_t spiEepromTaskHandle;
osThreadId_t uartMonitorTaskHandle;
osMutexId_t eepromMutexHandle;
osSemaphoreId_t i2cRxDoneSemaphoreHandle;
osSemaphoreId_t spiTxDoneSemaphoreHandle;
osSemaphoreId_t canTxDoneSemaphoreHandle;
osMessageQueueId_t dtcProcessingQueueHandle;

volatile I2C_State_t g_i2c_state = I2C_STATE_IDLE;
uint8_t g_i2c_rx_buffer[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
void I2cMonitorTask(void *argument);
void SpiEepromTask(void *argument);
void UartMonitorTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) { if (hi2c->Instance == I2C1) g_i2c_state = I2C_STATE_IDLE; }
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) { if (hi2c->Instance == I2C1) pmic_i2c_dma_rx_callback_handler(); }
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) { if (hspi->Instance == SPI1) osSemaphoreRelease(spiTxDoneSemaphoreHandle); }
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { osSemaphoreRelease(canTxDoneSemaphoreHandle); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { osSemaphoreRelease(canTxDoneSemaphoreHandle); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { osSemaphoreRelease(canTxDoneSemaphoreHandle); }
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    DtcEvent_t event;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        if (rxHeader.StdId == 0x7E0 && rxData[0] == 0x19) { // Read DTC
             event.command = DTC_EVENT_READ_ALL;
             osMessageQueuePut(dtcProcessingQueueHandle, &event, 0U, 0U);
        } else if (rxHeader.StdId == 0x7E0 && rxData[0] == 0x14) { // Clear DTC
             event.command = DTC_EVENT_CLEAR_ALL;
             osMessageQueuePut(dtcProcessingQueueHandle, &event, 0U, 0U);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef canfilterconfig = {0};
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  printf("ECU System Initialized (Advanced Arch).\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();   //  RTOS 커널을 초기화

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

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
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
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
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
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
}

/**
  * @brief SPI1 Initialization Function
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
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
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = EEPROM_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EEPROM_CS_PORT, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void MX_FREERTOS_Init(void) {
  eepromMutexHandle = osMutexNew(NULL);
  i2cRxDoneSemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  spiTxDoneSemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  canTxDoneSemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  dtcProcessingQueueHandle = osMessageQueueNew(8, sizeof(DtcEvent_t), NULL);

  const osThreadAttr_t defaultTaskAttr = { .name = "DefaultTask", .stack_size = 256 * 4 };
  i2cMonitorTaskHandle = osThreadNew(I2cMonitorTask, NULL, &defaultTaskAttr);
  spiEepromTaskHandle = osThreadNew(SpiEepromTask, NULL, &defaultTaskAttr);
  uartMonitorTaskHandle = osThreadNew(UartMonitorTask, NULL, &defaultTaskAttr);
}

void I2cMonitorTask(void *argument) {
  for(;;) {
    pmic_request_fault_read_dma();
    osDelay(200);
  }
}

void SpiEepromTask(void *argument) {
  DtcEvent_t event;
  uint32_t dtc_buffer[DTC_MAX_COUNT];
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8];
  uint32_t txMailbox;

  for(;;) {
    if (osMessageQueueGet(dtcProcessingQueueHandle, &event, NULL, osWaitForever) == osOK) {
        osMutexAcquire(eepromMutexHandle, osWaitForever);    // Mutex획득해서 다른작업의 접근 제한
        switch(event.command) {
            case DTC_EVENT_WRITE:
                printf("[SPI Task] Logging DTC: 0x%lX\r\n", event.dtc_code);
                eeprom_log_new_dtc(event.dtc_code);    // DTC코드를 저장함
                break;
            case DTC_EVENT_READ_ALL:
                printf("[SPI Task] Reading DTCs for CAN.\r\n");
                uint8_t count = eeprom_read_all_dtcs(dtc_buffer);
                txHeader.StdId = 0x7E8; txHeader.IDE = CAN_ID_STD; txHeader.RTR = CAN_RTR_DATA;
                if (count == 0) {
                    txHeader.DLC = 3; txData[0] = 0x59; txData[1] = 0x02; txData[2] = 0xFF; // No DTC
                    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
                } else {
                    for(int i = 0; i < count; i++) {
                        txHeader.DLC = 6;
                        txData[0] = 0x59; txData[1] = 0x02;
                        txData[2] = (dtc_buffer[i] >> 16) & 0xFF;
                        txData[3] = (dtc_buffer[i] >> 8) & 0xFF;
                        txData[4] = dtc_buffer[i] & 0xFF;
                        txData[5] = 0x01;    // status: confirmed
                        HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
                        osSemaphoreAcquire(canTxDoneSemaphoreHandle, 100);
                    }
                }
                break;
            case DTC_EVENT_CLEAR_ALL:
                printf("[SPI Task] Clearing all DTCs.\r\n");
                eeprom_clear_all_dtcs();
                break;
        }
        osMutexRelease(eepromMutexHandle);
    }
  }
}

void UartMonitorTask(void *argument) {
    char msg[] = "ECU Alive...\r\n";
    for(;;) {
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 200);
        osDelay(1000);
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
  __disable_irq();    // 모든 인터럽트를 비활성화하고 시스템을 멈춤
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
