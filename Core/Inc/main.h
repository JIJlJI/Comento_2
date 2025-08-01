/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// 메시지 큐를 통해 전달될 이벤트의 종류
typedef enum {
    DTC_EVENT_WRITE,      // 새로운 DTC를 쓰라는 명령
    DTC_EVENT_READ_ALL,   // 저장된 모든 DTC를 읽으라는 명령 (CAN 요청)
    DTC_EVENT_CLEAR_ALL   // 모든 DTC를 지우라는 명령 (CAN 요청)
} DtcCommand_t;

// 메시지 큐를 통해 전달될 이벤트 데이터 구조체
typedef struct {
    DtcCommand_t command;
    uint32_t     dtc_code; // DTC_EVENT_WRITE 일 때만 유효한 데이터
} DtcEvent_t;

// 비동기 처리를 위한 상태 머신(State Machine) enum
typedef enum {
    I2C_STATE_IDLE,
    I2C_STATE_READ_FAULTS_WAIT
} I2C_State_t;

// 시스템의 전체 동작 상태 관리
typedef enum {
    SYS_STATE_INIT,         // 초기화 상태
    SYS_STATE_IDLE,         // 대기 상태 (브레이크 OFF)
    SYS_STATE_BRAKING,      // 브레이크 작동 중인 상태
    SYS_STATE_FAULT,        // 오류 발생 상태
    SYS_STATE_DOWNLOAD_READY// 다운로드 준비/실행 상태
} SystemState_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define EEPROM_CS_PORT  GPIOA
#define EEPROM_CS_PIN   GPIO_PIN_4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
