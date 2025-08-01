/*
 * CAN_task.h
 *
 *  Created on: Jul 31, 2025
 *      Author: pythonjihyun
 */

#ifndef INC_CAN_TASK_H_
#define INC_CAN_TASK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

void CAN_Init(void);
void CAN_RxCallback(CAN_HandleTypeDef *hcan);
void CAN_StartTask(void *argument);

#endif /* INC_CAN_TASK_H_ */
