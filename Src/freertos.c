/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>
#include <semphr.h>
#include <stdio.h>

#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "imu.h"
#include "display.h"
#include "fusion.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/


/* USER CODE BEGIN Variables */
TaskHandle_t imuTaskHandle;
TaskHandle_t fusionTaskHandle;
TaskHandle_t displayTaskHandle;
SemaphoreHandle_t mutexI2C;
SemaphoreHandle_t irqAccGyro;
SemaphoreHandle_t irqCompass;
SemaphoreHandle_t drdyCompass;

static unsigned const QUEUE_DEPTH = 16;
QueueHandle_t	qImuToFusion;
QueueHandle_t	qFusionToDisplay;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	mutexI2C =  xSemaphoreCreateMutex();

	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	qImuToFusion = xQueueCreate(QUEUE_DEPTH, sizeof(IMUData_t));
	qFusionToDisplay = xQueueCreate(QUEUE_DEPTH, sizeof(FusedData_t));
	if ((NULL==qImuToFusion) || (NULL==qFusionToDisplay)) {
		printf("Failed to create queue\r\n");
		/* TODO: cleanup */
	}

	irqAccGyro = xSemaphoreCreateCounting(QUEUE_DEPTH, 0);
	irqCompass = xSemaphoreCreateCounting(QUEUE_DEPTH, 0);
	drdyCompass = xSemaphoreCreateCounting(QUEUE_DEPTH, 0);

	if (! (irqAccGyro && irqCompass && drdyCompass)) {
		printf("Failed to create a semaphore\r\n");
		/* TODO: Cleanup */
	}

	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */

	/* USER CODE BEGIN RTOS_THREADS */
	BaseType_t ret;
	ret = xTaskCreate(&IMUTaskThread, "imuTask", 128, NULL, tskIDLE_PRIORITY + 3, &imuTaskHandle);
	ret &= xTaskCreate(&FusionTaskThread, "fusionTask", 128, NULL, tskIDLE_PRIORITY, &fusionTaskHandle);
	ret &= xTaskCreate(&DisplayTaskThread, "displayTask", 128, NULL, tskIDLE_PRIORITY + 2, &displayTaskHandle);

	if (pdPASS == ret) {
		/* TODO: cleanup */
		printf("Failed to create all threads\r\n");
	}

	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
	vTaskStartScheduler();
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
