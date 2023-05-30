/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USER_BUTTON_PIN                       GPIO_PIN_13
//#define USER_BUTTON_GPIO_PORT                 GPIOC
#define YELLOW_PIN                              GPIO_PIN_5
#define YELLOW_GPIO_PORT                        GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t LD2_Period = 100;
uint32_t YELLOW_Period = 300;

int i, n = 5;
time_t t;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LedTask1Handle;
osThreadId myTask03Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint16_t sqrt_new(uint32_t L) {
  if (L < 2)
    return (uint16_t) L;

  uint32_t rslt, div;

  rslt = L;
  div = L / 2;

  while (1) {
    div = (L / div + div) / 2;

    if (rslt > div)
      rslt = div;
    else
      return (uint16_t) rslt;
  }
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(const void *argument);
void LedTaskFunc(const void *argument);
void YellowLED(const void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LedTask1 */
  osThreadDef(LedTask1, LedTaskFunc, osPriorityAboveNormal, 0, 128);
  LedTask1Handle = osThreadCreate(osThread(LedTask1), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, YellowLED, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(const void *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    if (LD2_Period == 100) {
      LD2_Period = 1000;
    } else
      LD2_Period = 100;
    osDelay(5000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LedTaskFunc */
/**
 * @brief Function implementing the LedTask1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LedTaskFunc */
void LedTaskFunc(const void *argument) {
  /* USER CODE BEGIN LedTaskFunc */
  /* Infinite loop */
  for (;;) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    osDelay(LD2_Period);
  }
  /* USER CODE END LedTaskFunc */
}

/* USER CODE BEGIN Header_YellowLED */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_YellowLED */
void YellowLED(const void *argument) {
  /* USER CODE BEGIN YellowLED */
  srand((unsigned) time(&t));
  /* Infinite loop */
  for (;;) {
    HAL_GPIO_TogglePin(YELLOW_GPIO_PORT, YELLOW_PIN);
    //osDelay(3 * sqrt_new(LD2_Period));
    osDelay(rand() % 500);

  }
  /* USER CODE END YellowLED */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

