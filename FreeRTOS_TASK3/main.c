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
#include "sevenSegment.h"
#include "modbusRTU.h"
#include "timers.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_DEBUG
#define BUF_SIZE 256
#define COIL_NUMBERS 1
#define SHOW_MINUS 10
#define LOCK_PRINT_SEGMENT 11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;

/* Definitions for indicationSegm */
osThreadId_t indicationSegmHandle;
const osThreadAttr_t indicationSegm_attributes = {
  .name = "indicationSegm",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for modbusReceive */
osThreadId_t modbusReceiveHandle;
const osThreadAttr_t modbusReceive_attributes = {
  .name = "modbusReceive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for queueDisplayS */
osMessageQueueId_t queueDisplaySHandle;
const osMessageQueueAttr_t queueDisplayS_attributes = {
  .name = "queueDisplayS"
};
/* Definitions for swoTimerDebug */
osTimerId_t swoTimerDebugHandle;
osStaticTimerDef_t swoTimerDebugControlBlock;
const osTimerAttr_t swoTimerDebug_attributes = {
  .name = "swoTimerDebug",
  .cb_mem = &swoTimerDebugControlBlock,
  .cb_size = sizeof(swoTimerDebugControlBlock),
};
/* Definitions for buttonEvent */
osEventFlagsId_t buttonEventHandle;
const osEventFlagsAttr_t buttonEvent_attributes = {
  .name = "buttonEvent"
};
/* USER CODE BEGIN PV */

segmen_t sSegment;
int8_t i_buffer[BUF_SIZE] = {0};
int16_t temp = 1; // температура
static uint8_t isLoaded = 1; // флаг для ожидания загрузки значений из UART

typedef struct{
	uint8_t a;
	uint8_t b;
	uint8_t c;
}temp_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
void indicationFunc(void *argument);
void modbusReceiveFunc(void *argument);
void swoTimerDebugCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	for(int i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
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
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  initSegment(&sSegment);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of swoTimerDebug */
  swoTimerDebugHandle = osTimerNew(swoTimerDebugCallback, osTimerPeriodic, NULL, &swoTimerDebug_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  	if(swoTimerDebugHandle != NULL)
  	{
  		xTimerStart(swoTimerDebugHandle, 0);  // Второй параметр — время ожидания (0 — не блокирующий вызов)
  		xTimerChangePeriod(swoTimerDebugHandle, 1000, 0);
    }
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of queueDisplayS */
  queueDisplaySHandle = osMessageQueueNew (16, sizeof(int16_t), &queueDisplayS_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of indicationSegm */
  indicationSegmHandle = osThreadNew(indicationFunc, NULL, &indicationSegm_attributes);

  /* creation of modbusReceive */
  modbusReceiveHandle = osThreadNew(modbusReceiveFunc, NULL, &modbusReceive_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of buttonEvent */
  buttonEventHandle = osEventFlagsNew(&buttonEvent_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Task1_Pin|Task2_Pin|Task3_Pin|TaskIdle_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LA_Pin|LB_Pin|LC_Pin|L3_Pin
                          |LD_Pin|LE_Pin|LF_Pin|LG_Pin
                          |LP_Pin|L1_Pin|L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Task1_Pin Task2_Pin Task3_Pin TaskIdle_Pin */
  GPIO_InitStruct.Pin = Task1_Pin|Task2_Pin|Task3_Pin|TaskIdle_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT1_Pin BUT2_Pin BUT3_Pin BUT4_Pin */
  GPIO_InitStruct.Pin = BUT1_Pin|BUT2_Pin|BUT3_Pin|BUT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LA_Pin LB_Pin LC_Pin L3_Pin
                           LD_Pin LE_Pin LF_Pin LG_Pin
                           LP_Pin L1_Pin L2_Pin */
  GPIO_InitStruct.Pin = LA_Pin|LB_Pin|LC_Pin|L3_Pin
                          |LD_Pin|LE_Pin|LF_Pin|LG_Pin
                          |LP_Pin|L1_Pin|L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TaskSwichedIn(int tag)
{
  #ifdef TASK_DEBUG
    switch (tag)
    {
	  case 0:
		  GPIOA->BSRR = GPIO_PIN_0;
		  break;
	  case 1:
		  GPIOA->BSRR = GPIO_PIN_1;
		  break;
	  case 2:
		  GPIOA->BSRR = GPIO_PIN_2;
		  break;
    }
    #endif
}

void TaskSwichedOut(int tag)
{
  #ifdef TASK_DEBUG
    switch (tag)
    {
	  case 0:
		  GPIOA->BSRR = (uint32_t)GPIO_PIN_0 << 16U;
		  break;
	  case 1:
		  GPIOA->BSRR = (uint32_t)GPIO_PIN_1 << 16U;
		  break;
	  case 2:
		  GPIOA->BSRR = (uint32_t)GPIO_PIN_2 << 16U;
		  break;
    }
    #endif
}

void vApplicationIdleHook(void)
{
  #ifdef TASK_DEBUG
    GPIOA->BSRR = GPIO_PIN_3;
    __NOP();
    GPIOA->BSRR = (uint32_t)GPIO_PIN_3 << 16U;
  #endif
}

void parseTemp(int16_t resTemp, temp_t* segment)
{


	if(resTemp < 0)
	{
		segment->a = SHOW_MINUS;	// если отрицательно печатаем 10 (знак -)
		resTemp = -resTemp; // меняем знак
	}
	else
		segment->a = LOCK_PRINT_SEGMENT; // Не печатем первый сегмент ( a > 10 return)
	segment->c = resTemp % 10; // Третьий сегмент
    resTemp /= 10;
	segment->b = resTemp % 10; // Второй сегмент
}

// Функция для получения данных по UART
void modbusRvTmData(void)
{
	extern uint8_t isLoaded;
    // Чтение данных из UART
    HAL_UART_Receive(&huart1, i_buffer, BUF_SIZE, 50);
    if(parse(i_buffer, BUF_SIZE, &temp))
    {
    	xQueueSendFromISR(queueDisplaySHandle, &temp, NULL);
        isLoaded = 0;
    }
    //taskEXIT_CRITICAL();
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_indicationFunc */
/**
  * @brief  Function implementing the indicationSegm thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_indicationFunc */
void indicationFunc(void *argument)
{
  /* USER CODE BEGIN 5 */
  #ifdef TASK_DEBUG
	  vTaskSetApplicationTaskTag(NULL, (void*)0);
  #endif


  while(isLoaded)
	  osDelay(1);
  uint8_t i = 0;
  int16_t resTemp = 0; // приходящая из очереди температура
  temp_t segments = {.a = LOCK_PRINT_SEGMENT, .b = 0, .c = 0}; // 1 - для знака, 2 и 3 числа
  /* Infinite loop */
  for(;;)
  {
	  	if(xQueueReceive(queueDisplaySHandle, &resTemp, 1) == pdTRUE)
	  	{
	  		// если получаем не корректные значения температуры уходим в for(;;);
		  	if(resTemp > 99 || resTemp < -99)
		  		Error_Handler();
	  		parseTemp(resTemp, &segments);
	  	}
		switch(i)
		{
			case 0:
				// Отображаем 1 сегмент
				displaySegment(&sSegment, i, segments.a, 0);
				break;
			case 1:
				// Отображаем 2 сегмент
				displaySegment(&sSegment, i, segments.b, 0);
				break;
			case 2:
				// Отображаем 3 сегмент
				displaySegment(&sSegment, i, segments.c, 0);
				break;
		}
		if(++i > 2)
			i = 0;
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_modbusReceiveFunc */
/**
* @brief Function implementing the modbusReceive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_modbusReceiveFunc */
void modbusReceiveFunc(void *argument)
{
  /* USER CODE BEGIN modbusReceiveFunc */
  /* Infinite loop */
  #ifdef TASK_DEBUG
    vTaskSetApplicationTaskTag(NULL, (void*)1);
  #endif

  for(;;)
  {
	  modbusRvTmData();
	  osDelay(1);
  }
  /* USER CODE END modbusReceiveFunc */
}

/* swoTimerDebugCallback function */
void swoTimerDebugCallback(void *argument)
{
  /* USER CODE BEGIN swoTimerDebugCallback */
    GPIOA->BSRR = GPIO_PIN_2;
    __NOP();
    GPIOA->BSRR = (uint32_t)GPIO_PIN_2 << 16U;
    HAL_IWDG_Refresh(&hiwdg);
    //printf("swo test\n");
  /* USER CODE END swoTimerDebugCallback */
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
  uint8_t i = 0;
  while (1)
  {
	  displaySegment(&sSegment, (uint8_t)2, i++ > 8 ? i = 0 : i, 0);
	  for(uint32_t i = 0xFFF; i < 0xFFFFF; i++)__NOP();
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
