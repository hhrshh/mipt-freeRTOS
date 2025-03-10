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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_DEBUG
#define BUF_SIZE 256
#define MB_ADDR 2
#define COIL_NUMBERS 1
#define SHOW_MINUS 10
#define LOCK_PRINT_SEGMENT 11
#define configASSERT(x) if((x) == 0) {taskDISABLE_INTERRUPTS(); for(;;){ \
									  GPIOA->BSRR = GPIO_PIN_4; \
									  __NOP(); \
									  GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;}}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;

osThreadId indicationSegmHandle;
osThreadId modbusReceiveHandle;
osMessageQId queueDisplaySHandle;
osTimerId swoTimerDebugHandle;
osStaticTimerDef_t swoTimerDebugControlBlock;
/* USER CODE BEGIN PV */

segmen_t sSegment;
int8_t i_buffer[BUF_SIZE] = {0};
int16_t temp = 1; // температура
uint8_t isLoaded = 1; // флаг для ожидания загрузки значений из UART
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
void indicationFunc(void const * argument);
void modbusReceiveFunc(void const * argument);
void swoTimerDebugCallback(void const * argument);

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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of swoTimerDebug */
  osTimerStaticDef(swoTimerDebug, swoTimerDebugCallback, &swoTimerDebugControlBlock);
  swoTimerDebugHandle = osTimerCreate(osTimer(swoTimerDebug), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  	if(swoTimerDebugHandle != NULL)
  	{
  		xTimerStart(swoTimerDebugHandle, 0);  // Второй параметр — время ожидания (0 — не блокирующий вызов)
  		xTimerChangePeriod(swoTimerDebugHandle, 1000, 0);
    }
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of queueDisplayS */
  osMessageQDef(queueDisplayS, 16, int16_t);
  queueDisplaySHandle = osMessageCreate(osMessageQ(queueDisplayS), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of indicationSegm */
  osThreadDef(indicationSegm, indicationFunc, osPriorityNormal, 0, 128);
  indicationSegmHandle = osThreadCreate(osThread(indicationSegm), NULL);

  /* definition and creation of modbusReceive */
  osThreadDef(modbusReceive, modbusReceiveFunc, osPriorityNormal, 0, 128);
  modbusReceiveHandle = osThreadCreate(osThread(modbusReceive), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
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
	  case 1:
		  GPIOA->BSRR = GPIO_PIN_1;
		  break;
	  case 2:
		  GPIOA->BSRR = GPIO_PIN_2;
		  break;
	  case 3:
		  GPIOA->BSRR = GPIO_PIN_3;
		  break;
    }
    #endif
}

void TaskSwichedOut(int tag)
{
  #ifdef TASK_DEBUG
    switch (tag)
    {
	  case 1:
		  GPIOA->BSRR = (uint32_t)GPIO_PIN_1 << 16U;
		  break;
	  case 2:
		  GPIOA->BSRR = (uint32_t)GPIO_PIN_2 << 16U;
		  break;
	  case 3:
		  GPIOA->BSRR = (uint32_t)GPIO_PIN_3 << 16U;
		  break;
    }
    #endif
}

void vApplicationIdleHook(void)
{
  #ifdef TASK_DEBUG
    GPIOA->BSRR = GPIO_PIN_4;
    __NOP();
    GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
  #endif
}

uint16_t crc_mb(uint8_t *buf, uint8_t len)
{
	uint16_t crc = 0xFFFF;

	for (int pos = 0; pos < len; pos++)
	{
		crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc
		for (int i = 8; i != 0; i--)
		{ // Loop over each bit
			if ((crc & 0x0001) != 0)
			{ // If the LSB is set
				crc >>= 1; // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else // Else LSB is not set
				crc >>= 1; // Just shift right
		}
	}
	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
	return crc;
}







uint8_t parse(int8_t *in_buffer, uint16_t buffer_size, int16_t *temp)
{
	//Опрелеляем функции, которые обслуживает наш серверы
	typedef enum {MB_F_NONE = 0,
				  MB_F_WRITE_SINGLE_REGISTER = 0x6,
				  MB_F_WRITE_MULTIPLAY_REGISTER = 0x10}fcode_t;


	static int8_t out_buffer[256]; 			// Массив для нашего ответа
	int out_len = 0; 							// Длина ответа
	union
	{
		uint16_t uint16;
		uint8_t  bytes[2];
	}crc; 										// Сюда удобно класть контрольную сумму

	fcode_t f_code 		   = MB_F_NONE;			// Код запроса
	//uint16_t registr       = 0;					// Адрес выхода


	if(in_buffer[0] == MB_ADDR)					// Если нам пришёл запрос…
	{
		f_code = in_buffer[1];					// Запоминаем его код
		switch(f_code)
		{
		case MB_F_WRITE_SINGLE_REGISTER:					// Запись в регистр
			for(int i = 0; i < 6; i++)
				out_buffer[i] = in_buffer[i];

			//registr = (((uint16_t)in_buffer[2]) << 8) | ((uint16_t)in_buffer[3]); // Читаем адрес выхода
			*temp = (((uint16_t)in_buffer[4]) << 8) | ((uint16_t)in_buffer[5]); // Записываем значение температуры

			crc.uint16 = crc_mb(out_buffer, 6);	// Считаем КС ответа
			out_buffer[6] = crc.bytes[0];		// Записываем её в ответ
			out_buffer[7] = crc.bytes[1];
			out_len = 8;						// количество байт фиксированное
			HAL_UART_Transmit(&huart1, out_buffer, out_len, 200); // Передаём ответ
			return 1;
			break;
		default:									// Другая функция
			out_len = 5;
			out_buffer [0] = in_buffer[0]; 			// Адрес устройства
			out_buffer [1] = in_buffer[1] | 0x80; 	// Признак ошибки: номер функции с единицей в старшем разряде
			out_buffer [2] = 1;						// Стандартная ошибка № 1: функция не реализована
			crc.uint16 = crc_mb(out_buffer, 3);
			out_buffer[3] = crc.bytes[0];
			out_buffer[4] = crc.bytes[1];

 			HAL_UART_Transmit(&huart1, out_buffer, out_len, 200);
 			return 0;
			break;
		}
	}
	return 0;
}

// Функция для получения данных по UART
void modbusRvTmData(void)
{
    // Чтение данных из UART
    HAL_UART_Receive(&huart1, i_buffer, BUF_SIZE, 10);
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
void indicationFunc(void const * argument)
{
  /* USER CODE BEGIN 5 */
  #ifdef TASK_DEBUG
    vTaskSetApplicationTaskTag(NULL, (void*)1);
  #endif


  while(isLoaded)
	  osDelay(1);
  uint8_t i = 0;
  int16_t resTemp = 0; // приходячая из очереди температура
  uint8_t a = LOCK_PRINT_SEGMENT, b = 0, c = 0; // 1 - для знака, 2 и 3 числа
  /* Infinite loop */
  for(;;)
  {
          // Обработка полученных данных
          // Например, вывод данных в консоль или выполнение других действий
	  	if(xQueueReceive(queueDisplaySHandle, &resTemp, 1) == pdTRUE)
	  	{
	  		// не корректное значение температуры, уходим в for(;;);
	  		configASSERT(!(resTemp > 99 || resTemp < -99));
	  		if(resTemp < 0)
	  		{
	  			a = SHOW_MINUS;	// если отрицательно печатаем 10 (знак -)
	  			resTemp = -resTemp; // меняем знак
	  		}
	  		else
	  			a = LOCK_PRINT_SEGMENT; // Не печатем первый сегмент ( a > 10 return)
	  	    c = resTemp % 10; // Третьий сегмент
	  	    resTemp /= 10;
	  	    b = resTemp % 10; // Второй сегмент
	  	    resTemp /= 10;
	  	}
		switch(i)
		{
			case 0:
				// Отображаем 1 сегмент
				displaySegment(&sSegment, i, a, 0);
				break;
			case 1:
				// Отображаем 2 сегмент
				displaySegment(&sSegment, i, b, 0);
				break;
			case 2:
				// Отображаем 3 сегмент
				displaySegment(&sSegment, i, c, 0);
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
void modbusReceiveFunc(void const * argument)
{
  /* USER CODE BEGIN modbusReceiveFunc */
  /* Infinite loop */
  #ifdef TASK_DEBUG
    vTaskSetApplicationTaskTag(NULL, (void*)2);
  #endif

    // Запуск таймера

  for(;;)
  {
	  modbusRvTmData();
	  osDelay(1);
  }
  /* USER CODE END modbusReceiveFunc */
}

/* swoTimerDebugCallback function */
void swoTimerDebugCallback(void const * argument)
{
  /* USER CODE BEGIN swoTimerDebugCallback */
    GPIOA->BSRR = GPIO_PIN_3;
    __NOP();
    GPIOA->BSRR = (uint32_t)GPIO_PIN_3 << 16U;
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
