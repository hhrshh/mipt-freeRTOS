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
#include "event_groups.h"
#include "sevenSegment.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_DEBUG
#define BIT_0	( 1 << 0 )  // Определение бита 0
#define BIT_1	( 1 << 1 )  // Определение бита 1
#define BIT_2	( 1 << 2 )  // Определение бита 2
#define BIT_3	( 1 << 3 )  // Определение бита 3
#define BIT_4	( 1 << 4 )  // Определение бита 4
#define BIT_5	( 1 << 5 )  // Определение бита 5
#define BIT_6	( 1 << 6 )  // Определение бита 6
#define BIT_7	( 1 << 7 )  // Определение бита 7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

typedef enum {
    LOW = 0,
    HIGH = 1
} PinState;

typedef union {
    uint8_t gpio_PinState; // Поле для хранения состояния всех пинов
    struct {
        PinState io_7 : 1; // Бит 7
        PinState io_6 : 1; // Бит 6
        PinState io_5 : 1; // Бит 5
        PinState io_4 : 1; // Бит 4
        PinState io_3 : 1; // Бит 3
        PinState io_2 : 1; // Бит 2
        PinState io_1 : 1; // Бит 1
        PinState io_0 : 1; // Бит 0
    };
} mask_t;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for inTask */
osThreadId_t inTaskHandle;
const osThreadAttr_t inTask_attributes = {
  .name = "inTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for outTask */
osThreadId_t outTaskHandle;
const osThreadAttr_t outTask_attributes = {
  .name = "outTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
segmen_t sSegment;
EventGroupHandle_t Event_Handle;
mask_t mask = {.gpio_PinState = 0b00001111};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void inTaskFunc(void *argument);
void outTaskFunc(void *argument);

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

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of inTask */
  inTaskHandle = osThreadNew(inTaskFunc, NULL, &inTask_attributes);

  /* creation of outTask */
  outTaskHandle = osThreadNew(outTaskFunc, NULL, &outTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

  /* add events, ... */
  Event_Handle = xEventGroupCreate();
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Task1_Pin|Task2_Pin|Task3_Pin|Task4_Pin
                          |LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LA_Pin|LB_Pin|LC_Pin|L3_Pin
                          |LD_Pin|LE_Pin|LF_Pin|LG_Pin
                          |LP_Pin|L1_Pin|L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : Task1_Pin Task2_Pin Task3_Pin Task4_Pin
                           LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = Task1_Pin|Task2_Pin|Task3_Pin|Task4_Pin
                          |LED1_Pin|LED2_Pin;
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
	  case 3:
		  GPIOA->BSRR = GPIO_PIN_3;
		  break;
	  case 4:
		  GPIOB->BSRR = GPIO_PIN_3;
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
	  case 3:
		  GPIOA->BSRR = (uint32_t)GPIO_PIN_3 << 16U;
		  break;
	  case 4:
		  GPIOB->BSRR = (uint32_t)GPIO_PIN_3 << 16U;
		  break;
    }
    #endif
}


void setOutputs(EventGroupHandle_t xEventGroup, mask_t* mask)
{
    EventBits_t uxBits;  // Переменная для хранения битов события
    const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;  // Время ожидания в тиках (максимум 100 мс)

    /* Ждем максимум 100 мс, чтобы один из битов 0, 1, 2, 3 или 4 был установлен в группе событий.
       Очищаем биты перед выходом. */
    uxBits = xEventGroupWaitBits(
               xEventGroup,   /* Группа событий, которую мы проверяем. */
               mask->gpio_PinState, /* Биты в группе событий, на которые мы ждем. */
			   pdTRUE,        /* Все биты должны быть очищены перед возвратом. */
               pdFALSE,         /* Не ждем, чтобы все биты были установлены, подойдет любой бит. */
               xTicksToWait);  /* Ждем максимум 100 мс, чтобы один из битов был установлен. */
    if((uxBits & (BIT_0 | BIT_3 )) == (BIT_0 | BIT_3))
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // Выход 1: срабатывает при событиях 1 и 4
    else
    	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // Сброс

    if((uxBits & BIT_0) == BIT_0)
    	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); // Выход 2: срабатывает при событии 1
    else
    	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); // Сброс

    if((uxBits & (BIT_2 | BIT_3)) == (BIT_2 | BIT_3))
    	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); // Выход 3: срабатывает при событиях 3 и 4
    else
    	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); // Сброс

    if((uxBits & (BIT_0 | BIT_1 | BIT_2 | BIT_3)) != 0)
    	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET); // Выход 4: срабатывает при любом событии
    else
    	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET); // Сброс
}

uint32_t readInputs(uint8_t pins)
{
    uint32_t bitmask = 0;
    uint16_t  pin;

    // Считываем состояние каждого пина и формируем битовую маску
    for (uint8_t i = 0; i < pins; i++)
    {
        GPIO_TypeDef* port = GPIOA;
        pin = 1 << (i + 4);

        // Считываем состояние пина
        if(HAL_GPIO_ReadPin(port, pin))
            bitmask |= (1 << i);
    }
    return bitmask;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_inTaskFunc */
/**
  * @brief  Function implementing the inTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_inTaskFunc */
void inTaskFunc(void *argument)
{
  /* USER CODE BEGIN 5 */
  #ifdef TASK_DEBUG
	vTaskSetApplicationTaskTag(NULL, (void*)0);
  #endif
	/* Infinite loop */
  for(;;)
  {
	setOutputs(Event_Handle, &mask);
	osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_outTaskFunc */
/**
* @brief Function implementing the outTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_outTaskFunc */
void outTaskFunc(void *argument)
{
  /* USER CODE BEGIN outTaskFunc */
  /* Infinite loop */
  #ifdef TASK_DEBUG
    vTaskSetApplicationTaskTag(NULL, (void*)4);
  #endif
  for(;;)
  {
	printf("%u\n", xEventGroupSetBits(Event_Handle, readInputs(4)));

	osDelay(5);
  }
  /* USER CODE END outTaskFunc */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  uint8_t i = 0;
  __disable_irq();
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
