/******************************************************************************
 * sevenSegment.c
 *
 *  Created on: Mar 6, 2025
 *  Author: Mikhailichenko D.
 *  Description: Библиотека для упрвления 7 сегментным индикатором.
 ******************************************************************************/
#include "sevenSegment.h"
#include "main.h"

void initSegment(segmen_t* segment)
{
    // Инициализация состояния сегментов для цифр от 0 до 9
	const uint8_t state[11][7] = {
	    {1, 1, 1, 1, 1, 1, 0}, // 0
	    {0, 1, 1, 0, 0, 0, 0}, // 1
	    {1, 1, 0, 1, 1, 0, 1}, // 2
	    {1, 1, 1, 1, 0, 0, 1}, // 3
	    {0, 1, 1, 0, 0, 1, 1}, // 4
	    {1, 0, 1, 1, 0, 1, 1}, // 5
	    {1, 0, 1, 1, 1, 1, 1}, // 6
	    {1, 1, 1, 0, 0, 0, 0}, // 7
	    {1, 1, 1, 1, 1, 1, 1}, // 8
	    {1, 1, 1, 1, 0, 1, 1}, // 9
	    {0, 0, 0, 0, 0, 0, 1}  // -
	};

    // Копируем состояние сегментов
    for(int i = 0; i < 11; i++)
        for(int j = 0; j < 7; j++)
            segment->segmentState[i][j] = state[i][j];

    // Инициализация пинов для управления сегментами
    segment->pin[0] = LA_Pin;
    segment->pin[1] = LB_Pin;
    segment->pin[2] = LC_Pin;
    segment->pin[3] = LD_Pin;
    segment->pin[4] = LE_Pin;
    segment->pin[5] = LF_Pin;
    segment->pin[6] = LG_Pin;

    // Инициализация пина с точкой
    segment->point = LP_Pin;

    // Инициализация активации питания на нужный сегмент
    segment->pwrOnNumSegrment[0] = L1_Pin;
    segment->pwrOnNumSegrment[1] = L2_Pin;
    segment->pwrOnNumSegrment[2] = L3_Pin;

    // Инициализация порта GPIO
    segment->gpioPort = GPIOB;
}

// Отображения цифры на 7сегментном индикаторе c колчеством цифр = numSegmen
void displaySegment(segmen_t* segment, uint8_t numSegmen, uint8_t digit, uint8_t point)
{
    // Проверка, что digit находится в допустимом диапазоне (0-9)
    if(digit > 10)
        return; // Выход, если цифра недопустима
    if(numSegmen > MAXSEGMENT)
        return; // больше чем количество сегментов

    // Проходим по всем сегментам (A, B, C, D, E, F, G)
    for (uint8_t i = 0; i < 7; i++)
    {
    // Устанавливаем состояние пина в зависимости от segmentState
    HAL_GPIO_WritePin(
    		segment->gpioPort,          // Порт GPIO
            segment->pin[i],            // Пин сегмента
            segment->segmentState[digit][i] ? GPIO_PIN_SET : GPIO_PIN_RESET
        );
    }

    // Устанавливаем сегмента с точкой
    HAL_GPIO_WritePin(
    		segment->gpioPort,          			  // Порт GPIO
			segment->point,     					  // Пин точки
			point);
    // Отключаем питания сегментов
    HAL_GPIO_WritePin(
    		segment->gpioPort,          			  // Порт GPIO
            segment->pwrOnNumSegrment[0] | segment->pwrOnNumSegrment[1]
			| segment->pwrOnNumSegrment[2],           // Пин сегмента
			GPIO_PIN_RESET);
    // Включаем питание нужного по счету сегмента
    HAL_GPIO_WritePin(
    		segment->gpioPort,          			  // Порт GPIO
            segment->pwrOnNumSegrment[numSegmen],     // Пин сегмента
			GPIO_PIN_SET);
}
