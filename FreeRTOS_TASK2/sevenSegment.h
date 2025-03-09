/******************************************************************************
  * sevenSegment.h
  *
  * Created on: Mar 6, 2025
  * Author: Mikhailichenko D.
  * Description: Упрвления 7 сегментным индикатором.
******************************************************************************/
#ifndef SEVENSEGMENT_H
#define SEVENSEGMENT_H
#include "main.h"
#define MAXSEGMENT 3 // максимальное количество сегметов

typedef struct {
	uint8_t segmentState[11][7]; // состояние пинов для цифр от 0-9
    int16_t pin[7];   		   // пины для управления сегментами
    int16_t point;               // пин с точкой
    int16_t pwrOnNumSegrment[3]; // активация питания на нужный сегмент
    GPIO_TypeDef* gpioPort; // Порт GPIO, к которому подключены сегменты
}segmen_t;

// инициализация сигмента
void initSegment(segmen_t* segment);
// отображение сигмента
void displaySegment(segmen_t* segment, uint8_t numSegmen, uint8_t digit, uint8_t point);

#endif  /* __MAIN_H */
