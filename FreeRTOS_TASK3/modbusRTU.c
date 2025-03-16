/******************************************************************************
 * sevenSegment.c
 *
 *  Created on: Mar 6, 2025
 *  Author: Mikhailichenko D.
 *  Description: Библиотека для modbusRTU
 ******************************************************************************/

#include "modbusRTU.h"
#include "main.h"

extern UART_HandleTypeDef huart1;



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
	uint16_t out_len = 0; 							// Длина ответа
	union
	{
		int16_t int16;
		uint8_t  bytes[2];
	}crc; 										     // Сюда удобно класть контрольную сумму

	fcode_t f_code 		   = MB_F_NONE;			    // Код запроса
	//uint16_t registr       = 0;					// Адрес выхода


	if(in_buffer[0] == (int8_t)MB_ADDR)					// Если нам пришёл запрос…
	{
		f_code = in_buffer[1];					// Запоминаем его код
		switch(f_code)
		{
		case MB_F_WRITE_SINGLE_REGISTER:					// Запись в регистр
			for(int i = 0; i < 6; i++)
				out_buffer[i] = in_buffer[i];

			//registr = (((uint16_t)in_buffer[2]) << 8) | ((uint16_t)in_buffer[3]); // Читаем адрес выхода
			*temp = (((int16_t)in_buffer[4]) << 8) | ((int16_t)in_buffer[5]); // Записываем значение температуры

			crc.int16 = crc_mb(out_buffer, 6);	// Считаем КС ответа
			out_buffer[6] = crc.bytes[0];		// Записываем её в ответ
			out_buffer[7] = crc.bytes[1];
			out_len = 8;						// количество байт фиксированное

			HAL_UART_Transmit(&huart1, out_buffer, out_len, 10); // Передаём ответ
			return 1;
			break;
		default:									// Другая функция
			out_len = 5;
			out_buffer [0] = in_buffer[0]; 			// Адрес устройства
			out_buffer [1] = in_buffer[1] | 0x80; 	// Признак ошибки: номер функции с единицей в старшем разряде
			out_buffer [2] = 1;						// Стандартная ошибка № 1: функция не реализована
			crc.int16 = crc_mb(out_buffer, 3);
			out_buffer[3] = crc.bytes[0];
			out_buffer[4] = crc.bytes[1];

 			HAL_UART_Transmit(&huart1, out_buffer, out_len, 50);
 			return 0;
			break;
		}
	}
	return 0;
}
