/******************************************************************************
  * sevenSegment.h
  *
  * Created on: Mar 13, 2025
  * Author: Mikhailichenko D.
  * Description: Упрвления modbusRTU
******************************************************************************/
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H
#include "main.h"
#define MB_ADDR 2

uint16_t crc_mb(uint8_t *buf, uint8_t len);
uint8_t parse(int8_t *in_buffer, uint16_t buffer_size, int16_t *temp);


#endif  /* __MODBUS_RTU_H */
