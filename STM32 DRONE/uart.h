#ifndef UART_H
#define UART_H

#include "stm32f10x.h"
#include <stdint.h>

void UART_Init(uint32_t baud);

void UART_SendByte(uint8_t data);
void UART_SendArray(uint8_t *data,uint16_t size);

uint8_t UART_DataReady(void);
uint8_t UART_ReadByte(void);

#endif