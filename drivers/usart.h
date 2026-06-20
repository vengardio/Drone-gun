#ifndef USART_H
#define USART_H

#include "stm32f10x.h"
#include <stdint.h>

void USART_Init(uint32_t baud);

void USART2_SendByte(uint8_t data);
void USART2_SendArray(uint8_t *data, uint16_t size);

void USART1_SendByte(uint8_t data);
void USART1_SendArray(uint8_t *data, uint16_t size);

#endif
