#ifndef TIMER_H
#define TIMER_H

#include "stm32f10x.h"
#include <stdint.h>

void Timer_Init(void);
void StatusLedTimer_Init(void);

uint32_t GetTick(void);
void DelayMs(uint32_t ms);

uint8_t Timeout(uint32_t startTime,uint32_t delay);

#endif
