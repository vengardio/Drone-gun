#ifndef GPIO_H
#define GPIO_H

#include "stm32f10x.h"
#include <stdint.h>

void GPIO_Init(void);

uint8_t Launch_Read(void);
uint8_t Safety_Read(void);
uint8_t End_Read(void);

void Optocouplers_Enable(void);
void StatusLed_Toggle(void);

#endif
