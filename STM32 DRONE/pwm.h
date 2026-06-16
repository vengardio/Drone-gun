#ifndef PWM_H
#define PWM_H

#include "stm32f10x.h"
#include <stdint.h>

typedef struct
{
    TIM_TypeDef* TIMx;

}PWM_HandleTypeDef;


void PWM_Init(
                PWM_HandleTypeDef *pwm,
                uint32_t frequency
             );

void PWM_SetUs(
                PWM_HandleTypeDef *pwm,
                uint16_t us
              );


extern PWM_HandleTypeDef ServoPWM;

#endif