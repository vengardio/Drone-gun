#include "heartbeat.h"
#include "timer.h"
#include "stm32f10x.h"

void Heartbeat_Process(void)
{
    static uint32_t lastTime = 0;

    if(GetTick() - lastTime >= 500)
    {
        lastTime = GetTick();

        GPIOA->ODR ^= GPIO_ODR_ODR5;
    }
}