#include "timer.h"

static volatile uint32_t Tick=0;

void Timer_Init(void)
{
    SysTick_Config(SystemCoreClock/1000);
}

void SysTick_Handler(void)
{
    Tick++;
}

uint32_t GetTick(void)
{
    return Tick;
}

void DelayMs(uint32_t ms)
{
    int range = ms * 72000;
		for (int i = 0; i < range; i++)
		{}
		
}

uint8_t Timeout(uint32_t startTime,uint32_t delay)
{
    return ((Tick-startTime)>=delay);
}