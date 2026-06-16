#include "rcc.h"
#include "gpio.h"
#include "uart.h"
#include "timer.h"

#include "pwm.h"
#include "servo.h"
#include "heartbeat.h"
#include "drone.h"
#include "fsm.h"

int main(void)
{
    RCC_Init();
    GPIO_Init();
    Optocouplers_Enable();

    Timer_Init();
    UART_Init(115200);

    Servo_Init();
    Drone_Init();
    FSM_Init();

    while(1)
    {
        FSM_Process();
        Heartbeat_Process();
    }
}
