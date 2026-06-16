#include "fsm.h"

#include "servo.h"
#include "gpio.h"
#include "drone.h"
#include "timer.h"

#include <stdio.h>

FSM_State State;

static uint32_t StateTime;
static uint8_t lastLaunch;

static void FSM_SetState(FSM_State newState)
{
    State=newState;
    StateTime=GetTick();
}

void FSM_Init(void)
{
    lastLaunch = Launch_Read();

    if(End_Read())
    {
        Servo_Lock();
        /* ПЕРЕХОД В FSM_WAIT_SAFETY */
        FSM_SetState(FSM_WAIT_SAFETY);
    }
    else
    {
        Servo_Release();
        /* ПЕРЕХОД В FSM_WAIT_END */
        FSM_SetState(FSM_WAIT_END);
    }
}

void FSM_Process(void)
{
    uint8_t launch = Launch_Read();
    uint8_t safety = Safety_Read();
    uint8_t end = End_Read();

    if(!Timeout(StateTime, 200))
        return;

    if(State == FSM_WAIT_END)
    {
        /* ЖДЁМ END */
        if(end)
        {
            Servo_Lock();
            /* ПЕРЕХОД В FSM_WAIT_SAFETY */
            FSM_SetState(FSM_WAIT_SAFETY);
        }
    }
    else if(State == FSM_WAIT_SAFETY)
    {
        /* ЖДЁМ SAFETY */
        if(!end)
        {
            Servo_Release();
            /* ПЕРЕХОД В FSM_WAIT_END */
            FSM_SetState(FSM_WAIT_END);
        }
        else if(safety)
        {
            /* ОТПРАВЛЯЕМ ДРОНУ PREPARE */
            Drone_SendPrepare();
            /* ПЕРЕХОД В FSM_PREPARED */
            FSM_SetState(FSM_PREPARED);
        }
    }
    else if(State == FSM_PREPARED)
    {
        /* ЖДЁМ LAUNCH ИЛИ ОТКЛЮЧЕНИЕ SAFETY */
        if(!safety)
        {
            /* ОТПРАВЛЯЕМ ДРОНУ CANCEL */
            Drone_SendCancel();
            /* ПЕРЕХОД В FSM_CANCELLED */
            FSM_SetState(FSM_CANCELLED);
        }
        else if(launch && !lastLaunch)
        {
            Servo_Release();
            DelayMs(200);
            /* ОТПРАВЛЯЕМ ДРОНУ FLY */
            Drone_SendFly();
            /* ПЕРЕХОД В FSM_FIRED */
            FSM_SetState(FSM_FIRED);
        }
    }
    else if(State == FSM_CANCELLED)
    {
        /* ЖДЁМ 2 СЕКУНДЫ ПОСЛЕ CANCEL */
        if(Timeout(StateTime, 2000))
        {
            Servo_Release();
            DelayMs(200);
            /* ПЕРЕХОД В FSM_WAIT_END */
            FSM_SetState(FSM_WAIT_END);
        }
    }

    lastLaunch = launch;
}
