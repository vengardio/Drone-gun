#include "logic_drone.h"

#include "pwm.h"
#include "gpio.h"
#include "mavlink_tx.h"
#include "timer.h"

LogicDroneState logic_drone_state;

static DroneState drone_state;
static uint32_t stateTime;
static uint8_t lastLaunch;

static void LogicDrone_SetState(LogicDroneState newState)
{
    logic_drone_state = newState;
    stateTime = GetTick();
}

void LogicDrone_Init(void)
{
    drone_state = DRONE_STATE_IDLE;
    lastLaunch = Launch_Read();

    if(End_Read())
    {
        Servo_Hold();
        LogicDrone_SetState(LOGIC_DRONE_WAIT_SAFETY);
    }
    else
    {
        Servo_Release();
        LogicDrone_SetState(LOGIC_DRONE_WAIT_END);
    }
}

void LogicDrone_Process(void)
{
    uint8_t launch = Launch_Read();
    uint8_t safety = Safety_Read();
    uint8_t end = End_Read();

    if(!Timeout(stateTime, 200))
        return;

    if(logic_drone_state == LOGIC_DRONE_WAIT_END)
    {
        if(end)
        {
            Servo_Hold();
            LogicDrone_SetState(LOGIC_DRONE_WAIT_SAFETY);
        }
    }
    else if(logic_drone_state == LOGIC_DRONE_WAIT_SAFETY)
    {
        if(!end)
        {
            Servo_Release();
            LogicDrone_SetState(LOGIC_DRONE_WAIT_END);
        } 
				else if (launch && !lastLaunch) 
				{
					Servo_Release();
          LogicDrone_SetState(LOGIC_DRONE_WAIT_END);
				}
        else if(safety)
        {
            MavlinkTx_SendPrepare();
            LogicDrone_SetDroneState(DRONE_STATE_PREPARED);
            LogicDrone_SetState(LOGIC_DRONE_PREPARED);
        }
    }
    else if(logic_drone_state == LOGIC_DRONE_PREPARED)
    {
        if(!safety)
        {
            MavlinkTx_SendCancel();
            LogicDrone_SetDroneState(DRONE_STATE_CANCELLED);
            LogicDrone_SetState(LOGIC_DRONE_CANCELLED);
        }
        else if(launch && !lastLaunch)
        {
            Servo_Release();
            DelayMs(200);
            MavlinkTx_SendFly();
            LogicDrone_SetDroneState(DRONE_STATE_IDLE);
            LogicDrone_SetState(LOGIC_DRONE_WAIT_END);
        }
    }
    else if(logic_drone_state == LOGIC_DRONE_CANCELLED)
    {
        LogicDrone_SetDroneState(DRONE_STATE_IDLE);
        LogicDrone_SetState(LOGIC_DRONE_WAIT_END);
    }

    lastLaunch = launch;
}

void LogicDrone_ProcessMessage(Message *msg)
{
    (void)msg;
}

void LogicDrone_SetDroneState(DroneState state)
{
    drone_state = state;
}

DroneState LogicDrone_GetDroneState(void)
{
    return drone_state;
}
