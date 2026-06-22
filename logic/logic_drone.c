#include "logic_drone.h"

#include "pwm.h"
#include "gpio.h"
#include "mavlink_tx.h"
#include "timer.h"
#include "mavlink_tx.h"
#include "mavlink_rx.h"
#include "common/mavlink.h"
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

/*==============================================================================
*                        DroneResponseWaitCycle
*         Цикл ожидания для работы формата Запрос-ответ Ружьё-дрон
*=============================================================================*/
uint8_t LogicDrone_DroneResponseWaitCycle(uint8_t requested_msg_id, uint32_t TimeOut) {
	Message msg;
	mavlink_message_t mav_msg;
	uint8_t msg_arrived = 0;
	uint32_t startTime = GetTick();
	
	
	while(!msg_arrived) {
		MessageQueue_PeekLast(&msg);
		
		//Условия выхода с ошибкой
		if (GetTick()-startTime > TimeOut) {   
			return 0;
		}
		
		//Условие продолжения ожидания
		if (!Convert_msg_to_mavlink(&msg, &mav_msg)) continue;
		
		//Условие успешного выхода
		if(msg.source == MESSAGE_SOURCE_DRONE) {
			if(mav_msg.msgid == requested_msg_id) msg_arrived = 1;
		}
	}
	return 1;
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
					//        ВНИМАНИЕ! АХТУНГ! ВЫЛЕТ ДРОНА! Сложная логика
          
					//Отправка "Лети"
					MavlinkTx_SendFly();
					
					//Ожидание ACK или NACK
					MessageQueue_ClearBySource(MESSAGE_SOURCE_DRONE);
					uint8_t MessageGot = LogicDrone_DroneResponseWaitCycle(MAVLINK_MSGID_ACK, 2000);
					
					Message msg;
					uint8_t FlyError = 0;
					mavlink_message_t mav_msg;
					
					if (MessageGot == 1) {
						MessageQueue_PeekLast(&msg);
						
						if (Convert_msg_to_mavlink(&msg, &mav_msg)) {
							mavlink_command_ack_t command_ack;
							mavlink_msg_command_ack_decode(&mav_msg, &command_ack);
							
							if (command_ack.result == 0) FlyError = 1;
						} else {
							FlyError = 1;
						}
					} else {
						FlyError = 1;
					}
					
					//Если ACK 
					if (FlyError == 0) {
						Servo_Release();
						DelayMs(200);
					} else {
					//Если NACK (FlyError = 1)
						LogicDrone_SetDroneState(DRONE_STATE_IDLE);
						LogicDrone_SetState(LOGIC_DRONE_WAIT_END);
					}
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
