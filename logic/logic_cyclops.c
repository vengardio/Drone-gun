#include "logic_cyclops.h"
#include "cyclops_tx.h"
#include "mavlink_tx.h"
#include "mavlink_rx.h"
#include "common/mavlink.h"
#include "timer.h"
#include "pwm.h"

#include <stdint.h>

/*==============================================================================
*                        DroneResponseWaitCycle
*         Цикл ожидания для работы формата Запрос-ответ Ружьё-дрон
*=============================================================================*/
uint8_t DroneResponseWaitCycle(uint8_t requested_msg_id, uint32_t TimeOut) {
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

/*==============================================================================
*                        LogicCyclops_ProcessMessage
*                  Основной цикл прочтения сообщения Циклопа
*=============================================================================*/
void LogicCyclops_ProcessMessage(Message *msg) {
	uint8_t packet_id;
  uint8_t cmd;
  uint8_t data_count;
  uint8_t *op;

  if(msg->size < 6)
        return;

  packet_id = msg->data[2];
  cmd = msg->data[3];
  data_count = msg->data[4];
  op = &msg->data[6];
	
	switch (cmd) {
		//=============================
    //             PING
    // Проверка связи и отправка последней ошибки
    //=============================
    case CMD_PING:
    {
			uint8_t payload[1];
			
			// Ожидание следующего HEARTBEAT пакета
			MessageQueue_ClearBySource(MESSAGE_SOURCE_DRONE);
			if (DroneResponseWaitCycle(MAVLINK_MSGID_HEARTBEAT, DRONE_REQUEST_TIMEOUT)) {
				payload[0] = 0;
				Cyclops_SendResponse(packet_id, CMD_PING, payload, 1);
			}
			break;
		}
		
		//=============================
    //        HARDWARE RESET
    // Отправляем reset дрону, ждём новый heartbeat и отвечаем Циклопу
    //=============================
    case CMD_HW_RESET:
    {
			uint8_t payload[1];
			
			MessageQueue_ClearBySource(MESSAGE_SOURCE_DRONE);
			MavlinkTx_HwReset();
			// Ожидание следующего SYSTEM_TIME пакета
			if (DroneResponseWaitCycle(MAVLINK_MSGID_SYSTEM_TIME, DRONE_REQUEST_TIMEOUT)) {
				payload[0] = 0;
				Cyclops_SendResponse(packet_id, CMD_HW_RESET, payload, 1);
			} else {
				Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
			}
			
			break;
		}
		//=============================
    //         SOFTWARE RESET
    // Отправляем reset дрону, ждём новый heartbeat и отвечаем Циклопу
    //=============================
    case CMD_SW_RESET:
    {
  		uint8_t payload[1];
			
			MessageQueue_ClearBySource(MESSAGE_SOURCE_DRONE);
			MavlinkTx_SwReset();
			// Ожидание следующего HEARTBEAT пакета
			if (DroneResponseWaitCycle(MAVLINK_MSGID_HEARTBEAT, DRONE_REQUEST_TIMEOUT)) {
				payload[0] = 0;
				Cyclops_SendResponse(packet_id, CMD_SW_RESET, payload, 1);
			} else {
				Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
			}
			
			break;
		}
		//=============================
    //          GET ERROR
    // Всегда отвечаем, что ошибок нет.
    //=============================
		case CMD_GET_ERROR:
    {
      uint8_t payload[3] = {op[0], 0, 0};
      Cyclops_SendResponse(packet_id, CMD_GET_ERROR, payload, 3);
      break;
    }
		//=============================
    //        ZEROIZE ERROR
    // Всегда отвечаем, что ошибки обнулены.
    //=============================
    case CMD_ZEROIZE_ERROR:
    {
      uint8_t payload[1] = {0};
      Cyclops_SendResponse(packet_id, CMD_ZEROIZE_ERROR, payload, 1);
      break;
    }
		//=============================
    //          SET PARAM
    // Изменение состояния перехватчика по выбранному параметру
    //=============================
    case CMD_SET_PARAM:
		{
			// ОШИБКА в сообщении нет даты
      if(data_count < 1){
        Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
        return;
      }

      switch(op[0])
      {
				//
        //       ЗАПУСК ДВИГАТЕЛЕЙ
        // Проверяем BWA и шлём prepare/cancel
        //
        case PARAM_ENGINES_START_STOP:
        {
					uint8_t payload[3];
					
					// ОШИБКА контрольных данных
					if (op[1] != 0x42 |
							op[2] != 0x57 |
							op[3] != 0x41) {
						Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
						return;
					}
							
					if (op[4] == 1)  MavlinkTx_SendPrepare();
					else MavlinkTx_SendCancel();
					
					payload[0] = PARAM_ENGINES_START_STOP;
					payload[1] = op[4];
					payload[2] = 0;
					Cyclops_SendResponse(packet_id, CMD_SET_PARAM, payload, 3);
					
				  break;
				}
				//
        //          ЗАПУСК ДРОНА
        // Проверяем BWA, отдаём сервопривод и отправляем команду полёта
        //
        case PARAM_DRONE_LAUNCH:
        {
					// ОШИБКА контрольных данных
					if (op[1] != 0x42 |
							op[2] != 0x57 |
							op[3] != 0x41) {
						Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
						return;
					}
							
					if (op[4] == 1)  {
						//        ВНИМАНИЕ! АХТУНГ! ВЫЛЕТ ДРОНА! Сложная логика
          
					//Отправка "Лети"
					MavlinkTx_SendFly();
					
					//Ожидание ACK или NACK
					MessageQueue_ClearBySource(MESSAGE_SOURCE_DRONE);
					uint8_t MessageGot = DroneResponseWaitCycle(MAVLINK_MSGID_ACK, 2000);
					
					Message msg;
					uint8_t FlyError = 0;
					mavlink_message_t mav_msg;
					uint8_t payload[5];
					
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
						payload[0] = PARAM_DRONE_LAUNCH;
						payload[1] = 1;
						payload[2] = 2;
						payload[3] = 1;
						payload[4] = 0;
						
						Servo_Release();
						DelayMs(200);
					} else {
					//Если NACK (FlyError = 1)
						payload[0] = PARAM_DRONE_LAUNCH;
						payload[1] = 0;
						payload[2] = 15;
						payload[3] = 1;
						payload[4] = 0;
						
					}
					
					Cyclops_SendResponse(packet_id, CMD_SET_PARAM, payload, 5);
					
					break;
				}
				//
        //       УСТАНОВКА НАВЕДЕНИЯ
        // Читаем азимут и угол места из payload
        //
        case PARAM_SET_ORIENT:
        {
					Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
					break;
				}
				default:
          Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
          break;
			}
			break;
		}
		//
    //          GET PARAM
    // Отдаём состояние, батарею или инфу по перехватчику
    //
    case CMD_GET_PARAM:
		{
      if(data_count < 1)
      {
        Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
        return;
      }

			switch(op[0])
      {
				//
        //       СОСТОЯНИЕ БАТАРЕИ
        // Напряжение, процент заряда, температура и last_error
        //
        case PARAM_GET_ACC_STATE:
        {
					Message msg;
					mavlink_message_t mav_msg;
					
					// 3 попытки дождаться BATTERY STATUS
					for (int i = 0; i < 3; i++) {
						//Проверка что сообщение пришло
						MessageQueue_ClearBySource(MESSAGE_SOURCE_DRONE);
						if (DroneResponseWaitCycle(MAVLINK_MSGID_BATTERY_STATUS, DRONE_REQUEST_TIMEOUT)) {
							MessageQueue_PeekLast(&msg);
							
							//Проверка что оно конвертируется в mavlink
							if (Convert_msg_to_mavlink(&msg, &mav_msg)) 
								break;
							}
						//Если не конвертируется и уже 3 попытки было, отправляем ошибку
							if (i >= 2) {
								Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
								return;
						}
					}
					
					//Обработка принятого сообщения
					MessageQueue_PeekLast(&msg);
					Convert_msg_to_mavlink(&msg, &mav_msg);
					
					mavlink_battery_status_t battery;
					mavlink_msg_battery_status_decode(&mav_msg, &battery);
					uint16_t voltage_mv = battery.voltages[0];
					
					int8_t temperature_c = (int8_t)(battery.temperature / 100);
					
					//Отправка ответа
					uint8_t payload[12];
					payload[0] = PARAM_GET_ACC_STATE;
					payload[1] = (uint8_t)(voltage_mv >> 8);
					payload[2] = (uint8_t)voltage_mv;
					payload[3] = (uint8_t)battery.battery_remaining;
					for (int i = 4; i <= 9; i++) payload[i] = 0;
					payload[10] = (uint8_t)temperature_c;
					payload[11] = 0;
					
					Cyclops_SendResponse(packet_id, CMD_GET_PARAM, payload, 12);
					break;
				}
				//
        //      СОСТОЯНИЕ ПЕРЕХВАТЧИКА
        // Статус, наведение, цель и мощность цели
        //
				case PARAM_GET_DRONE_STATE:
				{
					Message msg;
					mavlink_message_t mav_msg;
					
					// 3 попытки дождаться DEBUG
					for (int i = 0; i < 3; i++) {
						MessageQueue_ClearBySource(MESSAGE_SOURCE_DRONE);
						MavlinkTx_SendGetDetections();
						//Проверка что сообщение пришло
						if (DroneResponseWaitCycle(MAVLINK_MSGID_DEBUG, DRONE_REQUEST_TIMEOUT)) {
							MessageQueue_PeekLast(&msg);
							
							//Проверка что оно конвертируется в mavlink
							if (Convert_msg_to_mavlink(&msg, &mav_msg)) 
								break;
							}
						//Если не конвертируется и уже 3 попытки было, отправляем ошибку
							if (i >= 2) {
								Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
								return;
						}
					}
					
					//Обработка принятого сообщения
					MessageQueue_PeekLast(&msg);
					Convert_msg_to_mavlink(&msg, &mav_msg);
					
					mavlink_debug_t debug;

					mavlink_msg_debug_decode(&mav_msg, &debug);
					
					uint8_t target_count;
					uint8_t target_intensity;

					target_count = debug.ind;

					if(target_count > 15)
							target_count = 15;

					if(debug.value < 0.0f)
							target_intensity = 0;
					else if(debug.value > 100.0f)
							target_intensity = 100;
					else
							target_intensity = (uint8_t)debug.value;
					
					//Отправка циклопу
					uint8_t payload[13];
					
					payload[0] = PARAM_GET_DRONE_STATE;                      // OP1
					payload[1] = ((target_count > 0 ? 2 : 0) << 4) | target_count;    // OP2

				  for (int i = 2; i <= 9; i++) payload[i] = 0; 

					payload[10] = target_count > 0 ? 1 : 0; // OP11: номер цели
					payload[11] = target_intensity;         // OP12: интенсивность
					payload[12] = 0;                        // OP13: LAST_ERROR

					Cyclops_SendResponse(packet_id, CMD_GET_PARAM, payload, 13);
					break;
				}			
				//=============================
        //       ИНФО ПЕРЕХВАТЧИКА
        // Серийник, версия прошивки и last_error
        //=============================
				case PARAM_GET_DRONE_INFO:
				{
					Message msg;
					mavlink_message_t mav_msg;
					
					// 3 попытки дождаться SYSTEM_TIME
					for (int i = 0; i < 3; i++) {
						MessageQueue_ClearBySource(MESSAGE_SOURCE_DRONE);
						MavlinkTx_SendInit();
						//Проверка что сообщение пришло
						if (DroneResponseWaitCycle(MAVLINK_MSGID_SYSTEM_TIME, DRONE_REQUEST_TIMEOUT)) {
							MessageQueue_PeekLast(&msg);
							
							//Проверка что оно конвертируется в mavlink
							if (Convert_msg_to_mavlink(&msg, &mav_msg)) 
								break;
							}
						//Если не конвертируется и уже 3 попытки было, отправляем ошибку
							if (i >= 2) {
								Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
								return;
						}
					}
					
					//Обработка принятого сообщения
					MessageQueue_PeekLast(&msg);
					Convert_msg_to_mavlink(&msg, &mav_msg);
					
					mavlink_system_time_t system_time;

					mavlink_msg_system_time_decode(&mav_msg, &system_time);
					
					uint8_t payload[8];

					payload[0] = PARAM_GET_DRONE_INFO;

					/* Последние четыре байта time_unix_usec */
					payload[1] = (uint8_t)(system_time.time_unix_usec >> 24);
					payload[2] = (uint8_t)(system_time.time_unix_usec >> 16);
					payload[3] = (uint8_t)(system_time.time_unix_usec >> 8);
					payload[4] = (uint8_t)system_time.time_unix_usec;

					/* Последние два байта time_boot_ms */
					payload[5] = (uint8_t)(system_time.time_boot_ms >> 8);
					payload[6] = (uint8_t)system_time.time_boot_ms;

					payload[7] = 0; 

					Cyclops_SendResponse(packet_id, CMD_GET_PARAM, payload, 8);
					break;
				}
				default: 
					Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
					break;
			}
		}
		default: 
			Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
			break;
		}
	}
}