#include "logic_cyclops.h"
#include "cyclops_tx.h"
#include "usart.h"
#include "mavlink_tx.h"
#include "mavlink_rx.h"
#include "logic_drone.h"
#include "gpio.h"
#include "pwm.h"
#include "timer.h"
#include "common/mavlink.h"
#include <stdbool.h>
#include <string.h>

#define DRONE_REQUEST_TIMEOUT_MS 5000
#define RESET_HEARTBEAT_TIMEOUT_MS 10000
#define MAVLINK_MSG_ID_HEARTBEAT_LOCAL 0
#define MAVLINK_HEARTBEAT_SYSTEM_STATUS_OFFSET 7

#define DRONE_REQUEST_HEARTBEAT 0
#define DRONE_REQUEST_SW_RESET  1
#define DRONE_REQUEST_HW_RESET  2
#define DRONE_REQUEST_GET_PARAM 3
#define DRONE_REQUEST_SERIAL    4

static uint8_t last_error = 0;
static uint8_t engines_state = 0;
static uint8_t interceptor_state = INTERCEPTOR_STATE_IDLE;
static uint8_t end_latched = 0;
static uint8_t launch_reset_pending = 0;
static uint16_t orient_azimuth = 0;
static int16_t orient_elevation = 0;

static uint16_t cached_drone_voltage_mv = 25200;
static int16_t cached_drone_temp = 25;
static uint8_t cached_drone_battery_pct = 100;
static uint64_t cached_drone_serial = 0;
static uint32_t cached_drone_version = 10;
static uint8_t cached_drone_target_count = 0;
static uint8_t cached_drone_target_intensity = 0;
static uint8_t cached_drone_system_status = 0;

static uint32_t drone_request_start;
static uint8_t drone_request_packet_id;
static uint8_t drone_request_timed_out;

void Cyclops_UpdateDroneBattery(uint16_t voltage_mv, int16_t temp, uint8_t pct)
{
    cached_drone_voltage_mv = voltage_mv;
    cached_drone_temp = temp;
    cached_drone_battery_pct = pct;
}

void Cyclops_UpdateDroneInfo(uint64_t serial, uint32_t version)
{
    cached_drone_serial = serial;
    cached_drone_version = version;
}

void Cyclops_UpdateDroneTarget(uint8_t count, uint8_t intensity)
{
    cached_drone_target_count = count;
    cached_drone_target_intensity = intensity;
}

void Cyclops_UpdateDroneStatus(uint8_t status)
{
    cached_drone_system_status = status;
}

static uint8_t Cyclops_ParseMavlinkMessage(Message *raw, mavlink_message_t *decoded)
{
    mavlink_status_t status;
    uint16_t i;

    if(raw->source != MESSAGE_SOURCE_DRONE)
        return 0;

    for(i=0;i<raw->size;i++)
    {
        if(mavlink_parse_char(MAVLINK_COMM_2, raw->data[i], decoded, &status))
            return 1;
    }

    return 0;
}

static uint16_t Cyclops_ReadTotalBatteryVoltage(mavlink_battery_status_t *battery)
{
    uint8_t i;
    uint8_t valid_cells = 0;
    uint32_t total_mv = 0;

    for(i=0;i<10;i++)
    {
        if((battery->voltages[i] == 0) || (battery->voltages[i] == UINT16_MAX))
            continue;

        valid_cells++;
        total_mv += battery->voltages[i];
    }

    if(valid_cells == 0)
        return 0;

    if(total_mv > UINT16_MAX)
        return UINT16_MAX;

    return (uint16_t)total_mv;
}

static int16_t Cyclops_ReadBatteryTemperatureC(mavlink_battery_status_t *battery)
{
    if(battery->temperature == INT16_MAX)
        return cached_drone_temp;

    return battery->temperature / 100;
}

static uint8_t Cyclops_ReadHeartbeatStatus(mavlink_message_t *msg)
{
    const uint8_t *payload = (const uint8_t *)_MAV_PAYLOAD(msg);

    if(msg->len <= MAVLINK_HEARTBEAT_SYSTEM_STATUS_OFFSET)
        return cached_drone_system_status;

    return payload[MAVLINK_HEARTBEAT_SYSTEM_STATUS_OFFSET];
}

void LogicCyclops_ProcessDroneMessage(Message *msg)
{
    mavlink_message_t mav_msg;
    mavlink_battery_status_t battery;
    uint16_t voltage_mv;

    if(!Cyclops_ParseMavlinkMessage(msg, &mav_msg))
        return;

    if(mav_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT_LOCAL)
    {
        Cyclops_UpdateDroneStatus(Cyclops_ReadHeartbeatStatus(&mav_msg));
        return;
    }

    if(mav_msg.msgid != MAVLINK_MSG_ID_BATTERY_STATUS)
        return;

    mavlink_msg_battery_status_decode(&mav_msg, &battery);

    voltage_mv = Cyclops_ReadTotalBatteryVoltage(&battery);
    if(voltage_mv)
        cached_drone_voltage_mv = voltage_mv;

    if((battery.battery_remaining >= 0) && (battery.battery_remaining <= 100))
        cached_drone_battery_pct = (uint8_t)battery.battery_remaining;

    cached_drone_temp = Cyclops_ReadBatteryTemperatureC(&battery);
}

static void Cyclops_SendResponse(uint8_t packet_id, uint8_t cmd, uint8_t *payload, uint8_t payload_size)
{
    uint8_t packet[280];
    uint16_t len;

    len = CyclopsTx_BuildPacket(
        CYCLOPS_MASTER_ADDRESS,
        packet_id,
        cmd,
        payload,
        payload_size,
        packet
    );

    USART1_SendArray(packet, len);
}

static void Cyclops_SendUnknown(uint8_t packet_id)
{
    Cyclops_SendResponse(packet_id, CMD_UNKNOWN, 0, 0);
}

static uint8_t DroneRequestValid(uint8_t request_type)
{
    Message msg;
    mavlink_message_t mav_msg;
    uint32_t expected_msgid;

    if(!MessageQueue_PeekLast(&msg))
        return 0;

    if(msg.source != MESSAGE_SOURCE_DRONE)
        return 0;

    if(!Cyclops_ParseMavlinkMessage(&msg, &mav_msg))
        return 0;

    switch(request_type)
    {
        case DRONE_REQUEST_HEARTBEAT:
        case DRONE_REQUEST_SW_RESET:
        case DRONE_REQUEST_HW_RESET:
            expected_msgid = MAVLINK_MSG_ID_HEARTBEAT;
            break;

        case DRONE_REQUEST_GET_PARAM:
            expected_msgid = MAVLINK_MSG_ID_DEBUG;
            break;

        case DRONE_REQUEST_SERIAL:
            expected_msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
            break;

        default:
            return 0;
    }

    return mav_msg.msgid == expected_msgid;
}

static uint8_t Cyclops_CheckBwa(uint8_t *op, uint8_t data_count)
{
    if(data_count < 5)
        return 0;

    return (op[1] == 0x42) && (op[2] == 0x57) && (op[3] == 0x41);
}

static uint16_t ReadU16BE(uint8_t hi, uint8_t lo)
{
    return ((uint16_t)hi << 8) | lo;
}

static int16_t ReadS16BE(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static void WriteU16BE(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)value;
}

static uint8_t MakeInterceptorStateByte(uint8_t state, uint8_t targets)
{
    return (uint8_t)(((state & 0x0F) << 4) | (targets & 0x0F));
}

void LogicCyclops_Process(void)
{
    uint8_t end = End_Read();

    if(launch_reset_pending)
    {
        if(!end)
        {
            Servo_Release();
            LogicDrone_SetDroneState(DRONE_STATE_IDLE);
            interceptor_state = INTERCEPTOR_STATE_IDLE;
            engines_state = 0;
            end_latched = 0;
            launch_reset_pending = 0;
        }

        return;
    }

    if(end && !end_latched)
    {
        Servo_Hold();
        interceptor_state = INTERCEPTOR_STATE_FIXED;
        end_latched = 1;
    }

    if(LogicDrone_GetDroneState() == DRONE_STATE_PREPARED)
    {
        Servo_Hold();
    }
}

void LogicCyclops_ProcessMessage(Message *msg)
{
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

    switch(cmd)
    {
        //=============================
        //             PING
        // Проверка связи и отправка последней ошибки
        //=============================
        case CMD_PING:
        {
            uint8_t payload[1];

            if(cached_drone_system_status == 5 || LogicDrone_GetDroneState() == DRONE_STATE_FIRED)
                return;

            payload[0] = last_error;
            Cyclops_SendResponse(packet_id, CMD_PING, payload, 1);
            break;
        }

        //=============================
        //        HARDWARE RESET
        // Отправляем reset дрону, ждём новый heartbeat и отвечаем Циклопу
        //=============================
        case CMD_HW_RESET:
        {
            Message msg;
            uint8_t payload[1];

            MessageQueue_ClearBySource(MESSAGE_SOURCE_DRONE);

            drone_request_start = GetTick();
            drone_request_packet_id = packet_id;
            drone_request_timed_out = 0;

            MavlinkTx_HwReset();

            while(!DroneRequestValid(DRONE_REQUEST_HW_RESET)) {
							if (TimedOut(drone_request_start, 5000)) {
								Cyclops_SendUnknown(packet_id);
								break;
							}
						}

            MessageQueue_PeekLast(&msg);
            LogicCyclops_ProcessDroneMessage(&msg);

            payload[0] = last_error;
            Cyclops_SendResponse(packet_id, CMD_HW_RESET, payload, 1);
            break;
        }

        //=============================
        //         SOFTWARE RESET
        // Отправляем reset дрону, ждём новый heartbeat и отвечаем Циклопу
        //=============================
        case CMD_SW_RESET:
        {
            Message old_msg;
            Message msg;
            uint32_t start;
            uint8_t old_msg_valid;
            uint8_t heartbeat_received = 0;
            uint8_t payload[1];

            old_msg_valid = MessageQueue_PeekLast(&old_msg);
            MavlinkTx_SwReset();
            start = GetTick();

            while(!heartbeat_received)
            {
                if(Timeout(start, RESET_HEARTBEAT_TIMEOUT_MS))
                {
                    Cyclops_SendUnknown(packet_id);
                    return;
                }

                if(!MessageQueue_PeekLast(&msg))
                    continue;

                if(old_msg_valid &&
                   (msg.source == old_msg.source) &&
                   (msg.size == old_msg.size) &&
                   (memcmp(msg.data, old_msg.data, msg.size) == 0))
                {
                    continue;
                }

                if(MavlinkRx_MessageIsHeartbeat(&msg))
                    heartbeat_received = 1;
            }

            payload[0] = last_error;
            Cyclops_SendResponse(packet_id, CMD_SW_RESET, payload, 1);
            break;
        }

        //=============================
        //          GET ERROR
        // Всегда отвечаем, что ошибок нет.
        //=============================
        case CMD_GET_ERROR:
        {
            uint8_t payload[3] = {0, 0, 0};

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
            if(data_count < 1)
            {
                Cyclops_SendUnknown(packet_id);
                return;
            }

            switch(op[0])
            {
                //=============================
                //       ЗАПУСК ДВИГАТЕЛЕЙ
                // Проверяем BWA, меняем engines_state и шлём prepare/cancel
                //=============================
                case PARAM_ENGINES_START_STOP:
                {
                    uint8_t payload[3];

                    if(Cyclops_CheckBwa(op, data_count))
                    {
                        engines_state = op[4] ? 1 : 0;

                        if(engines_state)
                        {
                            MavlinkTx_SendPrepare();
                        }
                        else
                        {
                            MavlinkTx_SendCancel();
                        }
                    }

                    payload[0] = PARAM_ENGINES_START_STOP;
                    payload[1] = engines_state;
                    payload[2] = last_error;

                    Cyclops_SendResponse(packet_id, CMD_SET_PARAM, payload, 3);
                    break;
                }

                //=============================
                //          ЗАПУСК ДРОНА
                // Проверяем BWA, отдаём сервопривод и отправляем команду полёта
                //=============================
                case PARAM_DRONE_LAUNCH:
                {
                    uint8_t payload[5];
                    uint8_t launched = 0;

                    if(Cyclops_CheckBwa(op, data_count) && (op[4] == 1))
                    {
                        launched = 1;
                        interceptor_state = INTERCEPTOR_STATE_LAUNCHED;
                    }

                    payload[0] = PARAM_DRONE_LAUNCH;
                    payload[1] = launched;
                    payload[2] = interceptor_state;
                    payload[3] = engines_state;
                    payload[4] = last_error;

                    Cyclops_SendResponse(packet_id, CMD_SET_PARAM, payload, 5);

                    if(launched)
                    {
                        Servo_Release();
                        DelayMs(200);
                        MavlinkTx_SendFly();
                        LogicDrone_SetDroneState(DRONE_STATE_FIRED);
                        launch_reset_pending = 1;
                    }

                    break;
                }

                //=============================
                //       УСТАНОВКА НАВЕДЕНИЯ
                // Читаем азимут и угол места из payload
                //=============================
                case PARAM_SET_ORIENT:
                {
                    uint8_t payload[6];

                    if(data_count < 5)
                    {
                        Cyclops_SendUnknown(packet_id);
                        return;
                    }

                    orient_azimuth = ReadU16BE(op[1], op[2]);
                    orient_elevation = ReadS16BE(op[3], op[4]);

                    payload[0] = PARAM_SET_ORIENT;
                    WriteU16BE(&payload[1], 0);
                    WriteU16BE(&payload[3], 0);
                    payload[5] = last_error;

                    Cyclops_SendResponse(packet_id, CMD_SET_PARAM, payload, 6);
                    break;
                }

                default:
                    Cyclops_SendUnknown(packet_id);
                    break;
            }

            break;

        //=============================
        //          GET PARAM
        // Отдаём состояние, батарею или инфу по перехватчику
        //=============================
        case CMD_GET_PARAM:
            if(data_count < 1)
            {
                Cyclops_SendUnknown(packet_id);
                return;
            }

            switch(op[0])
            {
                //=============================
                //       СОСТОЯНИЕ БАТАРЕИ
                // Напряжение, процент заряда, температура и last_error
                //=============================
                case PARAM_GET_ACC_STATE:
                {
                    uint8_t payload[12];
                    uint8_t cell_voltage_offset = 0;

                    payload[0] = PARAM_GET_ACC_STATE;
                    WriteU16BE(&payload[1], cached_drone_voltage_mv);
                    payload[3] = cached_drone_battery_pct;
                    payload[4] = cell_voltage_offset;
                    payload[5] = cell_voltage_offset;
                    payload[6] = cell_voltage_offset;
                    payload[7] = cell_voltage_offset;
                    payload[8] = cell_voltage_offset;
                    payload[9] = cell_voltage_offset;
                    payload[10] = (uint8_t)cached_drone_temp;
                    payload[11] = last_error;

                    Cyclops_SendResponse(packet_id, CMD_GET_PARAM, payload, 12);
                    break;
                }

                //=============================
                //      СОСТОЯНИЕ ПЕРЕХВАТЧИКА
                // Статус, наведение, цель и мощность цели
                //=============================
                case PARAM_GET_DRONE_STATE:
                {
                    uint8_t payload[13];
                    uint8_t targets = cached_drone_target_count;
                    uint8_t target_number = targets > 0 ? 1 : 0;
                    uint8_t target_power = cached_drone_target_intensity;

                    payload[0] = PARAM_GET_DRONE_STATE;
                    payload[1] = MakeInterceptorStateByte(interceptor_state, targets);

                    WriteU16BE(&payload[2], 0);
                    WriteU16BE(&payload[4], 0);
                    WriteU16BE(&payload[6], orient_azimuth);
                    WriteU16BE(&payload[8], (uint16_t)orient_elevation);

                    payload[10] = target_number;
                    payload[11] = target_power;
                    payload[12] = last_error;

                    Cyclops_SendResponse(packet_id, CMD_GET_PARAM, payload, 13);
                    break;
                }

                //=============================
                //       ИНФО ПЕРЕХВАТЧИКА
                // Серийник, версия прошивки и last_error
                //=============================
                case PARAM_GET_DRONE_INFO:
                {
                    uint8_t payload[8];

                    payload[0] = PARAM_GET_DRONE_INFO;
                    payload[1] = (uint8_t)(cached_drone_serial >> 24);
                    payload[2] = (uint8_t)(cached_drone_serial >> 16);
                    payload[3] = (uint8_t)(cached_drone_serial >> 8);
                    payload[4] = (uint8_t)cached_drone_serial;
                    payload[5] = (uint8_t)(cached_drone_version >> 8);
                    payload[6] = (uint8_t)cached_drone_version;
                    payload[7] = last_error;

                    Cyclops_SendResponse(packet_id, CMD_GET_PARAM, payload, 8);
                    break;
                }

                default:
                    Cyclops_SendUnknown(packet_id);
                    break;
            }

            break;

        //=============================
        //       НЕИЗВЕСТНАЯ КОМАНДА
        // Отвечаем CMD_UNKNOWN
        //=============================
        default:
            Cyclops_SendUnknown(packet_id);
            break;
    }
}
