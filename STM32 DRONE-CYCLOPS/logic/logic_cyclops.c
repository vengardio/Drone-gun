#include "logic_cyclops.h"
#include "cyclops_tx.h"
#include "usart.h"
#include "mavlink_tx.h"
#include "logic_drone.h"
#include "gpio.h"
#include "pwm.h"
#include "timer.h"
#include <stdbool.h>



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
        // Запоминаем packet_id и ждём heartbeat перед ответом
        //=============================
        case CMD_HW_RESET:
            pending_hw_reset_response = true;
            saved_packet_id_for_reset = packet_id;
            break;

        //=============================
        //         SOFTWARE RESET
        // Запоминаем packet_id и ждём heartbeat перед ответом
        //=============================
        case CMD_SW_RESET:
            pending_sw_reset_response = true;
            saved_packet_id_for_reset = packet_id;
            break;

        //=============================
        //          GET ERROR
        // Читаем ошибку по индексу и возвращаем её вместе с last_error
        //=============================
        case CMD_GET_ERROR:
        {
            uint8_t payload[3];
            uint8_t pos = 0;
            uint8_t value = 0;

            if(data_count < 1)
            {
                Cyclops_SendUnknown(packet_id);
                return;
            }

            pos = op[0];
            if(pos < 8)
            {
                value = errors[pos];
            }

            payload[0] = pos;
            payload[1] = value;
            payload[2] = last_error;

            Cyclops_SendResponse(packet_id, CMD_GET_ERROR, payload, 3);
            break;
        }

        //=============================
        //        ZEROIZE ERROR
        // Сбрасываем все ошибки и очищаем last_error
        //=============================
        case CMD_ZEROIZE_ERROR:
        {
            uint8_t payload[1];
            uint8_t i;

            for(i = 0; i < 8; i++)
            {
                errors[i] = 0;
            }

            last_error = 0;
            payload[0] = last_error;

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

void LogicCyclops_OnHeartbeatReceived(void)
{
    if(pending_hw_reset_response || pending_sw_reset_response)
    {
        pending_hw_reset_response = false;
        pending_sw_reset_response = false;

        uint8_t payload[1];
        payload[0] = last_error;

        Cyclops_SendResponse(saved_packet_id_for_reset, CMD_HW_RESET, payload, 1);
    }
}
