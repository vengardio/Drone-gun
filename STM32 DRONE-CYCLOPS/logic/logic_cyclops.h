#ifndef LOGIC_CYCLOPS_H
#define LOGIC_CYCLOPS_H

#include "message_queue.h"
#include <stdbool.h>

#define CYCLOPS_MASTER_ADDRESS  0xCA
#define CMD_UNKNOWN             0x00
#define CMD_PING                0x81
#define CMD_HW_RESET            0x82
#define CMD_SW_RESET            0x83
#define CMD_GET_ERROR           0x84
#define CMD_ZEROIZE_ERROR       0x85
#define CMD_SET_PARAM           0x86
#define CMD_GET_PARAM           0x87

#define PARAM_GET_ACC_STATE          0x01
#define PARAM_GET_DRONE_STATE  0x02
#define PARAM_GET_DRONE_INFO   0x03
#define PARAM_ENGINES_START_STOP     0x12
#define PARAM_DRONE_LAUNCH     0x13
#define PARAM_SET_ORIENT             0x14

static uint8_t last_error = 0;
static uint8_t errors[8] = {0};
static uint8_t engines_state = 0;
static uint8_t interceptor_state = 0;
static uint16_t orient_azimuth = 0;
static int16_t orient_elevation = 0;

static bool pending_hw_reset_response = false;
static bool pending_sw_reset_response = false;
static uint8_t saved_packet_id_for_reset = 0;

static uint16_t cached_drone_voltage_mv = 25200;
static int16_t cached_drone_temp = 25;
static uint8_t cached_drone_battery_pct = 100;
static uint64_t cached_drone_serial = 0;
static uint32_t cached_drone_version = 10;
static uint8_t cached_drone_target_count = 0;
static uint8_t cached_drone_target_intensity = 0;
static uint8_t cached_drone_system_status = 0;

void LogicCyclops_Process(void);
void LogicCyclops_ProcessMessage(Message *msg);

extern uint8_t LastError;
extern uint8_t txBuf[32];

void Cyclops_UpdateDroneBattery(uint16_t voltage_mv, int16_t temp, uint8_t pct);
void Cyclops_UpdateDroneInfo(uint64_t serial, uint32_t version);
void Cyclops_UpdateDroneTarget(uint8_t count, uint8_t intensity);
void Cyclops_UpdateDroneStatus(uint8_t status);
static void Cyclops_SendResponse(uint8_t packet_id, uint8_t cmd, uint8_t *payload, uint8_t payload_size);
static void Cyclops_SendUnknown(uint8_t packet_id);

#endif
