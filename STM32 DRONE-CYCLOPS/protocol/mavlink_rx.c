#include "mavlink_rx.h"
#include "message_queue.h"

#include "common/mavlink.h"

#define MAVLINK_V2_START 0xFD
#define MAVLINK_V2_HEADER_SIZE 10
#define MAVLINK_V2_CRC_SIZE 2
#define MAVLINK_V2_SIGNATURE_SIZE 13

static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static uint16_t pos;
static uint16_t expected;

/* Drops the current half-collected MAVLink frame. */
static void ResetParser(void)
{
    pos = 0;
    expected = 0;
}

/* Calculates full MAVLink v2 frame size after receiving the header. */
static void UpdateExpectedSize(void)
{
    uint16_t size;

    if(pos < MAVLINK_V2_HEADER_SIZE)
        return;

    size = MAVLINK_V2_HEADER_SIZE + buf[1] + MAVLINK_V2_CRC_SIZE;

    if(buf[2] & 0x01)
        size += MAVLINK_V2_SIGNATURE_SIZE;

    expected = size;
}

/* Feeds one byte from USART2, validates MAVLink CRC, stores full frame. */
void Mavlink_RxByte(uint8_t data)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    if(pos == 0)
    {
        if(data != MAVLINK_V2_START)
            return;

        buf[pos++] = data;
        mavlink_parse_char(MAVLINK_COMM_1, data, &msg, &status);
        return;
    }

    if(pos >= MAVLINK_MAX_PACKET_LEN)
    {
        ResetParser();
        return;
    }

    buf[pos++] = data;

    if(pos == MAVLINK_V2_HEADER_SIZE)
        UpdateExpectedSize();

    if(mavlink_parse_char(MAVLINK_COMM_1, data, &msg, &status))
    {
        if(expected && pos == expected)
            MessageQueue_Push(MESSAGE_SOURCE_DRONE, buf, pos);

        ResetParser();
        return;
    }

    if(expected && pos >= expected)
        ResetParser();
}
