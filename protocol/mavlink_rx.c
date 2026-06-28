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

uint8_t MavlinkRx_MessageIsHeartbeat(Message *msg)
{
    if(msg->source != MESSAGE_SOURCE_DRONE)
        return 0;

    if(msg->size < MAVLINK_V2_HEADER_SIZE)
        return 0;

    if(msg->data[0] != MAVLINK_V2_START)
        return 0;

    return (msg->data[7] == 0) && (msg->data[8] == 0) && (msg->data[9] == 0);
}

/* Переводит принятые сырые данные из USART2 в собранный пакет (не сока) */
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
            MessageQueue_Push(
                &mavlink_message_queue,
                MESSAGE_SOURCE_DRONE,
                buf,
                pos
            );

        ResetParser();
        return;
    }

    if(expected && pos >= expected)
        ResetParser();
}

//Расшифровывает принятый сырой пакет из msg в сообщение mavlink
uint8_t Convert_msg_to_mavlink(Message *raw_msg,
                               mavlink_message_t *mav_msg)
{
    mavlink_status_t status;
    uint16_t i;

    if((raw_msg == 0) || (mav_msg == 0))
        return 0;

    if(raw_msg->source != MESSAGE_SOURCE_DRONE)
        return 0;

    for(i = 0; i < raw_msg->size; i++)
    {
        if(mavlink_parse_char(
            MAVLINK_COMM_2,
            raw_msg->data[i],
            mav_msg,
            &status))
        {
            return 1;
        }
    }

    return 0;
}
