#include "cyclops_rx.h"
#include "message_queue.h"

#define CYCLOPS_START_BYTE 0xB7
#define CYCLOPS_HEADER_SIZE 6
#define CYCLOPS_MAX_SIZE 262

static uint8_t buf[CYCLOPS_MAX_SIZE];
static uint16_t pos;
static uint16_t expected;

/* XOR helper for SrvInt HASH and DATA_HASH fields. */
static uint8_t XorData(uint8_t *data, uint16_t start, uint16_t size)
{
    uint8_t res = 0;
    uint16_t i;

    for(i=start;i<size;i++)
        res ^= data[i];

    return res;
}

/* Drops the current half-collected Cyclops frame. */
static void ResetParser(void)
{
    pos = 0;
    expected = 0;
}

/* Checks START, PACKET_ID, length, HASH and DATA_HASH. */
static uint8_t PacketIsValid(void)
{
    uint8_t data_count;

    if(pos < CYCLOPS_HEADER_SIZE)
        return 0;

    if(buf[0] != CYCLOPS_START_BYTE)
        return 0;

    if(buf[2] & 0x80)
        return 0;

    if(buf[5] != XorData(buf, 1, 5))
        return 0;

    data_count = buf[4];

    if(data_count == 0)
        return pos == CYCLOPS_HEADER_SIZE;

    if(pos != (uint16_t)(CYCLOPS_HEADER_SIZE + data_count + 1))
        return 0;

    return buf[pos - 1] == XorData(buf, CYCLOPS_HEADER_SIZE, CYCLOPS_HEADER_SIZE + data_count);
}

/* Feeds one byte from USART1, stores only a full valid Cyclops frame. */
void Cyclops_RxByte(uint8_t data)
{
    if(pos == 0)
    {
        if(data != CYCLOPS_START_BYTE)
            return;

        buf[pos++] = data;
        return;
    }

    buf[pos++] = data;

    if(pos == CYCLOPS_HEADER_SIZE)
    {
        if(buf[5] != XorData(buf, 1, 5))
        {
            ResetParser();
            if(data == CYCLOPS_START_BYTE)
            {
                buf[pos++] = data;
            }
            return;
        }

        if(buf[4] == 0)
            expected = CYCLOPS_HEADER_SIZE;
        else
            expected = CYCLOPS_HEADER_SIZE + buf[4] + 1;
    }

    if(pos >= CYCLOPS_MAX_SIZE)
    {
        ResetParser();
        return;
    }

    if(expected && pos >= expected)
    {
        if(PacketIsValid())
            MessageQueue_Push(
                &cyclops_message_queue,
                MESSAGE_SOURCE_CYCLOPS,
                buf,
                pos
            );

        ResetParser();
    }
}
