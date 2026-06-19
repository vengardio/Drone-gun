#include "cyclops_tx.h"
#include "usart.h"

#define CYCLOPS_START_BYTE 0xB7
#define CYCLOPS_HEADER_SIZE 6

static uint8_t XorData(uint8_t *data, uint16_t start, uint16_t size)
{
    uint8_t res = 0;
    uint16_t i;

    for(i=start;i<size;i++)
        res ^= data[i];

    return res;
}

uint16_t CyclopsTx_BuildPacket(uint8_t adr,
                               uint8_t packet_id,
                               uint8_t cmd,
                               uint8_t *data,
                               uint8_t data_count,
                               uint8_t *out)
{
    uint16_t i;
    uint16_t pos = CYCLOPS_HEADER_SIZE;

    out[0] = CYCLOPS_START_BYTE;
    out[1] = adr;
    out[2] = packet_id & 0x7F;
    out[3] = cmd;
    out[4] = data_count;
    out[5] = XorData(out, 1, 5);

    for(i=0;i<data_count;i++)
        out[pos++] = data[i];

    if(data_count > 0)
        out[pos++] = XorData(out, CYCLOPS_HEADER_SIZE, CYCLOPS_HEADER_SIZE + data_count);

    return pos;
}

void Cyclops_SendResponse(uint8_t packet_id, uint8_t cmd, uint8_t *payload, uint8_t payload_size)
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