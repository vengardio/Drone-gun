#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <stdint.h>

#define MESSAGE_MAX_SIZE 280

typedef enum
{
    MESSAGE_SOURCE_DRONE=0,
    MESSAGE_SOURCE_CYCLOPS
}MessageSource;

typedef struct
{
    MessageSource source;
    uint16_t size;
    uint8_t data[MESSAGE_MAX_SIZE];
}Message;

void MessageQueue_Push(MessageSource source, uint8_t *data, uint16_t size);
uint8_t MessageQueue_Pop(Message *msg);
uint8_t MessageQueue_PeekLast(Message *msg);
uint8_t MessageQueue_ReadLast(Message *msg);
uint8_t MessageQueue_ReadLastBySource(MessageSource source, Message *msg);
void MessageQueue_ClearBySource(MessageSource source);

#endif
