#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <stdint.h>

#define MESSAGE_MAX_SIZE 280
#define MESSAGE_QUEUE_SIZE 8

typedef enum
{
    MESSAGE_SOURCE_DRONE = 0,
    MESSAGE_SOURCE_CYCLOPS
} MessageSource;

typedef struct
{
    MessageSource source;
    uint16_t size;
    uint8_t data[MESSAGE_MAX_SIZE];
} Message;

typedef struct
{
    volatile uint8_t head;
    volatile uint8_t tail;
    Message items[MESSAGE_QUEUE_SIZE];
} MessageQueue;

extern MessageQueue cyclops_message_queue;
extern MessageQueue mavlink_message_queue;

void MessageQueue_Push(MessageQueue *queue,
                       MessageSource source,
                       uint8_t *data,
                       uint16_t size);
uint8_t MessageQueue_Pop(MessageQueue *queue, Message *msg);
uint8_t MessageQueue_PeekLast(MessageQueue *queue, Message *msg);
void MessageQueue_Clear(MessageQueue *queue);

#endif
