#include "message_queue.h"
#include "stm32f10x.h"

#define MESSAGE_QUEUE_SIZE 8

static volatile uint8_t head;
static volatile uint8_t tail;
static Message queue[MESSAGE_QUEUE_SIZE];

/* Returns next ring buffer index. */
static uint8_t NextIndex(uint8_t x)
{
    x++;
    if(x >= MESSAGE_QUEUE_SIZE)
        x = 0;

    return x;
}

/* Copies raw packet bytes into a queue slot. */
static void CopyMessage(Message *dst, MessageSource source, uint8_t *data, uint16_t size)
{
    uint16_t i;

    if(size > MESSAGE_MAX_SIZE)
        size = MESSAGE_MAX_SIZE;

    dst->source = source;
    dst->size = size;

    for(i=0;i<size;i++)
        dst->data[i] = data[i];
}

/* Adds one validated message to the queue. Oldest message is overwritten if full. */
void MessageQueue_Push(MessageSource source, uint8_t *data, uint16_t size)
{
    uint8_t next = NextIndex(head);

    CopyMessage(&queue[head], source, data, size);
    head = next;

    if(head == tail)
        tail = NextIndex(tail);
}

/* Reads the oldest message and removes it from the queue. */
uint8_t MessageQueue_Pop(Message *msg)
{
    uint8_t res = 0;

    __disable_irq();

    if(tail != head)
    {
        *msg = queue[tail];
        tail = NextIndex(tail);
        res = 1;
    }

    __enable_irq();

    return res;
}

/* Reads the newest message and drops older queued messages. */
uint8_t MessageQueue_ReadLast(Message *msg)
{
    uint8_t res = 0;
    uint8_t last;

    __disable_irq();

    if(tail != head)
    {
        last = (head == 0) ? (MESSAGE_QUEUE_SIZE - 1) : (head - 1);
        *msg = queue[last];
        tail = head;
        res = 1;
    }

    __enable_irq();

    return res;
}

/* Reads the newest message from selected source and drops older queued messages. */
uint8_t MessageQueue_ReadLastBySource(MessageSource source, Message *msg)
{
    uint8_t res = 0;
    uint8_t i;

    __disable_irq();

    i = head;

    while(i != tail)
    {
        i = (i == 0) ? (MESSAGE_QUEUE_SIZE - 1) : (i - 1);

        if(queue[i].source == source)
        {
            *msg = queue[i];
            tail = head;
            res = 1;
            break;
        }
    }

    __enable_irq();

    return res;
}
