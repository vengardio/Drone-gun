#include "message_queue.h"
#include "stm32f10x.h"

MessageQueue cyclops_message_queue;
MessageQueue mavlink_message_queue;

static uint8_t NextIndex(uint8_t index)
{
    index++;
    if(index >= MESSAGE_QUEUE_SIZE)
        index = 0;

    return index;
}

static void CopyMessage(Message *destination,
                        MessageSource source,
                        uint8_t *data,
                        uint16_t size)
{
    uint16_t i;

    if(size > MESSAGE_MAX_SIZE)
        size = MESSAGE_MAX_SIZE;

    destination->source = source;
    destination->size = size;

    for(i = 0; i < size; i++)
        destination->data[i] = data[i];
}

void MessageQueue_Push(MessageQueue *queue,
                       MessageSource source,
                       uint8_t *data,
                       uint16_t size)
{
    uint8_t next;

    if(queue == 0)
        return;

    next = NextIndex(queue->head);
    CopyMessage(&queue->items[queue->head], source, data, size);
    queue->head = next;

    if(queue->head == queue->tail)
        queue->tail = NextIndex(queue->tail);
}

uint8_t MessageQueue_Pop(MessageQueue *queue, Message *msg)
{
    uint8_t result = 0;

    if((queue == 0) || (msg == 0))
        return 0;

    __disable_irq();

    if(queue->tail != queue->head)
    {
        *msg = queue->items[queue->tail];
        queue->tail = NextIndex(queue->tail);
        result = 1;
    }

    __enable_irq();
    return result;
}

uint8_t MessageQueue_PeekLast(MessageQueue *queue, Message *msg)
{
    uint8_t result = 0;
    uint8_t last;

    if((queue == 0) || (msg == 0))
        return 0;

    __disable_irq();

    if(queue->tail != queue->head)
    {
        last = (queue->head == 0) ?
               (MESSAGE_QUEUE_SIZE - 1) :
               (queue->head - 1);
        *msg = queue->items[last];
        result = 1;
    }

    __enable_irq();
    return result;
}

void MessageQueue_Clear(MessageQueue *queue)
{
    if(queue == 0)
        return;

    __disable_irq();
    queue->tail = queue->head;
    __enable_irq();
}
