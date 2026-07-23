#ifndef PC_SIM_QUEUE_H
#define PC_SIM_QUEUE_H

#include "FreeRTOS.h"

static inline BaseType_t xQueueSend(QueueHandle_t q, const void * item, uint32_t wait)
{
    (void)q;
    (void)item;
    (void)wait;
    return pdTRUE;
}

#endif
