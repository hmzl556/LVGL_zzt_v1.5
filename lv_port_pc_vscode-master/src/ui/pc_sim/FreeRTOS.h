#ifndef PC_SIM_FREERTOS_H
#define PC_SIM_FREERTOS_H

#include <stdint.h>

typedef int BaseType_t;
typedef void * QueueHandle_t;
typedef void * TaskHandle_t;

#define pdTRUE        ((BaseType_t)1)
#define pdFALSE       ((BaseType_t)0)
#define portMAX_DELAY ((uint32_t)0xFFFFFFFFu)
#define pdMS_TO_TICKS(ms) ((uint32_t)(ms))

static inline void vTaskDelay(uint32_t ticks)
{
    (void)ticks;
}

#endif
