#ifndef PC_SIM_INIT_H
#define PC_SIM_INIT_H

#include "FreeRTOS.h"
#include "queue.h"
#include "agreement.h"

typedef struct {
    QueueHandle_t q_beep;
    QueueHandle_t q_fsm;
} rtos;

extern rtos r;

rtos * get_rtos(void);

/* 支付页 Modbus 参数表（真机由下位机提供；PC 仿真用占位零表） */
extern const int32_t prog_table_comm[5][8];

#endif
