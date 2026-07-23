#ifndef UI_H
#define UI_H

#include "lvgl.h"
#include <stdlib.h>
#include <stdio.h>

//extern uint8_t lv_coil[N_COILS];
//extern uint8_t lv_discrete_input[N_DISCRETE_INPUTS];
//extern uint16_t lv_holding_register[N_HOLDING_REGISTERS];
//extern uint16_t lv_input_register[N_INPUT_REGISTERS];

void ui_init(void);
void ui_tick(void);
void task_LVGL(void *pvParameters);

#endif
