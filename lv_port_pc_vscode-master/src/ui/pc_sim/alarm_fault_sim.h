#ifndef PC_SIM_ALARM_FAULT_SIM_H
#define PC_SIM_ALARM_FAULT_SIM_H

#include <stdint.h>

/* PC 专用：由 ui-商用洗.c 在 USE_RTOS_FREERTOS=0 时实现；真机不链接本模块 */
void ui_pc_sim_fault_clear_all(void);
void ui_pc_sim_fault_show_one(uint8_t fault_idx); /* 0=E1 .. 14=E15，仅保留该项 */

/* 主循环每帧调用：SDL 键盘浏览 E1–E14 */
void pc_sim_alarm_fault_poll_keys(void);

/* 启动时打印键位说明（stderr） */
void pc_sim_alarm_fault_print_help(void);

#endif
