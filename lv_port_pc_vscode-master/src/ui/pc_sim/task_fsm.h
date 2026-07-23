#ifndef TASK_FSM_H
#define TASK_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"

typedef enum {
    FSM_OFF = 0,
    FSM_STANDBY,
    FSM_RUNNING,
    FSM_PAUSED,
} FsmState;

typedef struct {
    FsmState state;
    uint32_t flags;
} ControlFsm;

extern ControlFsm fsm;

void fsm_state_change(FsmState new_state);
BaseType_t ui_scr_load_async(void);

#define FSM_FLAG_NEED_PAYMENT      (1u << 0)
#define FSM_FLAG_PAYMENT_SUCCESS   (1u << 1)
#define FSM_FLAG_PAY_DONE_COMPLETE (1u << 2)

#define GETFLAG(f)  (((fsm).flags & (uint32_t)(f)) != 0u)
#define SETFLAG(f)  ((fsm).flags |= (uint32_t)(f))
#define CLRFLAG(f)  ((fsm).flags &= ~(uint32_t)(f))

#endif
