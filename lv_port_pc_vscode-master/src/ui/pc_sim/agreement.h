#ifndef PC_SIM_AGREEMENT_H
#define PC_SIM_AGREEMENT_H

#include <stdint.h>

typedef struct {
    uint8_t fsm_req_type;
    uint8_t next_state;
} FsmReq;

enum {
    FSM_EVT_CHILD_LOCK_ON  = 1,
    FSM_EVT_CHILD_LOCK_OFF = 2,
};

typedef struct {
    int32_t seq;
    int32_t repeat;
} BeepReq;

#endif
