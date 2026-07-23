#include "task_fsm.h"

ControlFsm fsm = {
    .state = FSM_OFF,
    .flags = 0u,
};

void fsm_state_change(FsmState new_state)
{
    if(fsm.state == new_state) {
        return;
    }
    fsm.state = new_state;
    ui_scr_load_async();
}
