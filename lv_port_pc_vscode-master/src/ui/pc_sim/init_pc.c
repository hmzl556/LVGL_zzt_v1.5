#include "init.h"

rtos r = {
    .q_beep = NULL,
    .q_fsm  = NULL,
};

rtos * get_rtos(void)
{
    return &r;
}

const int32_t prog_table_comm[5][8] = {{0}};
