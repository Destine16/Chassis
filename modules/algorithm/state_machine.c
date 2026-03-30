#include "state_machine.h"

void state_machine_init(state_machine_t *sm, uint32_t initial_state, uint32_t now_ms)
{
    sm->current_state = initial_state;
    sm->previous_state = initial_state;
    sm->entered_at_ms = now_ms;
}

bool state_machine_transition(state_machine_t *sm, uint32_t next_state, uint32_t now_ms)
{
    /* 相同状态不重复记切换时间。 */
    if (sm->current_state == next_state)
        return false;

    sm->previous_state = sm->current_state;
    sm->current_state = next_state;
    sm->entered_at_ms = now_ms;
    return true;
}
