#ifndef USER_CONTROL_STATE_MACHINE_H
#define USER_CONTROL_STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

/* 轻量级状态机记录器，只负责保存前态、现态和进入时刻。 */
typedef struct
{
    uint32_t current_state;
    uint32_t previous_state;
    uint32_t entered_at_ms;
} state_machine_t;

void state_machine_init(state_machine_t *sm, uint32_t initial_state, uint32_t now_ms);

/* 状态未变化返回 false，发生切换时更新 previous/current/entered_at_ms。 */
bool state_machine_transition(state_machine_t *sm, uint32_t next_state, uint32_t now_ms);

#endif /* USER_CONTROL_STATE_MACHINE_H */
