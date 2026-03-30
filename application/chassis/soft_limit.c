#include "soft_limit.h"

#include "robot_def.h"
#include "common_math.h"

float soft_limit_symmetric(float value, float limit)
{
    return common_clampf(value, -limit, limit);
}

void soft_limit_chassis_cmd(chassis_cmd_t *cmd)
{
    /* 统一在车体速度层面限幅，轮级别电流限幅在 ctrl_chassis 中处理。 */
    cmd->vx_mps = soft_limit_symmetric(cmd->vx_mps, APP_CFG_MAX_VX_MPS);
    cmd->vy_mps = soft_limit_symmetric(cmd->vy_mps, APP_CFG_MAX_VY_MPS);
    cmd->wz_radps = soft_limit_symmetric(cmd->wz_radps, APP_CFG_MAX_WZ_RADPS);
}
