/*******************************************************************************
 * FILE:         fcs_control.c
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  PID attitude & rate stabilization algorithm.
 *******************************************************************************/

#include "fcs_control.h"
#include <stddef.h>

static const float32_t DT_MIN_SEC           = 0.0005f;
static const float32_t DT_MAX_SEC           = 0.0500f;
static const float32_t THROTTLE_IDLE_THRESH = 0.0600f;
static const float32_t MOMENT_MAX           =  1.0f;
static const float32_t MOMENT_MIN           = -1.0f;
static const float32_t PID_SCALING          =  0.01f;

static float32_t ClampFloat(const float32_t val, const float32_t min, const float32_t max)
{
    float32_t out = val;
    if (val > max)
    {
        out = max;
    }
    else if (val < min)
    {
        out = min;
    }
    else
    {
        /* In range */
    }
    return out;
}

fcs_status_t FCS_Control_Init(fcs_pid_state_t* const p_state)
{
    fcs_status_t status = FCS_OK;

    if (p_state == NULL)
    {
        status = FCS_ERR_NULL_POINTER;
    }
    else
    {
        p_state->roll.integral    = 0.0f;
        p_state->roll.prev_error  = 0.0f;
        p_state->pitch.integral   = 0.0f;
        p_state->pitch.prev_error = 0.0f;
        p_state->yaw.integral     = 0.0f;
        p_state->yaw.prev_error   = 0.0f;
    }

    return status;
}

fcs_status_t FCS_Control_Update(
    const fcs_pilot_cmd_t*    const p_cmd,
    const fcs_euler_t*        const p_euler,
    const fcs_imu_filtered_t* const p_imu,
    const fcs_pid_config_t*   const p_roll_cfg,
    const fcs_pid_config_t*   const p_pitch_cfg,
    const fcs_pid_config_t*   const p_yaw_cfg,
    const float32_t                 dt_sec,
    fcs_pid_state_t*          const p_state,
    fcs_moments_t*            const p_moments
)
{
    fcs_status_t status = FCS_OK;

    if ((p_cmd == NULL) || (p_euler == NULL) || (p_imu == NULL) ||
        (p_roll_cfg == NULL) || (p_pitch_cfg == NULL) || (p_yaw_cfg == NULL) ||
        (p_state == NULL) || (p_moments == NULL))
    {
        status = FCS_ERR_NULL_POINTER;
    }
    /* [LLR-PID-001] Time Delta Verification */
    else if ((dt_sec < DT_MIN_SEC) || (dt_sec > DT_MAX_SEC))
    {
        status = FCS_ERR_INVALID_DT;
    }
    else if (p_cmd->signal_valid == FCS_FALSE)
    {
        status = FCS_ERR_LOSS_OF_SIGNAL;
        p_moments->roll  = 0.0f;
        p_moments->pitch = 0.0f;
        p_moments->yaw   = 0.0f;
    }
    else if (p_euler->is_valid == FCS_FALSE)
    {
        status = FCS_ERR_ATTITUDE_DEGENERATE;
        p_moments->roll  = 0.0f;
        p_moments->pitch = 0.0f;
        p_moments->yaw   = 0.0f;
    }
    else
    {
        /* -------------------------------------------------------------
         * Roll Channel [LLR-PID-002, LLR-PID-003, LLR-PID-004]
         * ------------------------------------------------------------- */
        const float32_t error_roll = p_cmd->roll_des_deg - p_euler->roll_deg;
        p_state->roll.integral += (error_roll * dt_sec);

        /* [LLR-PID-002] Low-Throttle Anti-Windup Zeroing */
        if (p_cmd->throttle < THROTTLE_IDLE_THRESH)
        {
            p_state->roll.integral = 0.0f;
        }
        /* [LLR-PID-003] Anti-Windup Clamping */
        p_state->roll.integral = ClampFloat(p_state->roll.integral, -p_roll_cfg->i_limit, p_roll_cfg->i_limit);

        /* [LLR-PID-004] Rate-Derived Derivative */
        const float32_t deriv_roll = p_imu->gyro_x_dps;
        const float32_t raw_roll = PID_SCALING * (
            (p_roll_cfg->kp * error_roll) +
            (p_roll_cfg->ki * p_state->roll.integral) -
            (p_roll_cfg->kd * deriv_roll)
        );
        /* [LLR-PID-005] Output Moment Sizing */
        p_moments->roll = ClampFloat(raw_roll, MOMENT_MIN, MOMENT_MAX);

        /* -------------------------------------------------------------
         * Pitch Channel [LLR-PID-002, LLR-PID-003, LLR-PID-004]
         * ------------------------------------------------------------- */
        const float32_t error_pitch = p_cmd->pitch_des_deg - p_euler->pitch_deg;
        p_state->pitch.integral += (error_pitch * dt_sec);

        if (p_cmd->throttle < THROTTLE_IDLE_THRESH)
        {
            p_state->pitch.integral = 0.0f;
        }
        p_state->pitch.integral = ClampFloat(p_state->pitch.integral, -p_pitch_cfg->i_limit, p_pitch_cfg->i_limit);

        const float32_t deriv_pitch = p_imu->gyro_y_dps;
        const float32_t raw_pitch = PID_SCALING * (
            (p_pitch_cfg->kp * error_pitch) +
            (p_pitch_cfg->ki * p_state->pitch.integral) -
            (p_pitch_cfg->kd * deriv_pitch)
        );
        p_moments->pitch = ClampFloat(raw_pitch, MOMENT_MIN, MOMENT_MAX);

        /* -------------------------------------------------------------
         * Yaw Rate Channel [LLR-PID-002, LLR-PID-003]
         * ------------------------------------------------------------- */
        const float32_t error_yaw = p_cmd->yaw_rate_des - p_imu->gyro_z_dps;
        p_state->yaw.integral += (error_yaw * dt_sec);

        if (p_cmd->throttle < THROTTLE_IDLE_THRESH)
        {
            p_state->yaw.integral = 0.0f;
        }
        p_state->yaw.integral = ClampFloat(p_state->yaw.integral, -p_yaw_cfg->i_limit, p_yaw_cfg->i_limit);

        const float32_t deriv_yaw = (error_yaw - p_state->yaw.prev_error) / dt_sec;
        p_state->yaw.prev_error = error_yaw;

        const float32_t raw_yaw = PID_SCALING * (
            (p_yaw_cfg->kp * error_yaw) +
            (p_yaw_cfg->ki * p_state->yaw.integral) +
            (p_yaw_cfg->kd * deriv_yaw)
        );
        p_moments->yaw = ClampFloat(raw_yaw, MOMENT_MIN, MOMENT_MAX);

        status = FCS_OK;
    }

    return status;
}
