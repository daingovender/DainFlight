/*******************************************************************************
 * FILE:         fcs_control.c
 * MODULE:       dRehmFlight Flight Control System (FCS)
 * STANDARD:     RTCA DO-178C Level A / MISRA C:2012
 * DESCRIPTION:  Safety-critical PID flight attitude stabilizer & mixer.
 *******************************************************************************/

#include "fcs_control.h"
#include <stddef.h>

/* Constants aligned with DO-178C LLR specifications */
static const float32_t DT_MIN_SEC           = 0.0005f; /* 2000 Hz upper bound */
static const float32_t DT_MAX_SEC           = 0.0500f; /* 20 Hz lower bound */
static const float32_t THROTTLE_IDLE_THRESH = 0.0600f; /* ~1060 us radio PWM */
static const float32_t MOMENT_MAX           =  1.0f;
static const float32_t MOMENT_MIN           = -1.0f;
static const float32_t MOTOR_MAX            =  1.0f;
static const float32_t MOTOR_MIN            =  0.0f;
static const float32_t PID_SCALE_FACTOR     =  0.01f;

/* Local helper: Clamp floating point value within [min, max] */
static float32_t ClampFloat(const float32_t val, const float32_t min, const float32_t max)
{
    float32_t result = val;
    if (val > max)
    {
        result = max;
    }
    else if (val < min)
    {
        result = min;
    }
    else
    {
        /* In range: result is val */
    }
    return result;
}

fcs_status_t FCS_InitState(fcs_state_t* const p_state)
{
    fcs_status_t status = FCS_OK;

    if (p_state == NULL)
    {
        status = FCS_ERR_NULL_POINTER;
    }
    else
    {
        p_state->roll.integral   = 0.0f;
        p_state->roll.prev_error = 0.0f;
        p_state->pitch.integral  = 0.0f;
        p_state->pitch.prev_error= 0.0f;
        p_state->yaw.integral    = 0.0f;
        p_state->yaw.prev_error  = 0.0f;
    }

    return status;
}

fcs_status_t FCS_UpdateAttitudeControl(
    const fcs_pilot_command_t* const p_cmd,
    const fcs_telemetry_t*     const p_telem,
    const fcs_pid_gains_t*     const p_roll_gains,
    const fcs_pid_gains_t*     const p_pitch_gains,
    const fcs_pid_gains_t*     const p_yaw_gains,
    const float32_t                  dt_sec,
    fcs_state_t*               const p_state,
    fcs_moments_t*             const p_moments
)
{
    fcs_status_t status = FCS_OK;

    /* 1. Defensive parameter validation */
    if ((p_cmd == NULL) || (p_telem == NULL) || (p_state == NULL) || (p_moments == NULL) ||
        (p_roll_gains == NULL) || (p_pitch_gains == NULL) || (p_yaw_gains == NULL))
    {
        status = FCS_ERR_NULL_POINTER;
    }
    /* [LLR-PID-001] Time Delta Plausibility Verification */
    else if ((dt_sec < DT_MIN_SEC) || (dt_sec > DT_MAX_SEC))
    {
        status = FCS_ERR_INVALID_DT;
    }
    /* [HLR-FCS-200] Sensor & Radio Validity Checks */
    else if (p_cmd->is_valid == FCS_FALSE)
    {
        status = FCS_ERR_LOSS_OF_SIGNAL;
        p_moments->roll_moment  = 0.0f;
        p_moments->pitch_moment = 0.0f;
        p_moments->yaw_moment   = 0.0f;
    }
    else if (p_telem->is_valid == FCS_FALSE)
    {
        status = FCS_ERR_SENSOR_OUT_OF_BOUNDS;
        p_moments->roll_moment  = 0.0f;
        p_moments->pitch_moment = 0.0f;
        p_moments->yaw_moment   = 0.0f;
    }
    else
    {
        /* -------------------------------------------------------------
         * Roll Channel [LLR-PID-002, LLR-PID-003, LLR-PID-004]
         * ------------------------------------------------------------- */
        const float32_t error_roll = p_cmd->roll_deg - p_telem->roll_deg;
        p_state->roll.integral += (error_roll * dt_sec);

        /* [LLR-PID-002] Low-Throttle Integrator Reset */
        if (p_cmd->throttle < THROTTLE_IDLE_THRESH)
        {
            p_state->roll.integral = 0.0f;
        }
        /* [LLR-PID-003] Anti-Windup Clamping */
        p_state->roll.integral = ClampFloat(p_state->roll.integral, -p_roll_gains->i_limit, p_roll_gains->i_limit);

        /* [LLR-PID-004] Derivative from Gyro Rate to avoid kick */
        const float32_t deriv_roll = p_telem->gyro_x;
        const float32_t raw_roll = PID_SCALE_FACTOR * (
            (p_roll_gains->kp * error_roll) +
            (p_roll_gains->ki * p_state->roll.integral) -
            (p_roll_gains->kd * deriv_roll)
        );
        /* [LLR-PID-005] Output Moment Sizing */
        p_moments->roll_moment = ClampFloat(raw_roll, MOMENT_MIN, MOMENT_MAX);

        /* -------------------------------------------------------------
         * Pitch Channel [LLR-PID-002, LLR-PID-003, LLR-PID-004]
         * ------------------------------------------------------------- */
        const float32_t error_pitch = p_cmd->pitch_deg - p_telem->pitch_deg;
        p_state->pitch.integral += (error_pitch * dt_sec);

        if (p_cmd->throttle < THROTTLE_IDLE_THRESH)
        {
            p_state->pitch.integral = 0.0f;
        }
        p_state->pitch.integral = ClampFloat(p_state->pitch.integral, -p_pitch_gains->i_limit, p_pitch_gains->i_limit);

        const float32_t deriv_pitch = p_telem->gyro_y;
        const float32_t raw_pitch = PID_SCALE_FACTOR * (
            (p_pitch_gains->kp * error_pitch) +
            (p_pitch_gains->ki * p_state->pitch.integral) -
            (p_pitch_gains->kd * deriv_pitch)
        );
        p_moments->pitch_moment = ClampFloat(raw_pitch, MOMENT_MIN, MOMENT_MAX);

        /* -------------------------------------------------------------
         * Yaw Channel (Rate Control Mode) [LLR-PID-002, LLR-PID-003]
         * ------------------------------------------------------------- */
        const float32_t error_yaw = p_cmd->yaw_rate - p_telem->gyro_z;
        p_state->yaw.integral += (error_yaw * dt_sec);

        if (p_cmd->throttle < THROTTLE_IDLE_THRESH)
        {
            p_state->yaw.integral = 0.0f;
        }
        p_state->yaw.integral = ClampFloat(p_state->yaw.integral, -p_yaw_gains->i_limit, p_yaw_gains->i_limit);

        const float32_t deriv_yaw = (error_yaw - p_state->yaw.prev_error) / dt_sec;
        p_state->yaw.prev_error = error_yaw;

        const float32_t raw_yaw = PID_SCALE_FACTOR * (
            (p_yaw_gains->kp * error_yaw) +
            (p_yaw_gains->ki * p_state->yaw.integral) +
            (p_yaw_gains->kd * deriv_yaw)
        );
        p_moments->yaw_moment = ClampFloat(raw_yaw, MOMENT_MIN, MOMENT_MAX);

        status = FCS_OK;
    }

    return status;
}

fcs_status_t FCS_MixActuators(
    const float32_t            throttle,
    const fcs_moments_t* const p_moments,
    fcs_actuators_t*     const p_motors
)
{
    fcs_status_t status = FCS_OK;

    if ((p_moments == NULL) || (p_motors == NULL))
    {
        status = FCS_ERR_NULL_POINTER;
    }
    else
    {
        const float32_t t = ClampFloat(throttle, MOTOR_MIN, MOTOR_MAX);
        const float32_t r = p_moments->roll_moment;
        const float32_t p = p_moments->pitch_moment;
        const float32_t y = p_moments->yaw_moment;

        /* [LLR-MIX-001] Quad-X Mixer Equations */
        const float32_t m1_raw = t - r + p + y;
        const float32_t m2_raw = t - r - p - y;
        const float32_t m3_raw = t + r - p + y;
        const float32_t m4_raw = t + r + p - y;

        /* [LLR-MIX-002] Physical Saturation Constraints */
        p_motors->m1 = ClampFloat(m1_raw, MOTOR_MIN, MOTOR_MAX);
        p_motors->m2 = ClampFloat(m2_raw, MOTOR_MIN, MOTOR_MAX);
        p_motors->m3 = ClampFloat(m3_raw, MOTOR_MIN, MOTOR_MAX);
        p_motors->m4 = ClampFloat(m4_raw, MOTOR_MIN, MOTOR_MAX);

        status = FCS_OK;
    }

    return status;
}
