/*******************************************************************************
 * FILE:         fcs_mixer.c
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Rigid-body propulsion mixer and saturation limiting.
 *******************************************************************************/

#include "fcs_mixer.h"
#include <stddef.h>

static const float32_t MOTOR_CMD_MIN = 0.0f;
static const float32_t MOTOR_CMD_MAX = 1.0f;

static float32_t ClampMotor(const float32_t val)
{
    float32_t out = val;
    if (val > MOTOR_CMD_MAX)
    {
        out = MOTOR_CMD_MAX;
    }
    else if (val < MOTOR_CMD_MIN)
    {
        out = MOTOR_CMD_MIN;
    }
    else
    {
        /* In valid range */
    }
    return out;
}

fcs_status_t FCS_Mixer_Process(
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
        /* [LLR-MIX-002] Clamp input throttle */
        const float32_t t = ClampMotor(throttle);
        const float32_t r = p_moments->roll;
        const float32_t p = p_moments->pitch;
        const float32_t y = p_moments->yaw;

        /* [LLR-MIX-001] Quad-X Mixer Equations */
        const float32_t m1_raw = t - r + p + y;
        const float32_t m2_raw = t - r - p - y;
        const float32_t m3_raw = t + r - p + y;
        const float32_t m4_raw = t + r + p - y;

        /* [LLR-MIX-002] Enforce Hard Saturation Bounds */
        p_motors->m1_front_right = ClampMotor(m1_raw);
        p_motors->m2_rear_right  = ClampMotor(m2_raw);
        p_motors->m3_rear_left   = ClampMotor(m3_raw);
        p_motors->m4_front_left  = ClampMotor(m4_raw);

        status = FCS_OK;
    }

    return status;
}
