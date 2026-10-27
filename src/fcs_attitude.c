/*******************************************************************************
 * FILE:         fcs_attitude.c
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Safety-critical orientation estimation & Euler calculation.
 *******************************************************************************/

#include "fcs_attitude.h"
#include <stddef.h>
#include <math.h>

static const float32_t DEG_TO_RAD          = 0.017453292519943295f;
static const float32_t RAD_TO_DEG          = 57.29577951308232f;
static const float32_t ACCEL_NORM_EPS_SQ   = 0.000001f;     /* 0.001g threshold [LLR-ATT-001] */
static const float32_t QUAT_NORM_EPS_SQ    = 0.0000000001f; /* 1e-5 threshold [LLR-ATT-002] */
static const float32_t ASIN_DOMAIN_LIMIT   = 0.999999f;     /* [LLR-ATT-003] */
static const float32_t DT_MIN_SEC          = 0.0005f;
static const float32_t DT_MAX_SEC          = 0.0500f;

static float32_t Clamp(const float32_t val, const float32_t min, const float32_t max)
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

fcs_status_t FCS_Attitude_Init(
    fcs_attitude_state_t* const p_state,
    const float32_t             beta
)
{
    fcs_status_t status = FCS_OK;

    if (p_state == NULL)
    {
        status = FCS_ERR_NULL_POINTER;
    }
    else if ((beta < 0.0001f) || (beta > 10.0f))
    {
        status = FCS_ERR_SENSOR_OUT_OF_BOUNDS;
    }
    else
    {
        /* Identity orientation quaternion */
        p_state->q.q0 = 1.0f;
        p_state->q.q1 = 0.0f;
        p_state->q.q2 = 0.0f;
        p_state->q.q3 = 0.0f;
        p_state->beta_madgwick = beta;
        p_state->is_initialized = FCS_TRUE;
    }

    return status;
}

fcs_status_t FCS_Attitude_Update(
    const fcs_imu_filtered_t* const p_imu,
    const float32_t                 dt_sec,
    fcs_attitude_state_t*     const p_state,
    fcs_euler_t*              const p_euler
)
{
    fcs_status_t status = FCS_OK;

    if ((p_imu == NULL) || (p_state == NULL) || (p_euler == NULL))
    {
        status = FCS_ERR_NULL_POINTER;
    }
    else if (p_state->is_initialized == FCS_FALSE)
    {
        status = FCS_ERR_ARMING_DENIED;
    }
    else if ((dt_sec < DT_MIN_SEC) || (dt_sec > DT_MAX_SEC))
    {
        status = FCS_ERR_INVALID_DT;
    }
    else if (p_imu->is_valid == FCS_FALSE)
    {
        status = FCS_ERR_SENSOR_OUT_OF_BOUNDS;
        p_euler->is_valid = FCS_FALSE;
    }
    else
    {
        /* Working quaternion copies */
        float32_t q0 = p_state->q.q0;
        float32_t q1 = p_state->q.q1;
        float32_t q2 = p_state->q.q2;
        float32_t q3 = p_state->q.q3;

        /* Convert rates to radians/sec (NWU aircraft frame: gy and gz inverted) */
        const float32_t gx =  p_imu->gyro_x_dps * DEG_TO_RAD;
        const float32_t gy = -p_imu->gyro_y_dps * DEG_TO_RAD;
        const float32_t gz = -p_imu->gyro_z_dps * DEG_TO_RAD;

        /* Rate of change of quaternion from gyroscope */
        float32_t qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float32_t qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
        float32_t qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
        float32_t qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

        /* Accelerometer readings in NWU frame */
        float32_t ax = -p_imu->accel_x_g;
        float32_t ay =  p_imu->accel_y_g;
        float32_t az =  p_imu->accel_z_g;

        const float32_t a_norm_sq = (ax * ax) + (ay * ay) + (az * az);

        /* [LLR-ATT-001] Accelerometer Zero-Norm Protection */
        if (a_norm_sq > ACCEL_NORM_EPS_SQ)
        {
            const float32_t inv_a = 1.0f / sqrtf(a_norm_sq);
            ax *= inv_a;
            ay *= inv_a;
            az *= inv_a;

            /* Auxiliary variables */
            const float32_t _2q0 = 2.0f * q0;
            const float32_t _2q1 = 2.0f * q1;
            const float32_t _2q2 = 2.0f * q2;
            const float32_t _2q3 = 2.0f * q3;
            const float32_t _4q0 = 4.0f * q0;
            const float32_t _4q1 = 4.0f * q1;
            const float32_t _4q2 = 4.0f * q2;
            const float32_t _8q1 = 8.0f * q1;
            const float32_t _8q2 = 8.0f * q2;
            const float32_t q0q0 = q0 * q0;
            const float32_t q1q1 = q1 * q1;
            const float32_t q2q2 = q2 * q2;
            const float32_t q3q3 = q3 * q3;

            /* Gradient descent corrective step */
            float32_t s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            float32_t s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            float32_t s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            float32_t s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

            const float32_t s_norm_sq = (s0 * s0) + (s1 * s1) + (s2 * s2) + (s3 * s3);
            if (s_norm_sq > 0.0000001f)
            {
                const float32_t inv_s = 1.0f / sqrtf(s_norm_sq);
                s0 *= inv_s;
                s1 *= inv_s;
                s2 *= inv_s;
                s3 *= inv_s;

                /* Apply feedback step */
                qDot1 -= (p_state->beta_madgwick * s0);
                qDot2 -= (p_state->beta_madgwick * s1);
                qDot3 -= (p_state->beta_madgwick * s2);
                qDot4 -= (p_state->beta_madgwick * s3);
            }
        }

        /* Integrate quaternion derivative */
        q0 += (qDot1 * dt_sec);
        q1 += (qDot2 * dt_sec);
        q2 += (qDot3 * dt_sec);
        q3 += (qDot4 * dt_sec);

        /* [LLR-ATT-002] Quaternion Normalization Protection */
        const float32_t q_norm_sq = (q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3);
        if (q_norm_sq < QUAT_NORM_EPS_SQ)
        {
            /* Reset degenerate quaternion to identity */
            q0 = 1.0f;
            q1 = 0.0f;
            q2 = 0.0f;
            q3 = 0.0f;
        }
        else
        {
            const float32_t inv_q = 1.0f / sqrtf(q_norm_sq);
            q0 *= inv_q;
            q1 *= inv_q;
            q2 *= inv_q;
            q3 *= inv_q;
        }

        p_state->q.q0 = q0;
        p_state->q.q1 = q1;
        p_state->q.q2 = q2;
        p_state->q.q3 = q3;

        /* Convert to Euler angles */
        p_euler->roll_deg = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * RAD_TO_DEG;

        /* [LLR-ATT-003] Trigonometric Domain Clamping */
        const float32_t sin_pitch_arg = Clamp(-2.0f * (q1 * q3 - q0 * q2), -ASIN_DOMAIN_LIMIT, ASIN_DOMAIN_LIMIT);
        p_euler->pitch_deg = -asinf(sin_pitch_arg) * RAD_TO_DEG;

        p_euler->yaw_deg = -atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * RAD_TO_DEG;
        p_euler->is_valid = FCS_TRUE;

        status = FCS_OK;
    }

    return status;
}
