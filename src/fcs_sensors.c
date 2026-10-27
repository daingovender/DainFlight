/*******************************************************************************
 * FILE:         fcs_sensors.c
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Sensor signal conditioning and plausibility checks.
 *******************************************************************************/

#include "fcs_sensors.h"
#include <stddef.h>
#include <math.h>

/* Plausibility limits defined in [LLR-SEN-002] and [LLR-SEN-003] */
static const float32_t GYRO_MAX_LIMIT_DPS  = 2000.0f;
static const float32_t GYRO_MIN_LIMIT_DPS  = -2000.0f;
static const float32_t ACCEL_MAX_LIMIT_G   = 16.0f;
static const float32_t ACCEL_MIN_LIMIT_G   = -16.0f;
static const float32_t ALPHA_MIN           = 0.0001f;
static const float32_t ALPHA_MAX           = 1.0f;

/* Local helper: Checks if floating point value is finite and defined */
static boolean_t IsFloatFinite(const float32_t val)
{
    boolean_t is_finite = FCS_FALSE;
    if (isfinite(val) != 0)
    {
        is_finite = FCS_TRUE;
    }
    return is_finite;
}

fcs_status_t FCS_Sensors_Init(
    fcs_sensor_filter_state_t* const p_state,
    const float32_t                  gyro_alpha,
    const float32_t                  accel_alpha
)
{
    fcs_status_t status = FCS_OK;

    if (p_state == NULL)
    {
        status = FCS_ERR_NULL_POINTER;
    }
    else if ((gyro_alpha < ALPHA_MIN) || (gyro_alpha > ALPHA_MAX) ||
             (accel_alpha < ALPHA_MIN) || (accel_alpha > ALPHA_MAX))
    {
        status = FCS_ERR_SENSOR_OUT_OF_BOUNDS;
    }
    else
    {
        p_state->gyro_alpha  = gyro_alpha;
        p_state->accel_alpha = accel_alpha;
        p_state->prev_filtered.gyro_x_dps = 0.0f;
        p_state->prev_filtered.gyro_y_dps = 0.0f;
        p_state->prev_filtered.gyro_z_dps = 0.0f;
        p_state->prev_filtered.accel_x_g  = 0.0f;
        p_state->prev_filtered.accel_y_g  = 0.0f;
        p_state->prev_filtered.accel_z_g  = 1.0f; /* 1g nominal upright gravity */
        p_state->prev_filtered.is_valid   = FCS_FALSE;
        p_state->is_initialized           = FCS_TRUE;
    }

    return status;
}

fcs_status_t FCS_Sensors_Process(
    const fcs_imu_raw_t*       const p_raw,
    fcs_sensor_filter_state_t* const p_state,
    fcs_imu_filtered_t*        const p_filtered
)
{
    fcs_status_t status = FCS_OK;

    if ((p_raw == NULL) || (p_state == NULL) || (p_filtered == NULL))
    {
        status = FCS_ERR_NULL_POINTER;
    }
    else if (p_state->is_initialized == FCS_FALSE)
    {
        status = FCS_ERR_ARMING_DENIED;
    }
    /* [LLR-SEN-001] Floating Point Sanity Check */
    else if ((IsFloatFinite(p_raw->gyro_x_dps) == FCS_FALSE) ||
             (IsFloatFinite(p_raw->gyro_y_dps) == FCS_FALSE) ||
             (IsFloatFinite(p_raw->gyro_z_dps) == FCS_FALSE) ||
             (IsFloatFinite(p_raw->accel_x_g)  == FCS_FALSE) ||
             (IsFloatFinite(p_raw->accel_y_g)  == FCS_FALSE) ||
             (IsFloatFinite(p_raw->accel_z_g)  == FCS_FALSE))
    {
        status = FCS_ERR_SENSOR_NAN;
        p_filtered->is_valid = FCS_FALSE;
    }
    /* Hardware bus health flag */
    else if (p_raw->is_healthy == FCS_FALSE)
    {
        status = FCS_ERR_LOSS_OF_SIGNAL;
        p_filtered->is_valid = FCS_FALSE;
    }
    /* [LLR-SEN-002] Gyroscope Plausibility Bounds */
    else if ((p_raw->gyro_x_dps < GYRO_MIN_LIMIT_DPS) || (p_raw->gyro_x_dps > GYRO_MAX_LIMIT_DPS) ||
             (p_raw->gyro_y_dps < GYRO_MIN_LIMIT_DPS) || (p_raw->gyro_y_dps > GYRO_MAX_LIMIT_DPS) ||
             (p_raw->gyro_z_dps < GYRO_MIN_LIMIT_DPS) || (p_raw->gyro_z_dps > GYRO_MAX_LIMIT_DPS))
    {
        status = FCS_ERR_SENSOR_OUT_OF_BOUNDS;
        p_filtered->is_valid = FCS_FALSE;
    }
    /* [LLR-SEN-003] Accelerometer Plausibility Bounds */
    else if ((p_raw->accel_x_g < ACCEL_MIN_LIMIT_G) || (p_raw->accel_x_g > ACCEL_MAX_LIMIT_G) ||
             (p_raw->accel_y_g < ACCEL_MIN_LIMIT_G) || (p_raw->accel_y_g > ACCEL_MAX_LIMIT_G) ||
             (p_raw->accel_z_g < ACCEL_MIN_LIMIT_G) || (p_raw->accel_z_g > ACCEL_MAX_LIMIT_G))
    {
        status = FCS_ERR_SENSOR_OUT_OF_BOUNDS;
        p_filtered->is_valid = FCS_FALSE;
    }
    else
    {
        /* [LLR-SEN-004] First-Order Digital Low-Pass Filter */
        const float32_t ga = p_state->gyro_alpha;
        const float32_t aa = p_state->accel_alpha;

        p_filtered->gyro_x_dps = ((1.0f - ga) * p_state->prev_filtered.gyro_x_dps) + (ga * p_raw->gyro_x_dps);
        p_filtered->gyro_y_dps = ((1.0f - ga) * p_state->prev_filtered.gyro_y_dps) + (ga * p_raw->gyro_y_dps);
        p_filtered->gyro_z_dps = ((1.0f - ga) * p_state->prev_filtered.gyro_z_dps) + (ga * p_raw->gyro_z_dps);

        p_filtered->accel_x_g  = ((1.0f - aa) * p_state->prev_filtered.accel_x_g)  + (aa * p_raw->accel_x_g);
        p_filtered->accel_y_g  = ((1.0f - aa) * p_state->prev_filtered.accel_y_g)  + (aa * p_raw->accel_y_g);
        p_filtered->accel_z_g  = ((1.0f - aa) * p_state->prev_filtered.accel_z_g)  + (aa * p_raw->accel_z_g);

        p_filtered->is_valid = FCS_TRUE;

        /* Save filter history */
        p_state->prev_filtered = *p_filtered;
        status = FCS_OK;
    }

    return status;
}
