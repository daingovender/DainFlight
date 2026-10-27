/*******************************************************************************
 * FILE:         fcs_sensors.h
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Sensor plausibility, validation, and anti-aliasing low-pass filtering.
 *******************************************************************************/

#ifndef FCS_SENSORS_H
#define FCS_SENSORS_H

#include "fcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initializes sensor filter state and sets low-pass coefficients.
 * @param  p_state      Pointer to filter state structure.
 * @param  gyro_alpha   Low-pass filter alpha for gyroscope (0.0, 1.0].
 * @param  accel_alpha  Low-pass filter alpha for accelerometer (0.0, 1.0].
 * @return fcs_status_t FCS_OK or error.
 */
fcs_status_t FCS_Sensors_Init(
    fcs_sensor_filter_state_t* const p_state,
    const float32_t                  gyro_alpha,
    const float32_t                  accel_alpha
);

/**
 * @brief  Validates raw IMU telemetry against physical plausibility bounds
 *         and applies a first-order digital low-pass filter.
 * @param[in]     p_raw       Raw sensor measurement from IMU.
 * @param[in,out] p_state     Persistent filter state.
 * @param[out]    p_filtered  Conditioned and validated sensor telemetry.
 * @return fcs_status_t       FCS_OK or error code.
 */
fcs_status_t FCS_Sensors_Process(
    const fcs_imu_raw_t*       const p_raw,
    fcs_sensor_filter_state_t* const p_state,
    fcs_imu_filtered_t*        const p_filtered
);

#ifdef __cplusplus
}
#endif

#endif /* FCS_SENSORS_H */
