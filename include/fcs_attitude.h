/*******************************************************************************
 * FILE:         fcs_attitude.h
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Quaternion sensor fusion & attitude determination.
 *******************************************************************************/

#ifndef FCS_ATTITUDE_H
#define FCS_ATTITUDE_H

#include "fcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initializes the attitude quaternion to level identity [1, 0, 0, 0]^T.
 * @param  p_state Pointer to attitude estimator state.
 * @param  beta    Filter divergence correction gain.
 * @return fcs_status_t FCS_OK or error.
 */
fcs_status_t FCS_Attitude_Init(
    fcs_attitude_state_t* const p_state,
    const float32_t             beta
);

/**
 * @brief  Updates orientation quaternion and resolves vehicle Euler angles.
 * @param[in]     p_imu    Filtered gyroscope and accelerometer telemetry.
 * @param[in]     dt_sec   Elapsed sampling interval in seconds.
 * @param[in,out] p_state  Persistent quaternion state.
 * @param[out]    p_euler  Calculated Roll, Pitch, Yaw angles in degrees.
 * @return fcs_status_t    FCS_OK or error code.
 */
fcs_status_t FCS_Attitude_Update(
    const fcs_imu_filtered_t* const p_imu,
    const float32_t                 dt_sec,
    fcs_attitude_state_t*     const p_state,
    fcs_euler_t*              const p_euler
);

#ifdef __cplusplus
}
#endif

#endif /* FCS_ATTITUDE_H */
