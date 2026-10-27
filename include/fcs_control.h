/*******************************************************************************
 * FILE:         fcs_control.h
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Primary multi-axis Fly-By-Wire PID stabilization laws.
 *******************************************************************************/

#ifndef FCS_CONTROL_H
#define FCS_CONTROL_H

#include "fcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initializes PID integrators and error state to zero.
 * @param  p_state Pointer to PID controller state.
 * @return fcs_status_t FCS_OK or error.
 */
fcs_status_t FCS_Control_Init(fcs_pid_state_t* const p_state);

/**
 * @brief  Calculates stabilized corrective moments (Roll, Pitch, Yaw)
 *         based on commanded setpoints and measured attitude/rates.
 * @param[in]     p_cmd          Commanded setpoints from pilot/navigation.
 * @param[in]     p_euler        Estimated vehicle attitude angles.
 * @param[in]     p_imu          Filtered gyroscope rates.
 * @param[in]     p_roll_cfg     PID configuration for roll axis.
 * @param[in]     p_pitch_cfg    PID configuration for pitch axis.
 * @param[in]     p_yaw_cfg      PID configuration for yaw axis.
 * @param[in]     dt_sec         Sampling cycle time in seconds.
 * @param[in,out] p_state        Persistent PID integrators and history.
 * @param[out]    p_moments      Output moments bounded to [-1.0, 1.0].
 * @return fcs_status_t          FCS_OK or error code.
 */
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
);

#ifdef __cplusplus
}
#endif

#endif /* FCS_CONTROL_H */
