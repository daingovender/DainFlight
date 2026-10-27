/*******************************************************************************
 * FILE:         fcs_control.h
 * MODULE:       dRehmFlight Flight Control System (FCS)
 * STANDARD:     RTCA DO-178C Level A / MISRA C:2012
 * DESCRIPTION:  Flight control and mixing interface.
 *******************************************************************************/

#ifndef FCS_CONTROL_H
#define FCS_CONTROL_H

#include "fcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initializes the FCS internal state to zero.
 * @param  p_state Pointer to state struct.
 * @return fcs_status_t FCS_OK or error.
 */
fcs_status_t FCS_InitState(fcs_state_t* const p_state);

/**
 * @brief  Calculates stabilized attitude control moments using PID algorithms.
 * @param[in]     p_cmd     Desired setpoint from pilot/navigation.
 * @param[in]     p_telem   Estimated attitude and angular rates from sensors.
 * @param[in]     p_gains   Calibrated PID gains for pitch/roll/yaw.
 * @param[in]     dt_sec    Elapsed loop time in seconds.
 * @param[in,out] p_state   Persistent integrator and derivative state.
 * @param[out]    p_moments Computed normalized moments [-1.0, 1.0].
 * @return fcs_status_t     FCS_OK or error code.
 */
fcs_status_t FCS_UpdateAttitudeControl(
    const fcs_pilot_command_t* const p_cmd,
    const fcs_telemetry_t*     const p_telem,
    const fcs_pid_gains_t*     const p_roll_gains,
    const fcs_pid_gains_t*     const p_pitch_gains,
    const fcs_pid_gains_t*     const p_yaw_gains,
    const float32_t                  dt_sec,
    fcs_state_t*               const p_state,
    fcs_moments_t*             const p_moments
);

/**
 * @brief  Mixes throttle and stabilized moments into individual actuator commands.
 * @param[in]  throttle   Normalized throttle [0.0, 1.0].
 * @param[in]  p_moments  Stabilized roll, pitch, yaw moments.
 * @param[out] p_motors   Resulting motor commands clamped to [0.0, 1.0].
 * @return fcs_status_t   FCS_OK or error code.
 */
fcs_status_t FCS_MixActuators(
    const float32_t            throttle,
    const fcs_moments_t* const p_moments,
    fcs_actuators_t*     const p_motors
);

#ifdef __cplusplus
}
#endif

#endif /* FCS_CONTROL_H */
