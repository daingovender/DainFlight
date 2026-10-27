/*******************************************************************************
 * FILE:         fcs_core.h
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Primary Flight Computer cyclic executive & state machine.
 *******************************************************************************/

#ifndef FCS_CORE_H
#define FCS_CORE_H

#include "fcs_types.h"
#include "fcs_sensors.h"
#include "fcs_attitude.h"
#include "fcs_control.h"
#include "fcs_mixer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initializes the complete avionics suite and resets internal memory.
 * @param  p_sys        Pointer to master system state container.
 * @param  beta         Madgwick filter gain.
 * @param  gyro_alpha   Sensor low-pass filter alpha for gyro.
 * @param  accel_alpha  Sensor low-pass filter alpha for accelerometer.
 * @return fcs_status_t FCS_OK or error.
 */
fcs_status_t FCS_Core_Init(
    fcs_system_state_t* const p_sys,
    const float32_t           beta,
    const float32_t           gyro_alpha,
    const float32_t           accel_alpha
);

/**
 * @brief  Master deterministic cyclic execution function (250 Hz nominal tick).
 *         Executes sensor validation, attitude estimation, PID laws, and mixing.
 * @param[in,out] p_sys        Master avionics state.
 * @param[in]     p_raw_imu    Raw inertial measurement from sensors.
 * @param[in]     p_cmd        Pilot/Autopilot flight setpoint and switches.
 * @param[in]     p_roll_cfg   Roll PID gains.
 * @param[in]     p_pitch_cfg  Pitch PID gains.
 * @param[in]     p_yaw_cfg    Yaw PID gains.
 * @param[in]     dt_sec       Sampling interval in seconds.
 * @param[out]    p_actuators  Commanded output to motor ESCs.
 * @return fcs_status_t        FCS_OK or error code.
 */
fcs_status_t FCS_Core_Step(
    fcs_system_state_t*     const p_sys,
    const fcs_imu_raw_t*    const p_raw_imu,
    const fcs_pilot_cmd_t*  const p_cmd,
    const fcs_pid_config_t* const p_roll_cfg,
    const fcs_pid_config_t* const p_pitch_cfg,
    const fcs_pid_config_t* const p_yaw_cfg,
    const float32_t               dt_sec,
    fcs_actuators_t*        const p_actuators
);

#ifdef __cplusplus
}
#endif

#endif /* FCS_CORE_H */
