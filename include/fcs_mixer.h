/*******************************************************************************
 * FILE:         fcs_mixer.h
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Actuator matrix mixing and hardware saturation bounds.
 *******************************************************************************/

#ifndef FCS_MIXER_H
#define FCS_MIXER_H

#include "fcs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Mixes normalized throttle and stabilized moments into individual
 *         motor commands, enforcing strict physical limits [0.0, 1.0].
 * @param[in]  throttle   Commanded throttle in [0.0, 1.0].
 * @param[in]  p_moments  Stabilized roll, pitch, and yaw moments [-1.0, 1.0].
 * @param[out] p_motors   Resulting bounded motor commands.
 * @return fcs_status_t   FCS_OK or error.
 */
fcs_status_t FCS_Mixer_Process(
    const float32_t            throttle,
    const fcs_moments_t* const p_moments,
    fcs_actuators_t*     const p_motors
);

#ifdef __cplusplus
}
#endif

#endif /* FCS_MIXER_H */
