/*******************************************************************************
 * FILE:         fcs_core.c
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Executive cyclic scheduler and flight safety state machine.
 *******************************************************************************/

#include "fcs_core.h"
#include <stddef.h>
#include <math.h>

static const float32_t LEVEL_TOLERANCE_DEG   = 10.0f; /* Max tilt angle to permit arming [LLR-FSM-001] */
static const float32_t DEPARTURE_LIMIT_DEG   = 80.0f; /* Flight envelope trip boundary [LLR-FSM-002] */
static const float32_t THROTTLE_ARM_LIMIT    = 0.06f; /* Idle throttle for arming */

/* Helper: zeroes all motor outputs */
static void ZeroMotors(fcs_actuators_t* const p_actuators)
{
    if (p_actuators != NULL)
    {
        p_actuators->m1_front_right = 0.0f;
        p_actuators->m2_rear_right  = 0.0f;
        p_actuators->m3_rear_left   = 0.0f;
        p_actuators->m4_front_left  = 0.0f;
    }
}

fcs_status_t FCS_Core_Init(
    fcs_system_state_t* const p_sys,
    const float32_t           beta,
    const float32_t           gyro_alpha,
    const float32_t           accel_alpha
)
{
    fcs_status_t status = FCS_OK;

    if (p_sys == NULL)
    {
        status = FCS_ERR_NULL_POINTER;
    }
    else
    {
        p_sys->fsm_state     = FCS_STATE_INIT;
        p_sys->cycle_counter = 0U;
        p_sys->last_status   = FCS_OK;

        const fcs_status_t s_stat = FCS_Sensors_Init(&p_sys->sensor_state, gyro_alpha, accel_alpha);
        const fcs_status_t a_stat = FCS_Attitude_Init(&p_sys->attitude_state, beta);
        const fcs_status_t c_stat = FCS_Control_Init(&p_sys->pid_state);

        if ((s_stat != FCS_OK) || (a_stat != FCS_OK) || (c_stat != FCS_OK))
        {
            status = FCS_ERR_SENSOR_OUT_OF_BOUNDS;
            p_sys->fsm_state = FCS_STATE_FAILSAFE;
        }
        else
        {
            p_sys->fsm_state = FCS_STATE_DISARMED;
            status = FCS_OK;
        }

        p_sys->last_status = status;
    }

    return status;
}

/* [LLR-FSM-003] Deterministic Step Function */
fcs_status_t FCS_Core_Step(
    fcs_system_state_t*     const p_sys,
    const fcs_imu_raw_t*    const p_raw_imu,
    const fcs_pilot_cmd_t*  const p_cmd,
    const fcs_pid_config_t* const p_roll_cfg,
    const fcs_pid_config_t* const p_pitch_cfg,
    const fcs_pid_config_t* const p_yaw_cfg,
    const float32_t               dt_sec,
    fcs_actuators_t*        const p_actuators
)
{
    fcs_status_t status = FCS_OK;

    if ((p_sys == NULL) || (p_raw_imu == NULL) || (p_cmd == NULL) ||
        (p_roll_cfg == NULL) || (p_pitch_cfg == NULL) || (p_yaw_cfg == NULL) ||
        (p_actuators == NULL))
    {
        status = FCS_ERR_NULL_POINTER;
    }
    else
    {
        p_sys->cycle_counter++;
        ZeroMotors(p_actuators);

        /* 1. Condition & Validate Sensor Data [HLR-FCS-200] */
        fcs_imu_filtered_t filtered_imu;
        filtered_imu.is_valid = FCS_FALSE;
        const fcs_status_t sens_stat = FCS_Sensors_Process(p_raw_imu, &p_sys->sensor_state, &filtered_imu);

        if (sens_stat != FCS_OK)
        {
            status = sens_stat;
            p_sys->fsm_state = FCS_STATE_FAILSAFE;
        }
        else
        {
            /* 2. Resolve Vehicle Attitude [HLR-FCS-300] */
            fcs_euler_t euler;
            euler.is_valid = FCS_FALSE;
            const fcs_status_t att_stat = FCS_Attitude_Update(&filtered_imu, dt_sec, &p_sys->attitude_state, &euler);

            if (att_stat != FCS_OK)
            {
                status = att_stat;
                p_sys->fsm_state = FCS_STATE_FAILSAFE;
            }
            else
            {
                /* 3. Evaluate State Machine & Transitions */
                switch (p_sys->fsm_state)
                {
                    case FCS_STATE_DISARMED:
                    {
                        /* [LLR-FSM-001] Arming Interlock Checks */
                        if (p_cmd->arm_switch == FCS_TRUE)
                        {
                            const boolean_t throttle_low = (p_cmd->throttle < THROTTLE_ARM_LIMIT) ? FCS_TRUE : FCS_FALSE;
                            const boolean_t roll_level   = (fabsf(euler.roll_deg)  < LEVEL_TOLERANCE_DEG) ? FCS_TRUE : FCS_FALSE;
                            const boolean_t pitch_level  = (fabsf(euler.pitch_deg) < LEVEL_TOLERANCE_DEG) ? FCS_TRUE : FCS_FALSE;
                            const boolean_t signal_ok    = p_cmd->signal_valid;

                            if ((throttle_low == FCS_TRUE) && (roll_level == FCS_TRUE) &&
                                (pitch_level == FCS_TRUE)  && (signal_ok == FCS_TRUE))
                            {
                                /* Interlocks satisfied: Reset controllers & Transition to ARMED */
                                (void)FCS_Control_Init(&p_sys->pid_state);
                                p_sys->fsm_state = FCS_STATE_ARMED;
                            }
                            else
                            {
                                status = FCS_ERR_ARMING_DENIED;
                            }
                        }
                        ZeroMotors(p_actuators);
                        break;
                    }

                    case FCS_STATE_ARMED:
                    {
                        /* Pilot disarmed via switch */
                        if (p_cmd->arm_switch == FCS_FALSE)
                        {
                            p_sys->fsm_state = FCS_STATE_DISARMED;
                            ZeroMotors(p_actuators);
                        }
                        /* Radio Loss of Signal */
                        else if (p_cmd->signal_valid == FCS_FALSE)
                        {
                            status = FCS_ERR_LOSS_OF_SIGNAL;
                            p_sys->fsm_state = FCS_STATE_FAILSAFE;
                            ZeroMotors(p_actuators);
                        }
                        /* [LLR-FSM-002] Flight Envelope Departure Protection */
                        else if ((fabsf(euler.roll_deg) > DEPARTURE_LIMIT_DEG) ||
                                 (fabsf(euler.pitch_deg) > DEPARTURE_LIMIT_DEG))
                        {
                            status = FCS_ERR_ATTITUDE_DEPARTURE;
                            p_sys->fsm_state = FCS_STATE_FAILSAFE;
                            ZeroMotors(p_actuators);
                        }
                        else
                        {
                            /* 4. Normal Flight: Compute PID Moments [HLR-FCS-400] */
                            fcs_moments_t moments;
                            const fcs_status_t ctrl_stat = FCS_Control_Update(
                                p_cmd, &euler, &filtered_imu,
                                p_roll_cfg, p_pitch_cfg, p_yaw_cfg,
                                dt_sec, &p_sys->pid_state, &moments
                            );

                            if (ctrl_stat != FCS_OK)
                            {
                                status = ctrl_stat;
                                p_sys->fsm_state = FCS_STATE_FAILSAFE;
                                ZeroMotors(p_actuators);
                            }
                            else
                            {
                                /* 5. Mix to Propulsion Actuators [HLR-FCS-500] */
                                const fcs_status_t mix_stat = FCS_Mixer_Process(p_cmd->throttle, &moments, p_actuators);
                                if (mix_stat != FCS_OK)
                                {
                                    status = mix_stat;
                                    p_sys->fsm_state = FCS_STATE_FAILSAFE;
                                    ZeroMotors(p_actuators);
                                }
                                else
                                {
                                    status = FCS_OK;
                                }
                            }
                        }
                        break;
                    }

                    case FCS_STATE_FAILSAFE:
                    default:
                    {
                        /* [HLR-FCS-600] Latch fail-safe: All actuators neutral/off */
                        ZeroMotors(p_actuators);
                        if (p_cmd->arm_switch == FCS_FALSE)
                        {
                            /* Allow recovery to DISARMED if pilot manually toggles arm switch off */
                            p_sys->fsm_state = FCS_STATE_DISARMED;
                        }
                        break;
                    }
                }
            }
        }

        p_sys->last_status = status;
    }

    return status;
}
