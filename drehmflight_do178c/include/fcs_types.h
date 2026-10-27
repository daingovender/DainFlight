/*******************************************************************************
 * FILE:         fcs_types.h
 * MODULE:       dRehmFlight Flight Control System (FCS)
 * STANDARD:     RTCA DO-178C Level A / MISRA C:2012
 * DESCRIPTION:  Explicit fundamental data types and data structures.
 *******************************************************************************/

#ifndef FCS_TYPES_H
#define FCS_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* MISRA C:2012 Rule 4.6: Explicitly specified bit-width types */
typedef float       float32_t;
typedef double      float64_t;
typedef uint8_t     boolean_t;

#define FCS_TRUE    ((boolean_t)1U)
#define FCS_FALSE   ((boolean_t)0U)

/* FCS Status and Error Codes */
typedef enum {
    FCS_OK                     = 0,
    FCS_ERR_NULL_POINTER       = 1,
    FCS_ERR_INVALID_DT         = 2,
    FCS_ERR_SENSOR_OUT_OF_BOUNDS = 3,
    FCS_ERR_LOSS_OF_SIGNAL     = 4,
    FCS_ERR_UNARMED            = 5
} fcs_status_t;

/* Flight PID Configuration Parameters */
typedef struct {
    float32_t kp;
    float32_t ki;
    float32_t kd;
    float32_t i_limit;
} fcs_pid_gains_t;

/* Controller State per Axis (Preserves continuity without global variables) */
typedef struct {
    float32_t integral;
    float32_t prev_error;
} fcs_axis_state_t;

/* Complete FCS State Representation */
typedef struct {
    fcs_axis_state_t roll;
    fcs_axis_state_t pitch;
    fcs_axis_state_t yaw;
} fcs_state_t;

/* Pilot Command Setpoint */
typedef struct {
    float32_t throttle;  /* Normalized [0.0, 1.0] */
    float32_t roll_deg;  /* Desired Roll Angle [-maxRoll, +maxRoll] */
    float32_t pitch_deg; /* Desired Pitch Angle [-maxPitch, +maxPitch] */
    float32_t yaw_rate;  /* Desired Yaw Rate (deg/s) */
    boolean_t is_valid;  /* Signal validity flag */
} fcs_pilot_command_t;

/* Sensor Telemetry Measurement */
typedef struct {
    float32_t roll_deg;  /* Estimated Roll Angle (deg) */
    float32_t pitch_deg; /* Estimated Pitch Angle (deg) */
    float32_t gyro_x;    /* Angular Rate Roll (deg/s) */
    float32_t gyro_y;    /* Angular Rate Pitch (deg/s) */
    float32_t gyro_z;    /* Angular Rate Yaw (deg/s) */
    boolean_t is_valid;  /* Telemetry validity flag */
} fcs_telemetry_t;

/* Control Moments Output */
typedef struct {
    float32_t roll_moment;  /* Normalized [-1.0, 1.0] */
    float32_t pitch_moment; /* Normalized [-1.0, 1.0] */
    float32_t yaw_moment;   /* Normalized [-1.0, 1.0] */
} fcs_moments_t;

/* 4-Motor Actuator Outputs */
typedef struct {
    float32_t m1; /* Front Right [0.0, 1.0] */
    float32_t m2; /* Rear Right  [0.0, 1.0] */
    float32_t m3; /* Rear Left   [0.0, 1.0] */
    float32_t m4; /* Front Left  [0.0, 1.0] */
} fcs_actuators_t;

#endif /* FCS_TYPES_H */
