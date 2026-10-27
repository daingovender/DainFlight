/*******************************************************************************
 * FILE:         fcs_types.h
 * SYSTEM:       DainFlight Primary Flight Control System
 * STANDARD:     RTCA DO-178C DAL-A / MISRA C:2012
 * DESCRIPTION:  Fundamental data types, constants, structures, and status codes.
 *******************************************************************************/

#ifndef FCS_TYPES_H
#define FCS_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* MISRA C:2012 Rule 4.6: Explicit bit-width types */
typedef float       float32_t;
typedef double      float64_t;
typedef uint8_t     boolean_t;

#define FCS_TRUE    ((boolean_t)1U)
#define FCS_FALSE   ((boolean_t)0U)

/* FCS Operational Status and Error Codes */
typedef enum {
    FCS_OK                         = 0,
    FCS_ERR_NULL_POINTER           = 1,
    FCS_ERR_INVALID_DT             = 2,
    FCS_ERR_SENSOR_NAN             = 3,
    FCS_ERR_SENSOR_OUT_OF_BOUNDS   = 4,
    FCS_ERR_ATTITUDE_DEGENERATE    = 5,
    FCS_ERR_LOSS_OF_SIGNAL         = 6,
    FCS_ERR_ARMING_DENIED          = 7,
    FCS_ERR_ATTITUDE_DEPARTURE     = 8
} fcs_status_t;

/* Primary Flight Executive State Machine States */
typedef enum {
    FCS_STATE_INIT                 = 0,
    FCS_STATE_DISARMED             = 1,
    FCS_STATE_ARMED                = 2,
    FCS_STATE_FAILSAFE             = 3
} fcs_fsm_state_t;

/* Raw Sensor Telemetry from Inertial Measurement Unit */
typedef struct {
    float32_t gyro_x_dps;  /* Raw Roll Rate (deg/s) */
    float32_t gyro_y_dps;  /* Raw Pitch Rate (deg/s) */
    float32_t gyro_z_dps;  /* Raw Yaw Rate (deg/s) */
    float32_t accel_x_g;   /* Raw X Acceleration (g) */
    float32_t accel_y_g;   /* Raw Y Acceleration (g) */
    float32_t accel_z_g;   /* Raw Z Acceleration (g) */
    boolean_t is_healthy;  /* Hardware communication flag */
} fcs_imu_raw_t;

/* Conditioned / Low-Pass Filtered IMU Telemetry */
typedef struct {
    float32_t gyro_x_dps;
    float32_t gyro_y_dps;
    float32_t gyro_z_dps;
    float32_t accel_x_g;
    float32_t accel_y_g;
    float32_t accel_z_g;
    boolean_t is_valid;
} fcs_imu_filtered_t;

/* Filter State Retention Structure */
typedef struct {
    fcs_imu_filtered_t prev_filtered;
    float32_t          gyro_alpha;   /* Filter coefficient in (0.0, 1.0] */
    float32_t          accel_alpha;  /* Filter coefficient in (0.0, 1.0] */
    boolean_t          is_initialized;
} fcs_sensor_filter_state_t;

/* Normalized Attitude Quaternion (NWU Frame) */
typedef struct {
    float32_t q0; /* Scalar */
    float32_t q1; /* Vector X */
    float32_t q2; /* Vector Y */
    float32_t q3; /* Vector Z */
} fcs_quaternion_t;

/* Estimated Attitude Euler Angles (Degrees) */
typedef struct {
    float32_t roll_deg;
    float32_t pitch_deg;
    float32_t yaw_deg;
    boolean_t is_valid;
} fcs_euler_t;

/* Attitude Estimator Internal State */
typedef struct {
    fcs_quaternion_t q;
    float32_t        beta_madgwick; /* Filter algorithm gain */
    boolean_t        is_initialized;
} fcs_attitude_state_t;

/* Pilot / Autopilot Guidance Command */
typedef struct {
    float32_t throttle;      /* Demand in [0.0, 1.0] */
    float32_t roll_des_deg;  /* Roll setpoint in degrees */
    float32_t pitch_des_deg; /* Pitch setpoint in degrees */
    float32_t yaw_rate_des;  /* Yaw rate setpoint in deg/s */
    boolean_t arm_switch;    /* Arm switch engaged */
    boolean_t signal_valid;  /* Radio link health */
} fcs_pilot_cmd_t;

/* Single Axis PID Gains and Saturation Configuration */
typedef struct {
    float32_t kp;
    float32_t ki;
    float32_t kd;
    float32_t i_limit;
} fcs_pid_config_t;

/* Single Axis PID Integrator & Memory State */
typedef struct {
    float32_t integral;
    float32_t prev_error;
} fcs_axis_pid_state_t;

/* 3-Axis PID Controller State */
typedef struct {
    fcs_axis_pid_state_t roll;
    fcs_axis_pid_state_t pitch;
    fcs_axis_pid_state_t yaw;
} fcs_pid_state_t;

/* Stabilized Corrective Moments [-1.0, 1.0] */
typedef struct {
    float32_t roll;
    float32_t pitch;
    float32_t yaw;
} fcs_moments_t;

/* Actuator Propulsion Commands [0.0, 1.0] (Quad-X Geometry) */
typedef struct {
    float32_t m1_front_right;
    float32_t m2_rear_right;
    float32_t m3_rear_left;
    float32_t m4_front_left;
} fcs_actuators_t;

/* Complete Avionics System State Container */
typedef struct {
    fcs_fsm_state_t           fsm_state;
    fcs_sensor_filter_state_t sensor_state;
    fcs_attitude_state_t      attitude_state;
    fcs_pid_state_t           pid_state;
    uint32_t                  cycle_counter;
    fcs_status_t              last_status;
} fcs_system_state_t;

#endif /* FCS_TYPES_H */
