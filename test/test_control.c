#include "test_framework.h"
#include "fcs_control.h"

void Run_Control_Tests(void)
{
    TEST_SUITE_BEGIN("PID Flight Stabilization ([LLR-PID-001]..[005])");

    fcs_pid_state_t state;
    fcs_status_t status = FCS_Control_Init(&state);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Control_Init should succeed");

    fcs_pid_config_t roll_cfg  = {2.0f, 0.5f, 0.1f, 15.0f};
    fcs_pid_config_t pitch_cfg = {2.0f, 0.5f, 0.1f, 15.0f};
    fcs_pid_config_t yaw_cfg   = {2.5f, 0.2f, 0.05f, 10.0f};

    fcs_euler_t euler = {0};
    euler.is_valid = FCS_TRUE;
    fcs_imu_filtered_t imu = {0};
    imu.is_valid = FCS_TRUE;

    /* Test 1: [LLR-PID-001] Invalid dt rejection */
    fcs_pilot_cmd_t cmd = {0};
    cmd.signal_valid = FCS_TRUE;
    cmd.throttle = 0.5f;
    fcs_moments_t moments;

    status = FCS_Control_Update(&cmd, &euler, &imu, &roll_cfg, &pitch_cfg, &yaw_cfg, 0.0f, &state, &moments);
    TEST_ASSERT_EQUAL_INT(FCS_ERR_INVALID_DT, status, "[LLR-PID-001] Reject dt=0.0s division by zero");

    status = FCS_Control_Update(&cmd, &euler, &imu, &roll_cfg, &pitch_cfg, &yaw_cfg, 0.080f, &state, &moments);
    TEST_ASSERT_EQUAL_INT(FCS_ERR_INVALID_DT, status, "[LLR-PID-001] Reject dt=80ms exceeding 50ms bound");

    /* Test 2: [LLR-PID-002] Low-throttle integrator zeroing */
    cmd.throttle = 0.5f;
    cmd.roll_des_deg = 20.0f;
    euler.roll_deg = 0.0f; /* 20 deg error */

    /* Run one step at mid throttle */
    status = FCS_Control_Update(&cmd, &euler, &imu, &roll_cfg, &pitch_cfg, &yaw_cfg, 0.004f, &state, &moments);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Control update nominal OK");
    TEST_ASSERT(state.roll.integral > 0.0f, "Integrator accumulated error");

    /* Drop throttle below idle (0.02 < 0.06) */
    cmd.throttle = 0.02f;
    status = FCS_Control_Update(&cmd, &euler, &imu, &roll_cfg, &pitch_cfg, &yaw_cfg, 0.004f, &state, &moments);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Control update nominal at idle");
    TEST_ASSERT_FLOAT_NEAR(0.0f, state.roll.integral, 1e-5f, "[LLR-PID-002] Low throttle forces integral to 0.0");

    /* Test 3: [LLR-PID-003] Integrator anti-windup saturation limit */
    cmd.throttle = 0.8f;
    cmd.roll_des_deg = 40.0f;
    euler.roll_deg = 0.0f;
    for (int i = 0; i < 500; i++)
    {
        (void)FCS_Control_Update(&cmd, &euler, &imu, &roll_cfg, &pitch_cfg, &yaw_cfg, 0.010f, &state, &moments);
    }
    TEST_ASSERT_FLOAT_NEAR(roll_cfg.i_limit, state.roll.integral, 1e-4f, "[LLR-PID-003] Integrator clamped to i_limit (15.0)");

    /* Test 4: [LLR-PID-005] Output moment bounds [-1.0, 1.0] */
    fcs_pid_config_t extreme_cfg = {500.0f, 100.0f, 50.0f, 50.0f};
    cmd.roll_des_deg = 80.0f;
    (void)FCS_Control_Update(&cmd, &euler, &imu, &extreme_cfg, &extreme_cfg, &extreme_cfg, 0.004f, &state, &moments);
    TEST_ASSERT_FLOAT_NEAR(1.0f, moments.roll, 1e-5f, "[LLR-PID-005] Roll moment clamped to +1.0");

    TEST_SUITE_END();
}
