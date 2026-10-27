#include "test_framework.h"
#include "fcs_core.h"

void Run_Core_Tests(void)
{
    TEST_SUITE_BEGIN("Executive Flight State Machine ([LLR-FSM-001]..[003])");

    fcs_system_state_t sys;
    fcs_status_t status = FCS_Core_Init(&sys, 0.04f, 0.2f, 0.2f);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Core_Init should succeed");
    TEST_ASSERT_EQUAL_INT(FCS_STATE_DISARMED, sys.fsm_state, "Initial state must be DISARMED");

    fcs_pid_config_t roll_cfg  = {2.0f, 0.5f, 0.1f, 15.0f};
    fcs_pid_config_t pitch_cfg = {2.0f, 0.5f, 0.1f, 15.0f};
    fcs_pid_config_t yaw_cfg   = {2.5f, 0.2f, 0.05f, 10.0f};

    fcs_imu_raw_t raw_level = {0};
    raw_level.accel_z_g = 1.0f;
    raw_level.is_healthy = FCS_TRUE;

    fcs_pilot_cmd_t cmd = {0};
    cmd.throttle = 0.0f;
    cmd.arm_switch = FCS_FALSE;
    cmd.signal_valid = FCS_TRUE;

    fcs_actuators_t motors;

    /* Prime filter with a few level steps */
    for (int i = 0; i < 20; i++)
    {
        (void)FCS_Core_Step(&sys, &raw_level, &cmd, &roll_cfg, &pitch_cfg, &yaw_cfg, 0.004f, &motors);
    }
    TEST_ASSERT_EQUAL_INT(FCS_STATE_DISARMED, sys.fsm_state, "Remains DISARMED without arm switch");
    TEST_ASSERT_FLOAT_NEAR(0.0f, motors.m1_front_right, 1e-5f, "Motors zeroed in DISARMED state");

    /* Test 1: [LLR-FSM-001] Interlock rejects arming if throttle is high */
    cmd.arm_switch = FCS_TRUE;
    cmd.throttle   = 0.5f; /* Dangerous! High throttle */
    status = FCS_Core_Step(&sys, &raw_level, &cmd, &roll_cfg, &pitch_cfg, &yaw_cfg, 0.004f, &motors);
    TEST_ASSERT_EQUAL_INT(FCS_ERR_ARMING_DENIED, status, "[LLR-FSM-001] Arming denied on high throttle");
    TEST_ASSERT_EQUAL_INT(FCS_STATE_DISARMED, sys.fsm_state, "Remains DISARMED on interlock trip");

    /* Test 2: [LLR-FSM-001] Arming succeeds with low throttle & level attitude */
    cmd.throttle = 0.02f; /* Safe idle */
    status = FCS_Core_Step(&sys, &raw_level, &cmd, &roll_cfg, &pitch_cfg, &yaw_cfg, 0.004f, &motors);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Arming succeeds with safe conditions");
    TEST_ASSERT_EQUAL_INT(FCS_STATE_ARMED, sys.fsm_state, "Transitioned to ARMED state");

    /* Test 3: [LLR-FSM-002] Attitude Departure Trip (> 80 deg) triggers FAILSAFE */
    fcs_imu_raw_t raw_tumble = {0};
    raw_tumble.gyro_x_dps = 1500.0f; /* Violent roll */
    raw_tumble.accel_z_g  = 1.0f;
    raw_tumble.is_healthy = FCS_TRUE;
    cmd.throttle = 0.5f;

    /* Integrate violent roll until attitude exceeds 80 deg */
    for (int i = 0; i < 100; i++)
    {
        (void)FCS_Core_Step(&sys, &raw_tumble, &cmd, &roll_cfg, &pitch_cfg, &yaw_cfg, 0.004f, &motors);
        if (sys.fsm_state == FCS_STATE_FAILSAFE) {
            break;
        }
    }
    TEST_ASSERT_EQUAL_INT(FCS_STATE_FAILSAFE, sys.fsm_state, "[LLR-FSM-002] Departed attitude trips to FAILSAFE");
    TEST_ASSERT_FLOAT_NEAR(0.0f, motors.m1_front_right, 1e-5f, "[HLR-FCS-600] All motors zeroed upon fail-safe trip");

    TEST_SUITE_END();
}
