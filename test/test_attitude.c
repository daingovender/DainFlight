#include "test_framework.h"
#include "fcs_attitude.h"

void Run_Attitude_Tests(void)
{
    TEST_SUITE_BEGIN("Attitude Estimator ([LLR-ATT-001]..[003])");

    fcs_attitude_state_t state;
    fcs_status_t status = FCS_Attitude_Init(&state, 0.04f);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Attitude_Init should succeed");
    TEST_ASSERT_FLOAT_NEAR(1.0f, state.q.q0, 1e-5f, "Initial q0 must be 1.0 (identity)");

    /* Test 1: [LLR-ATT-001] Zero-norm accelerometer input (e.g. 0G freefall) */
    fcs_imu_filtered_t zero_accel = {0};
    zero_accel.gyro_x_dps = 0.0f;
    zero_accel.accel_x_g  = 0.0f;
    zero_accel.accel_y_g  = 0.0f;
    zero_accel.accel_z_g  = 0.0f; /* 0G! */
    zero_accel.is_valid   = FCS_TRUE;

    fcs_euler_t euler;
    status = FCS_Attitude_Update(&zero_accel, 0.004f, &state, &euler);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "[LLR-ATT-001] Must survive 0G freefall without error");
    TEST_ASSERT(isfinite(euler.roll_deg) != 0, "Roll must be finite (no NaN on 0G)");
    TEST_ASSERT(isfinite(euler.pitch_deg) != 0, "Pitch must be finite (no NaN on 0G)");

    /* Test 2: [LLR-ATT-002] Corrupted degenerate quaternion recovery */
    state.q.q0 = 0.0f;
    state.q.q1 = 0.0f;
    state.q.q2 = 0.0f;
    state.q.q3 = 0.0f; /* Corrupted all zeros */
    fcs_imu_filtered_t normal_imu = {0};
    normal_imu.accel_z_g = 1.0f;
    normal_imu.is_valid  = FCS_TRUE;

    status = FCS_Attitude_Update(&normal_imu, 0.004f, &state, &euler);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Survives degenerate zero quaternion");
    TEST_ASSERT(state.q.q0 > 0.9f, "[LLR-ATT-002] Quaternion auto-reset to identity");

    /* Test 3: [LLR-ATT-003] Level flight attitude measurement */
    fcs_imu_filtered_t level_imu = {0};
    level_imu.accel_z_g = 1.0f; /* 1G down */
    level_imu.is_valid  = FCS_TRUE;
    (void)FCS_Attitude_Init(&state, 0.04f);
    status = FCS_Attitude_Update(&level_imu, 0.004f, &state, &euler);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Level flight attitude updates successfully");
    TEST_ASSERT_FLOAT_NEAR(0.0f, euler.roll_deg, 0.1f, "Level flight roll is ~0.0 deg");
    TEST_ASSERT_FLOAT_NEAR(0.0f, euler.pitch_deg, 0.1f, "Level flight pitch is ~0.0 deg");

    TEST_SUITE_END();
}
