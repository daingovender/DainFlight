#include "test_framework.h"
#include "fcs_mixer.h"

void Run_Mixer_Tests(void)
{
    TEST_SUITE_BEGIN("Actuator Mixer & Saturation ([LLR-MIX-001]..[002])");

    fcs_moments_t moments = {0};
    fcs_actuators_t motors;

    /* Test 1: Level hover at 50% throttle */
    fcs_status_t status = FCS_Mixer_Process(0.5f, &moments, &motors);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Mixer_Process nominal OK");
    TEST_ASSERT_FLOAT_NEAR(0.5f, motors.m1_front_right, 1e-5f, "[LLR-MIX-001] M1 level thrust 0.5");
    TEST_ASSERT_FLOAT_NEAR(0.5f, motors.m2_rear_right,  1e-5f, "[LLR-MIX-001] M2 level thrust 0.5");
    TEST_ASSERT_FLOAT_NEAR(0.5f, motors.m3_rear_left,   1e-5f, "[LLR-MIX-001] M3 level thrust 0.5");
    TEST_ASSERT_FLOAT_NEAR(0.5f, motors.m4_front_left,  1e-5f, "[LLR-MIX-001] M4 level thrust 0.5");

    /* Test 2: Full Pitch Up (+1.0) at Full Throttle (1.0) */
    moments.pitch = 1.0f;
    status = FCS_Mixer_Process(1.0f, &moments, &motors);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Full pitch mixer OK");
    /* M1 = 1 - 0 + 1 + 0 = 2.0 -> clamped to 1.0 */
    /* M2 = 1 - 0 - 1 - 0 = 0.0 -> clamped to 0.0 */
    TEST_ASSERT_FLOAT_NEAR(1.0f, motors.m1_front_right, 1e-5f, "[LLR-MIX-002] M1 saturated at 1.0 max thrust");
    TEST_ASSERT_FLOAT_NEAR(0.0f, motors.m2_rear_right,  1e-5f, "[LLR-MIX-002] M2 saturated at 0.0 min thrust");
    TEST_ASSERT_FLOAT_NEAR(0.0f, motors.m3_rear_left,   1e-5f, "[LLR-MIX-002] M3 saturated at 0.0 min thrust");
    TEST_ASSERT_FLOAT_NEAR(1.0f, motors.m4_front_left,  1e-5f, "[LLR-MIX-002] M4 saturated at 1.0 max thrust");

    /* Test 3: NULL pointer protection */
    status = FCS_Mixer_Process(0.5f, NULL, &motors);
    TEST_ASSERT_EQUAL_INT(FCS_ERR_NULL_POINTER, status, "Reject NULL moments pointer");

    TEST_SUITE_END();
}
