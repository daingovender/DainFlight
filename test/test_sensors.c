#include "test_framework.h"
#include "fcs_sensors.h"

void Run_Sensor_Tests(void)
{
    TEST_SUITE_BEGIN("Sensor Conditioning & Plausibility ([LLR-SEN-001]..[004])");

    fcs_sensor_filter_state_t state;
    fcs_status_t status = FCS_Sensors_Init(&state, 0.2f, 0.2f);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Sensors_Init should succeed with valid alphas");

    /* Test 1: NULL pointer defensive protection */
    status = FCS_Sensors_Process(NULL, &state, NULL);
    TEST_ASSERT_EQUAL_INT(FCS_ERR_NULL_POINTER, status, "Sensors_Process should reject NULL pointers");

    /* Test 2: [LLR-SEN-001] Non-finite / NaN rejection */
    fcs_imu_raw_t raw_nan = {0};
    raw_nan.gyro_x_dps = (float32_t)sqrt(-1.0); /* NaN */
    raw_nan.is_healthy = FCS_TRUE;
    fcs_imu_filtered_t filtered;
    status = FCS_Sensors_Process(&raw_nan, &state, &filtered);
    TEST_ASSERT_EQUAL_INT(FCS_ERR_SENSOR_NAN, status, "[LLR-SEN-001] Reject NaN gyro input");
    TEST_ASSERT_EQUAL_INT(FCS_FALSE, filtered.is_valid, "Filtered output marked invalid on NaN");

    /* Test 3: [LLR-SEN-002] Gyro out of bounds (+2500 dps > 2000 dps) */
    fcs_imu_raw_t raw_high_gyro = {0};
    raw_high_gyro.gyro_x_dps = 2500.0f;
    raw_high_gyro.accel_z_g  = 1.0f;
    raw_high_gyro.is_healthy = FCS_TRUE;
    status = FCS_Sensors_Process(&raw_high_gyro, &state, &filtered);
    TEST_ASSERT_EQUAL_INT(FCS_ERR_SENSOR_OUT_OF_BOUNDS, status, "[LLR-SEN-002] Reject gyro > 2000 dps");

    /* Test 4: [LLR-SEN-003] Accelerometer out of bounds (+20g > 16g) */
    fcs_imu_raw_t raw_high_accel = {0};
    raw_high_accel.gyro_x_dps = 0.0f;
    raw_high_accel.accel_z_g  = 20.0f;
    raw_high_accel.is_healthy = FCS_TRUE;
    status = FCS_Sensors_Process(&raw_high_accel, &state, &filtered);
    TEST_ASSERT_EQUAL_INT(FCS_ERR_SENSOR_OUT_OF_BOUNDS, status, "[LLR-SEN-003] Reject accel > 16g");

    /* Test 5: [LLR-SEN-004] Low-pass filter smoothing */
    fcs_imu_raw_t raw_step = {0};
    raw_step.gyro_x_dps = 100.0f;
    raw_step.accel_z_g  = 1.0f;
    raw_step.is_healthy = FCS_TRUE;
    status = FCS_Sensors_Process(&raw_step, &state, &filtered);
    TEST_ASSERT_EQUAL_INT(FCS_OK, status, "Valid sensor data processes OK");
    /* With alpha=0.2 and initial=0, step output is 0.8*0 + 0.2*100 = 20.0 */
    TEST_ASSERT_FLOAT_NEAR(20.0f, filtered.gyro_x_dps, 0.001f, "[LLR-SEN-004] Filter step output is 20 dps");

    TEST_SUITE_END();
}
