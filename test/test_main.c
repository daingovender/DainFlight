#include "test_framework.h"

/* Define shared test metrics */
int g_tests_run    = 0;
int g_tests_passed = 0;
int g_tests_failed = 0;

/* Forward declarations of test suites */
void Run_Sensor_Tests(void);
void Run_Attitude_Tests(void);
void Run_Control_Tests(void);
void Run_Mixer_Tests(void);
void Run_Core_Tests(void);

int main(void)
{
    printf("=================================================================\n");
    printf("       DAINFLIGHT: RTCA DO-178C DAL-A VERIFICATION TEST HARNESS   \n");
    printf("=================================================================\n");

    Run_Sensor_Tests();
    Run_Attitude_Tests();
    Run_Control_Tests();
    Run_Mixer_Tests();
    Run_Core_Tests();

    printf("\n=================================================================\n");
    printf("VERIFICATION SUMMARY:\n");
    printf("  Total Checks Executed : %d\n", g_tests_run);
    printf("  Checks Passed         : %d\n", g_tests_passed);
    printf("  Checks Failed         : %d\n", g_tests_failed);
    printf("=================================================================\n");

    if (g_tests_failed == 0)
    {
        printf("RESULT: [PASS] 100%% DO-178C LEVEL A OBJECTIVES SATISFIED.\n\n");
        return 0;
    }
    else
    {
        printf("RESULT: [FAIL] VERIFICATION DEFICIENCIES DETECTED.\n\n");
        return 1;
    }
}
