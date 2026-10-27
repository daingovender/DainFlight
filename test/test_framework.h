/*******************************************************************************
 * FILE:         test_framework.h
 * SYSTEM:       DainFlight Software Verification Test Harness
 * STANDARD:     RTCA DO-178C Level A Verification
 * DESCRIPTION:  Lightweight, standalone test assertion framework.
 *******************************************************************************/

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

extern int g_tests_run;
extern int g_tests_passed;
extern int g_tests_failed;

#define TEST_SUITE_BEGIN(name) \
    printf("\n=== Running Test Suite: %s ===\n", name)

#define TEST_SUITE_END() \
    printf("--- Suite Complete ---\n")

#define TEST_ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (cond) { \
        g_tests_passed++; \
    } else { \
        g_tests_failed++; \
        printf("  [FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg); \
    } \
} while(0)

#define TEST_ASSERT_EQUAL_INT(expected, actual, msg) do { \
    g_tests_run++; \
    if ((expected) == (actual)) { \
        g_tests_passed++; \
    } else { \
        g_tests_failed++; \
        printf("  [FAIL] %s:%d: %s (Expected: %d, Got: %d)\n", __FILE__, __LINE__, msg, (int)(expected), (int)(actual)); \
    } \
} while(0)

#define TEST_ASSERT_FLOAT_NEAR(expected, actual, tol, msg) do { \
    g_tests_run++; \
    if (fabsf((float)(expected) - (float)(actual)) <= (float)(tol)) { \
        g_tests_passed++; \
    } else { \
        g_tests_failed++; \
        printf("  [FAIL] %s:%d: %s (Expected: %f, Got: %f, Tol: %f)\n", __FILE__, __LINE__, msg, (double)(expected), (double)(actual), (double)(tol)); \
    } \
} while(0)

#endif /* TEST_FRAMEWORK_H */
