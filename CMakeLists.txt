cmake_minimum_required(VERSION 3.16)
project(DainFlight C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Enforce strict safety-critical compiler warnings (MISRA compliance)
if (MSVC)
    add_compile_options(/W4 /WX /permissive-)
else()
    add_compile_options(-Wall -Wextra -Werror -Wpedantic -Wconversion -Wsign-conversion -O2)
endif()

# Core Flight Control Library
add_library(dainflight_fcs STATIC
    src/fcs_sensors.c
    src/fcs_attitude.c
    src/fcs_control.c
    src/fcs_mixer.c
    src/fcs_core.c
)

target_include_directories(dainflight_fcs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Test Runner Executable
add_executable(dainflight_tests
    test/test_main.c
    test/test_sensors.c
    test/test_attitude.c
    test/test_control.c
    test/test_mixer.c
    test/test_core.c
)

target_link_libraries(dainflight_tests PRIVATE dainflight_fcs)

target_include_directories(dainflight_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
)

enable_testing()
add_test(NAME FlightVerificationTest COMMAND dainflight_tests)
