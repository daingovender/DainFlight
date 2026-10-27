@echo off
setlocal

echo [DainFlight] Building DO-178C DAL-A Flight Control System...

if not exist bin mkdir bin

set CC=zig cc
where zig >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    set CC=gcc
    where gcc >nul 2>&1
    if %ERRORLEVEL% NEQ 0 (
        set CC=clang
        where clang >nul 2>&1
        if %ERRORLEVEL% NEQ 0 (
            echo [ERROR] No suitable C compiler found in PATH.
            exit /b 1
        )
    )
)

echo [DainFlight] Compiler detected: %CC%

%CC% -std=c99 -Wall -Wextra -Werror -Wpedantic -O2 -Iinclude -Itest src\fcs_sensors.c src\fcs_attitude.c src\fcs_control.c src\fcs_mixer.c src\fcs_core.c test\test_sensors.c test\test_attitude.c test\test_control.c test\test_mixer.c test\test_core.c test\test_main.c -o bin\dainflight_tests.exe -lm

if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation failed with exit code %ERRORLEVEL%.
    exit /b %ERRORLEVEL%
)

echo [DainFlight] Compilation successful.
echo.
bin\dainflight_tests.exe
