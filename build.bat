@echo off
setlocal enabledelayedexpansion

echo [DainFlight] Building DO-178C DAL-A Flight Control System...

if not exist bin mkdir bin

set CC=
where zig >nul 2>&1 && set CC=zig cc
if not defined CC (
    where gcc >nul 2>&1 && set CC=gcc
)
if not defined CC (
    where clang >nul 2>&1 && set CC=clang
)

if defined CC (
    echo [DainFlight] Compiler detected: !CC!
    !CC! -std=c99 -Wall -Wextra -Werror -Wpedantic -O2 -Iinclude -Itest src\fcs_sensors.c src\fcs_attitude.c src\fcs_control.c src\fcs_mixer.c src\fcs_core.c test\test_sensors.c test\test_attitude.c test\test_control.c test\test_mixer.c test\test_core.c test\test_main.c -o bin\dainflight_tests.exe -lm
    if !ERRORLEVEL! NEQ 0 (
        echo [ERROR] Compilation failed with exit code !ERRORLEVEL!.
        exit /b !ERRORLEVEL!
    )
) else (
    where cl >nul 2>&1
    if !ERRORLEVEL! NEQ 0 (
        set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
        if exist "!VSWHERE!" (
            for /f "usebackq tokens=*" %%i in (`"!VSWHERE!" -products * -property installationPath`) do (
                if exist "%%i\VC\Auxiliary\Build\vcvars64.bat" (
                    call "%%i\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
                )
            )
        )
    )

    where cl >nul 2>&1
    if !ERRORLEVEL! EQU 0 (
        echo [DainFlight] Compiler detected: MSVC cl
        cl /nologo /W4 /WX /permissive- /O2 /Iinclude /Itest src\fcs_sensors.c src\fcs_attitude.c src\fcs_control.c src\fcs_mixer.c src\fcs_core.c test\test_sensors.c test\test_attitude.c test\test_control.c test\test_mixer.c test\test_core.c test\test_main.c /Fe:bin\dainflight_tests.exe /Fo:bin\
        if !ERRORLEVEL! NEQ 0 (
            echo [ERROR] MSVC Compilation failed with exit code !ERRORLEVEL!.
            exit /b !ERRORLEVEL!
        )
    ) else (
        echo [ERROR] No suitable C compiler found in PATH.
        exit /b 1
    )
)

echo [DainFlight] Compilation successful.
echo.
bin\dainflight_tests.exe
