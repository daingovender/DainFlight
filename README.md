# DainFlight 🛩️

**DainFlight** is a safety-critical, hard real-time Primary Flight Control System (FCS) architected and implemented strictly according to **RTCA DO-178C / EUROCAE ED-12C Level A (DAL-A)** and **MISRA C:2012** avionics standards.

Originally inspired by Nicholas Rehm's `dRehmFlight`, this codebase is a complete ground-up re-engineering designed to meet civil and military airworthiness certification standards for fly-by-wire flight control computers.

---

## 🏛️ Aerospace Safety & Certification Architecture

Under **DO-178C Level A** (catastrophic failure condition / loss of aircraft), code cannot rely on hobbyist conventions:

| Design Dimension | Traditional Hobbyist Code | DainFlight (DO-178C Level A) |
| :--- | :--- | :--- |
| **Requirements Traceability** | None | 100% Bi-directional mapping (`HLR` $\leftrightarrow$ `LLR` $\leftrightarrow$ Source Code $\leftrightarrow$ Tests) |
| **Memory Allocation** | Dynamic heap (`malloc`/`new`) | **Zero Dynamic Allocation**. 100% static/stack memory with deterministic bounds |
| **State Management** | Naked mutable global variables | **Pure Re-Entrant Structures** (`fcs_system_state_t`). Zero global state |
| **Timing & Deadlines** | Variable loop sleep (`delay()`) | **Deterministic 250 Hz Cyclic Tick** ($\Delta t = 4.0\text{ ms}$) with jitter and range verification |
| **Mathematical Safety** | Implicit division by zero risks | **Singularity Protection**: Zero-norm gravity protection, NaN rejection, `asin` domain bounding |
| **Coding Standards** | Permissive C/C++ | **MISRA C:2012**: Explicit types (`float32_t`), single return point, zero recursion |

---

## 📁 Repository Structure

```text
DainFlight/
├── docs/                                  # Formal DO-178C Certification Artifacts
│   ├── PSAC.md                           # Plan for Software Aspects of Certification (Section 11.1)
│   ├── SRS.md                            # Software Requirements Specification (HLR & LLR)
│   ├── SDP.md                            # Software Development Plan (MISRA C:2012 Rules)
│   └── SVP.md                            # Software Verification Plan (MC/DC Structural Coverage)
├── include/                               # Public Avionics Interfaces & Type Definitions
│   ├── fcs_types.h                       # Explicit bit-width types, error codes, and state structures
│   ├── fcs_sensors.h                     # Sensor range plausibility & anti-aliasing low-pass filter
│   ├── fcs_attitude.h                    # Quaternion sensor fusion & Euler attitude estimation
│   ├── fcs_control.h                     # Fly-By-Wire PID attitude and rate stabilization laws
│   ├── fcs_mixer.h                       # Multi-rotor propulsion mixer & saturation limiting
│   └── fcs_core.h                        # Master 250 Hz executive scheduler & fail-safe state machine
├── src/                                   # MISRA C:2012 Compliant Avionics Implementations
│   ├── fcs_sensors.c                     # Sensor validation & digital IIR filtering
│   ├── fcs_attitude.c                    # Protected Madgwick attitude fusion (no singularities)
│   ├── fcs_control.c                     # Multi-axis PID stabilizer with anti-windup
│   ├── fcs_mixer.c                       # Quad-X mixing matrix & hard clamping [0.0, 1.0]
│   └── fcs_core.c                        # Executive cyclic handler and interlocks
├── test/                                  # Independent Verification Test Harness
│   ├── test_framework.h                  # Standalone C assertion & test runner framework
│   ├── test_sensors.c                    # Sensor boundary value analysis & NaN injection tests
│   ├── test_attitude.c                   # 0G freefall, degenerate quat, and domain tests
│   ├── test_control.c                    # Low-throttle zeroing, anti-windup, and rate tests
│   ├── test_mixer.c                      # Saturation limits and geometry mixing tests
│   ├── test_core.c                       # Arming interlocks, attitude departure, and fail-safe tests
│   └── test_main.c                       # Master test runner executable
├── CMakeLists.txt                         # Cross-platform CMake build configuration
├── build.bat                              # One-click Windows build and test execution script
└── .editorconfig                          # Avionics formatting standards (LF, 4 spaces, UTF-8)
```

---

## 🚀 Building and Running the Test Harness

DainFlight uses a zero-dependency build setup that compiles cleanly with any C99 compiler (`zig cc`, `gcc`, `clang`, or MSVC).

### One-Click Build & Run:
```powershell
.\build.bat
```

### Or Compile Directly:
```powershell
zig cc -std=c99 -Wall -Wextra -Werror -Wpedantic -O2 -Iinclude -Itest src/*.c test/*.c -o bin/dainflight_tests.exe -lm
.\bin\dainflight_tests.exe
```

---

## 📜 Traceability & Regulatory Reference
* **RTCA DO-178C / EUROCAE ED-12C:** Software Considerations in Airborne Systems and Equipment Certification (Design Assurance Level A).
* **MISRA C:2012:** Guidelines for the use of the C language in critical systems.
* **Lockheed Martin JSF AV C++:** Joint Strike Fighter Air Vehicle safety-critical coding standards.
