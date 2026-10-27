# Plan for Software Aspects of Certification (PSAC)
## System: DainFlight Primary Flight Control System (FCS)
## Document ID: DF-PSAC-001 | Revision: 1.0
## Guidelines: RTCA DO-178C / EUROCAE ED-12C
## Design Assurance Level: DAL-A (Catastrophic / Loss of Aircraft)

---

### 1. System Overview
DainFlight is an airborne primary flight control computer software system designed to stabilize and control multi-rotor and VTOL aerial vehicles. The system samples inertial telemetry (gyroscopes and accelerometers), calculates vehicle attitude (quaternions / Euler angles), executes primary attitude and angular rate PID control laws, and commands flight control actuators (motors and aerodynamic control surfaces) at a deterministic cycle frequency of 250 Hz.

### 2. Software Safety Classification (System Hazard Assessment)
According to the System Safety Assessment (SSA) aligned with ARP4754A / ARP4761:
* **Failure Condition:** Loss of primary attitude stabilization or uncommanded full actuator deflection during flight.
* **Severity:** **Catastrophic** (leads directly to loss of aircraft and potential fatality).
* **Assigned Software Level:** **DO-178C Level A (DAL-A)**.
* **Required Objectives:** 71 objectives from DO-178C Annex A, including 30 objectives with independence and 100% Modified Condition/Decision Coverage (MC/DC).

### 3. Software Lifecycle Processes
1. **Planning Process:** Production of PSAC, SDP, SVP, SCMP, and SQAP.
2. **Development Process:**
   * High-Level Requirements (HLR) definition.
   * Software Architecture and Low-Level Requirements (LLR) definition.
   * Source Code implementation adhering to MISRA C:2012 / JSF AV C++.
3. **Verification Process:**
   * Requirements-based testing (normal range and robustness testing).
   * Structural coverage analysis (100% Statement, 100% Decision, 100% MC/DC).
   * Target hardware verification.
4. **Configuration Management Process:** Baselines, change control, problem reporting via Git.
5. **Quality Assurance Process:** Independent conformity reviews and audit readiness.

### 4. Software Environment & Tooling
* **Target Architecture:** 32-bit / 64-bit embedded real-time microcontrollers (e.g. ARM Cortex-M7 / PowerPC).
* **Implementation Language:** ISO C99 under strict MISRA C:2012 guidelines.
* **Compiler / Toolchain:** Clang / GCC with static analysis and pedantic compiler warnings enabled (`-Wall -Wextra -Werror -pedantic -std=c99`).
* **Dynamic Memory:** Strictly forbidden. Zero calls to `malloc()`, `calloc()`, `free()`, or heap memory post-initialization.
* **Recursion:** Strictly forbidden. Bounded stack depth statically verifiable.

### 5. Certification Deliverables Summary
* Software Requirements Specification (`docs/SRS.md`)
* Software Development Plan (`docs/SDP.md`)
* Software Verification Plan (`docs/SVP.md`)
* Source Code (`include/`, `src/`)
* Software Verification Cases & Procedures (`test/`)
* Software Accomplishment Summary (SAS)
