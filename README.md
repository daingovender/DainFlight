# DainFlight 🛩️

**DainFlight** is a safety-critical, hard real-time Primary Flight Control System (FCS) re-engineered to comply with **RTCA DO-178C Level A** and **MISRA C:2012** avionics standards, based on Nicholas Rehm's open-source `dRehmFlight`.

---

## 📌 Architecture & Design Principles

Commercial and military flight control software (Fly-By-Wire) must adhere to the highest Design Assurance Level (**DAL-A: Loss of Aircraft / Life**). DainFlight transforms hobbyist flight controller paradigms into formal aerospace code:

1. **Bi-Directional Requirements Traceability:**
   Every single computation, constraint, and branch traces to formal systems specifications in [REQUIREMENTS.md](drehmflight_do178c/REQUIREMENTS.md).
2. **Defensive Arithmetic & Anti-Windup:**
   Guards against clock jitter, division by zero ($\Delta t \le 0$), sensor spikes, NaN/Inf, and integrator windup on ground/idle throttle.
3. **Pure Re-Entrant Architecture:**
   Zero global mutable state. Persistent integrator and derivative memories are passed via explicit typed pointers.
4. **Deterministic Worst-Case Execution Time (WCET):**
   Zero runtime dynamic memory allocation (`malloc`/`free`), no unbounded loops, no recursion, single point of return (MISRA Rule 15.5).

---

## 📁 Repository Structure

```text
DainFlight/
├── drehmflight_do178c/            # DO-178C DAL-A Avionics C Implementation
│   ├── REQUIREMENTS.md           # Systems & Low-Level Requirements Specification (SRS)
│   ├── include/
│   │   ├── fcs_types.h           # Explicit fixed bit-width types & state structs
│   │   └── fcs_control.h         # Flight control & mixer interfaces
│   └── src/
│       └── fcs_control.c         # MISRA C:2012 compliant PID controller & mixer
├── src/                          # Real-Time Java Reference Twin & Test Suite
│   ├── main/java/com/example/aerospace/
│   │   └── DrehmFlightController.java
│   └── test/java/com/example/aerospace/
│       └── DrehmFlightControllerTest.java # JUnit 5 verification test suite
├── drehmflight/                  # Baseline reference implementation (Nicholas Rehm)
├── pom.xml                       # Maven build descriptor (Java 21 LTS)
├── mvnw & mvnw.cmd               # Self-contained build wrapper
├── .editorconfig                 # Formatting standards
└── .gitignore                    # Build artifact and IDE exclusions
```

---

## 🧪 Verification & Testing

DainFlight includes an automated test harness covering all Low-Level Requirements (LLR), boundary conditions, and fail-safe transitions:

* `[LLR-PID-001]` $\Delta t$ validation & division-by-zero protection.
* `[HLR-FCS-200]` Loss-of-signal (LOS) and invalid telemetry fail-safe response.
* `[LLR-PID-002]` Automatic integrator zeroing under idle throttle ($< 6\%$).
* `[LLR-PID-003]` Integrator anti-windup clamping to saturation limits.
* `[LLR-PID-005]` Control moment bounding to $[-1.0, 1.0]$.
* `[LLR-MIX-001]` Quad-X motor mixing matrix with hard physical limits $[0.0, 1.0]$.

### Run Tests:
```powershell
.\mvnw.cmd test
```

### Build Executable Package:
```powershell
.\mvnw.cmd clean package
```

---

## 📜 Standards & Compliance
* **RTCA DO-178C / EUROCAE ED-12C:** Software Considerations in Airborne Systems and Equipment Certification (DAL-A).
* **MISRA C:2012:** Guidelines for the use of the C language in critical systems.
* **Lockheed Martin JSF AV C++:** Joint Strike Fighter Air Vehicle safety-critical coding principles.
