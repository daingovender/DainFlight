# Flight Control System (FCS) Software Requirements Specification (SRS)
## Standard: RTCA DO-178C / EUROCAE ED-12C
## Design Assurance Level: DAL-A (Loss of Aircraft / Catastrophic)
## Subsystem: dRehmFlight Primary Flight Control Computer (FCC)

---

### 1. High-Level Requirements (HLR)

* **[HLR-FCS-100] Flight State Attitude Stabilization**
  The flight control computer shall compute corrective control moments (Roll, Pitch, Yaw) at a deterministic cycle rate to drive the vehicle's attitude toward the commanded setpoint.

* **[HLR-FCS-200] Sensor & Input Fail-Safe Protection**
  The flight control computer shall monitor all sensor telemetry and pilot inputs for plausibility and validity. Upon detection of loss-of-signal (LOS) or invalid telemetry, the system shall command a neutral, safe state.

* **[HLR-FCS-300] Actuator Physical Saturation Limiting**
  The flight control computer shall ensure that commanded actuator outputs (motors/servos) never exceed mechanical or electrical limits, regardless of sensor spikes or integrator windup.

---

### 2. Low-Level Requirements (LLR)

#### Module: PID Attitude & Rate Stabilization
* **[LLR-PID-001] Time Delta Validation**
  The controller shall validate that the cycle time delta $\Delta t$ is within $(0.0005\,\text{s}, 0.0500\,\text{s}]$ ($20\,\text{Hz}$ to $2000\,\text{Hz}$). If $\Delta t$ is outside this range or non-finite (NaN/Inf), the controller shall reject the step, maintain prior state, and report `FCS_ERR_INVALID_DT`.

* **[LLR-PID-002] Low-Throttle Anti-Windup**
  If the commanded throttle is below idle threshold ($1060\,\mu\text{s}$ or normalized $< 0.06$), all integral terms (`integral_roll`, `integral_pitch`, `integral_yaw`) shall be clamped to exactly $0.0$ to prevent ground spool-up hazards.

* **[LLR-PID-003] Integrator Saturation (Anti-Windup)**
  The integrated error for each axis shall be strictly saturated within $[-I_{\text{limit}}, +I_{\text{limit}}]$.

* **[LLR-PID-004] Derivative Jitter & Kick Mitigation**
  The roll and pitch derivative terms shall be computed directly from filtered gyroscope angular rates ($\omega_x, \omega_y$) to avoid derivative kick from sudden setpoint step changes.

* **[LLR-PID-005] Output Moment Sizing**
  Each axis stabilized PID output shall be constrained within $[-1.0, +1.0]$.

#### Module: Actuator Mixer
* **[LLR-MIX-001] Quadcopter Mixing Matrix**
  The mixer shall combine normalized throttle ($T \in [0.0, 1.0]$) and normalized moments ($P, Q, R \in [-1.0, 1.0]$) according to standard quad-X airframe geometry:
  * Motor 1 (Front Right, CCW): $T - P + Q + R$
  * Motor 2 (Rear Right, CW): $T - P - Q - R$
  * Motor 3 (Rear Left, CCW): $T + P - Q + R$
  * Motor 4 (Front Left, CW): $T + P + Q - R$

* **[LLR-MIX-002] Actuator Hard Limits**
  All resulting motor output commands shall be strictly constrained between $0.0$ (idle/off) and $1.0$ (maximum thrust). No output shall underflow or overflow under any combination of inputs.
