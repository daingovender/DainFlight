# Software Requirements Specification (SRS)
## System: DainFlight Primary Flight Control System (FCS)
## Document ID: DF-SRS-001 | Revision: 1.0
## Standard: RTCA DO-178C Level A

---

### 1. High-Level Requirements (HLR)

* **[HLR-FCS-100] Deterministic Cyclic Execution**
  The flight control software shall execute a complete sensing, estimation, control, and actuation cycle at a deterministic nominal cycle frequency of 250 Hz (nominal $\Delta t = 0.0040\text{ s}$).

* **[HLR-FCS-200] Sensor Telemetry Validation and Signal Conditioning**
  The software shall monitor inertial sensor inputs (3-axis gyroscope, 3-axis accelerometer) for physical range plausibility, finite floating-point validity, and electrical disconnects. Out-of-bounds telemetry shall trigger fail-safe mode.

* **[HLR-FCS-300] Multi-Axis Attitude Estimation**
  The software shall calculate vehicle attitude angles (Roll, Pitch, Yaw) from validated inertial sensors using sensor fusion without trigonometric domain errors or zero-division singularities.

* **[HLR-FCS-400] Attitude & Rate Stabilization (Fly-By-Wire)**
  The software shall calculate corrective pitch, roll, and yaw control moments using proportional-integral-derivative (PID) control laws to eliminate state error relative to pilot commands.

* **[HLR-FCS-500] Actuator Mixing & Saturation Clamping**
  The software shall distribute stabilized control moments and throttle demand to individual propulsion actuators, ensuring no actuator command exceeds safe physical operating boundaries $[0.0, 1.0]$.

* **[HLR-FCS-600] Autonomous System Fail-Safe Protection**
  Upon detection of signal loss, unrecoverable sensor failure, attitude departure ($> 80^\circ$ pitch/roll), or internal software error, the software shall transition to `FCS_STATE_FAILSAFE` and command zero motor thrust.

---

### 2. Low-Level Requirements (LLR)

#### Module 1: Sensor Processing (`fcs_sensors`)
* **[LLR-SEN-001] Floating Point Sanity Check**
  All raw sensor inputs shall be verified to be finite (neither NaN nor $\pm\infty$). If any input is non-finite, the function shall return `FCS_ERR_SENSOR_NAN`.
* **[LLR-SEN-002] Gyroscope Plausibility Bounds**
  Angular rate inputs for each axis ($X, Y, Z$) shall be within $[-2000.0, +2000.0]^\circ/\text{s}$. Values exceeding this range shall be rejected with `FCS_ERR_SENSOR_OUT_OF_BOUNDS`.
* **[LLR-SEN-003] Accelerometer Plausibility Bounds**
  Linear acceleration inputs for each axis ($X, Y, Z$) shall be within $[-16.0, +16.0]\text{ g}$. Values exceeding this range shall be rejected with `FCS_ERR_SENSOR_OUT_OF_BOUNDS`.
* **[LLR-SEN-004] First-Order Low-Pass Filter**
  Valid gyro and accelerometer signals shall be conditioned using a digital first-order low-pass filter:
  $$y[k] = (1 - \alpha) \cdot y[k-1] + \alpha \cdot x[k]$$
  where filter coefficient $\alpha$ must be constrained to $(0.0, 1.0]$.

#### Module 2: Attitude Estimator (`fcs_attitude`)
* **[LLR-ATT-001] Accelerometer Zero-Norm Protection**
  During sensor fusion, the Euclidean norm of the acceleration vector $\|a\| = \sqrt{a_x^2 + a_y^2 + a_z^2}$ shall be compared against a threshold $\epsilon = 0.001\text{ g}$. If $\|a\| < \epsilon$, accelerometer corrective feedback shall be skipped to prevent division by zero.
* **[LLR-ATT-002] Quaternion Normalization Protection**
  The quaternion norm $\|q\| = \sqrt{q_0^2 + q_1^2 + q_2^2 + q_3^2}$ shall be verified to be non-zero ($\|q\| > 1.0 \times 10^{-6}$) before normalization. If degenerate, the quaternion shall be reset to the identity quaternion $[1, 0, 0, 0]^T$.
* **[LLR-ATT-003] Trigonometric Domain Clamping**
  When converting quaternions to Euler angles, the argument to `asin` (pitch calculation) shall be explicitly clamped to $[-0.999999, +0.999999]$ to prevent `NaN` returns resulting from floating-point roundoff.

#### Module 3: PID Flight Controller (`fcs_control`)
* **[LLR-PID-001] Time Delta $\Delta t$ Verification**
  The execution delta $\Delta t$ shall be validated to lie strictly within $(0.0005, 0.0500]\text{ s}$ ($20\text{ Hz}$ to $2000\text{ Hz}$). Steps outside this window shall be rejected with `FCS_ERR_INVALID_DT`.
* **[LLR-PID-002] Low-Throttle Anti-Windup**
  If commanded throttle is below idle threshold ($0.060$ or $6\%$), all axis integral accumulators shall be forcibly set to $0.0$ to eliminate ground spool-up hazards.
* **[LLR-PID-003] Integrator Saturation Limit**
  The integral accumulator for each axis shall be clamped to $[-I_{\text{limit}}, +I_{\text{limit}}]$.
* **[LLR-PID-004] Derivative Jitter & Kick Mitigation**
  The derivative component for pitch and roll shall be computed directly from filtered gyroscope rates rather than error differentiation, preventing high-frequency control spikes upon setpoint changes.
* **[LLR-PID-005] Output Moment Sizing**
  Calculated roll, pitch, and yaw moments shall be saturated within $[-1.0, +1.0]$.

#### Module 4: Actuator Mixer (`fcs_mixer`)
* **[LLR-MIX-001] Quad-X Mixer Transformation**
  Motor outputs shall be mixed using the rigid-body geometry matrix:
  $$M_1 = T - R + P + Y, \quad M_2 = T - R - P - Y, \quad M_3 = T + R - P + Y, \quad M_4 = T + R + P - Y$$
* **[LLR-MIX-002] Hard Saturation Limits**
  Every motor output command shall be strictly clamped within $[0.0, 1.0]$.

#### Module 5: Executive State Machine (`fcs_core`)
* **[LLR-FSM-001] Arming Interlock**
  Transition from `FCS_STATE_DISARMED` to `FCS_STATE_ARMED` shall require:
  1. Pilot arm switch TRUE.
  2. Commanded throttle below idle ($< 0.06$).
  3. Valid sensor telemetry and attitude within $\pm 10^\circ$ of level.
* **[LLR-FSM-002] Attitude Departure Trip**
  If in `FCS_STATE_ARMED` and estimated pitch or roll angle exceeds $\pm 80.0^\circ$, the system shall trip to `FCS_STATE_FAILSAFE` and disable motor commands.
* **[LLR-FSM-003] Deterministic Step Function**
  The master step function `FCS_Step()` shall execute in bounded constant time (no loops, no dynamic allocations, no recursion).
