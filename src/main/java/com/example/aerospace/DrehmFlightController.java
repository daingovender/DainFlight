package com.example.aerospace;

import java.util.Objects;

/**
 * DO-178C DAL-A Compliant Port of dRehmFlight Primary Flight Computer.
 * Traceability:
 *   [HLR-FCS-100] Flight State Attitude Stabilization
 *   [HLR-FCS-200] Sensor & Input Fail-Safe Protection
 *   [HLR-FCS-300] Actuator Physical Saturation Limiting
 */
public final class DrehmFlightController {

    public static final float DT_MIN_SEC = 0.0005f; // 2000 Hz
    public static final float DT_MAX_SEC = 0.0500f; // 20 Hz
    public static final float THROTTLE_IDLE_THRESH = 0.0600f;
    public static final float MOMENT_MAX = 1.0f;
    public static final float MOMENT_MIN = -1.0f;
    public static final float MOTOR_MAX = 1.0f;
    public static final float MOTOR_MIN = 0.0f;
    private static final float PID_SCALE_FACTOR = 0.01f;

    public enum Status {
        OK,
        ERR_INVALID_DT,
        ERR_LOSS_OF_SIGNAL,
        ERR_SENSOR_OUT_OF_BOUNDS
    }

    public record PidGains(float kp, float ki, float kd, float iLimit) {
        public PidGains {
            if (kp < 0 || ki < 0 || kd < 0 || iLimit < 0) {
                throw new IllegalArgumentException("PID gains and limits must be non-negative");
            }
        }
    }

    public static class AxisState {
        private float integral = 0.0f;
        private float prevError = 0.0f;

        public float getIntegral() { return integral; }
        public float getPrevError() { return prevError; }
        public void reset() {
            this.integral = 0.0f;
            this.prevError = 0.0f;
        }
    }

    public static class ControllerState {
        public final AxisState roll = new AxisState();
        public final AxisState pitch = new AxisState();
        public final AxisState yaw = new AxisState();

        public void reset() {
            roll.reset();
            pitch.reset();
            yaw.reset();
        }
    }

    public record PilotCommand(
        float throttle,
        float rollDeg,
        float pitchDeg,
        float yawRate,
        boolean isValid
    ) {}

    public record Telemetry(
        float rollDeg,
        float pitchDeg,
        float gyroX,
        float gyroY,
        float gyroZ,
        boolean isValid
    ) {}

    public record Moments(float roll, float pitch, float yaw) {}

    public record Actuators(float m1, float m2, float m3, float m4) {}

    public record StepResult(Status status, Moments moments) {}

    /**
     * [HLR-FCS-100, HLR-FCS-200]
     * Executes one deterministic stabilization cycle.
     */
    public static StepResult updateAttitudeControl(
        final PilotCommand cmd,
        final Telemetry telem,
        final PidGains rollGains,
        final PidGains pitchGains,
        final PidGains yawGains,
        final float dtSec,
        final ControllerState state
    ) {
        Objects.requireNonNull(cmd, "PilotCommand cannot be null");
        Objects.requireNonNull(telem, "Telemetry cannot be null");
        Objects.requireNonNull(rollGains, "RollGains cannot be null");
        Objects.requireNonNull(pitchGains, "PitchGains cannot be null");
        Objects.requireNonNull(yawGains, "YawGains cannot be null");
        Objects.requireNonNull(state, "ControllerState cannot be null");

        // [LLR-PID-001] Time Delta Validation (Guards against division by zero)
        if (Float.isNaN(dtSec) || Float.isInfinite(dtSec) || dtSec < DT_MIN_SEC || dtSec > DT_MAX_SEC) {
            return new StepResult(Status.ERR_INVALID_DT, new Moments(0.0f, 0.0f, 0.0f));
        }

        // [HLR-FCS-200] Sensor & Radio Validity Checks
        if (!cmd.isValid()) {
            return new StepResult(Status.ERR_LOSS_OF_SIGNAL, new Moments(0.0f, 0.0f, 0.0f));
        }
        if (!telem.isValid()) {
            return new StepResult(Status.ERR_SENSOR_OUT_OF_BOUNDS, new Moments(0.0f, 0.0f, 0.0f));
        }

        // --- Roll Axis [LLR-PID-002, LLR-PID-003, LLR-PID-004] ---
        final float errorRoll = cmd.rollDeg() - telem.rollDeg();
        state.roll.integral += (errorRoll * dtSec);
        if (cmd.throttle() < THROTTLE_IDLE_THRESH) {
            state.roll.integral = 0.0f;
        }
        state.roll.integral = clamp(state.roll.integral, -rollGains.iLimit(), rollGains.iLimit());
        final float derivRoll = telem.gyroX();
        final float rawRoll = PID_SCALE_FACTOR * (
            (rollGains.kp() * errorRoll) +
            (rollGains.ki() * state.roll.integral) -
            (rollGains.kd() * derivRoll)
        );
        final float rollMoment = clamp(rawRoll, MOMENT_MIN, MOMENT_MAX);

        // --- Pitch Axis [LLR-PID-002, LLR-PID-003, LLR-PID-004] ---
        final float errorPitch = cmd.pitchDeg() - telem.pitchDeg();
        state.pitch.integral += (errorPitch * dtSec);
        if (cmd.throttle() < THROTTLE_IDLE_THRESH) {
            state.pitch.integral = 0.0f;
        }
        state.pitch.integral = clamp(state.pitch.integral, -pitchGains.iLimit(), pitchGains.iLimit());
        final float derivPitch = telem.gyroY();
        final float rawPitch = PID_SCALE_FACTOR * (
            (pitchGains.kp() * errorPitch) +
            (pitchGains.ki() * state.pitch.integral) -
            (pitchGains.kd() * derivPitch)
        );
        final float pitchMoment = clamp(rawPitch, MOMENT_MIN, MOMENT_MAX);

        // --- Yaw Axis [LLR-PID-002, LLR-PID-003] ---
        final float errorYaw = cmd.yawRate() - telem.gyroZ();
        state.yaw.integral += (errorYaw * dtSec);
        if (cmd.throttle() < THROTTLE_IDLE_THRESH) {
            state.yaw.integral = 0.0f;
        }
        state.yaw.integral = clamp(state.yaw.integral, -yawGains.iLimit(), yawGains.iLimit());
        final float derivYaw = (errorYaw - state.yaw.prevError) / dtSec;
        state.yaw.prevError = errorYaw;
        final float rawYaw = PID_SCALE_FACTOR * (
            (yawGains.kp() * errorYaw) +
            (yawGains.ki() * state.yaw.integral) +
            (yawGains.kd() * derivYaw)
        );
        final float yawMoment = clamp(rawYaw, MOMENT_MIN, MOMENT_MAX);

        return new StepResult(Status.OK, new Moments(rollMoment, pitchMoment, yawMoment));
    }

    /**
     * [LLR-MIX-001, LLR-MIX-002]
     * Mixes throttle and moments into 4 constrained motor commands.
     */
    public static Actuators mixActuators(final float throttle, final Moments moments) {
        Objects.requireNonNull(moments, "Moments cannot be null");
        final float t = clamp(throttle, MOTOR_MIN, MOTOR_MAX);
        final float r = moments.roll();
        final float p = moments.pitch();
        final float y = moments.yaw();

        final float m1 = clamp(t - r + p + y, MOTOR_MIN, MOTOR_MAX);
        final float m2 = clamp(t - r - p - y, MOTOR_MIN, MOTOR_MAX);
        final float m3 = clamp(t + r - p + y, MOTOR_MIN, MOTOR_MAX);
        final float m4 = clamp(t + r + p - y, MOTOR_MIN, MOTOR_MAX);

        return new Actuators(m1, m2, m3, m4);
    }

    private static float clamp(final float value, final float min, final float max) {
        if (value > max) return max;
        if (value < min) return min;
        return value;
    }
}
