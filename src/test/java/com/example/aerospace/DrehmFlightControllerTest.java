package com.example.aerospace;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

/**
 * DO-178C DAL-A Verification Test Suite for dRehmFlight Primary Flight Computer.
 * Verifies every LLR and boundary condition.
 */
class DrehmFlightControllerTest {

    private DrehmFlightController.PidGains defaultGains;
    private DrehmFlightController.ControllerState state;

    @BeforeEach
    void setUp() {
        defaultGains = new DrehmFlightController.PidGains(2.0f, 0.5f, 0.1f, 15.0f);
        state = new DrehmFlightController.ControllerState();
    }

    @Test
    @DisplayName("[LLR-PID-001] Should reject dt below minimum (e.g. 0.0s / division by zero protection)")
    void shouldRejectInvalidDtZeroOrNegative() {
        DrehmFlightController.PilotCommand cmd = new DrehmFlightController.PilotCommand(0.5f, 0f, 0f, 0f, true);
        DrehmFlightController.Telemetry telem = new DrehmFlightController.Telemetry(0f, 0f, 0f, 0f, 0f, true);

        DrehmFlightController.StepResult result = DrehmFlightController.updateAttitudeControl(
            cmd, telem, defaultGains, defaultGains, defaultGains, 0.0f, state
        );

        assertEquals(DrehmFlightController.Status.ERR_INVALID_DT, result.status());
        assertEquals(0.0f, result.moments().roll());
    }

    @ParameterizedTest
    @ValueSource(floats = {-0.01f, 0.0001f, 0.06f, Float.NaN, Float.POSITIVE_INFINITY})
    @DisplayName("[LLR-PID-001] Boundary testing for time delta")
    void shouldRejectOutOfBoundsDt(float testDt) {
        DrehmFlightController.PilotCommand cmd = new DrehmFlightController.PilotCommand(0.5f, 0f, 0f, 0f, true);
        DrehmFlightController.Telemetry telem = new DrehmFlightController.Telemetry(0f, 0f, 0f, 0f, 0f, true);

        DrehmFlightController.StepResult result = DrehmFlightController.updateAttitudeControl(
            cmd, telem, defaultGains, defaultGains, defaultGains, testDt, state
        );

        assertEquals(DrehmFlightController.Status.ERR_INVALID_DT, result.status());
    }

    @Test
    @DisplayName("[HLR-FCS-200] Should enter fail-safe when pilot radio signal is lost")
    void shouldEnterFailsafeOnLossOfSignal() {
        DrehmFlightController.PilotCommand cmd = new DrehmFlightController.PilotCommand(0.5f, 10f, 10f, 0f, false);
        DrehmFlightController.Telemetry telem = new DrehmFlightController.Telemetry(0f, 0f, 0f, 0f, 0f, true);

        DrehmFlightController.StepResult result = DrehmFlightController.updateAttitudeControl(
            cmd, telem, defaultGains, defaultGains, defaultGains, 0.004f, state
        );

        assertEquals(DrehmFlightController.Status.ERR_LOSS_OF_SIGNAL, result.status());
        assertEquals(0.0f, result.moments().roll());
        assertEquals(0.0f, result.moments().pitch());
        assertEquals(0.0f, result.moments().yaw());
    }

    @Test
    @DisplayName("[HLR-FCS-200] Should enter fail-safe when sensor telemetry is invalid")
    void shouldEnterFailsafeOnInvalidTelemetry() {
        DrehmFlightController.PilotCommand cmd = new DrehmFlightController.PilotCommand(0.5f, 0f, 0f, 0f, true);
        DrehmFlightController.Telemetry telem = new DrehmFlightController.Telemetry(0f, 0f, 0f, 0f, 0f, false);

        DrehmFlightController.StepResult result = DrehmFlightController.updateAttitudeControl(
            cmd, telem, defaultGains, defaultGains, defaultGains, 0.004f, state
        );

        assertEquals(DrehmFlightController.Status.ERR_SENSOR_OUT_OF_BOUNDS, result.status());
    }

    @Test
    @DisplayName("[LLR-PID-002] Should reset integrator when throttle is at or below idle")
    void shouldClearIntegratorAtLowThrottle() {
        // Step 1: Accumulate integrator at high throttle
        DrehmFlightController.PilotCommand cmdNormal = new DrehmFlightController.PilotCommand(0.5f, 10f, 0f, 0f, true);
        DrehmFlightController.Telemetry telem = new DrehmFlightController.Telemetry(0f, 0f, 0f, 0f, 0f, true);
        DrehmFlightController.updateAttitudeControl(cmdNormal, telem, defaultGains, defaultGains, defaultGains, 0.004f, state);
        assertTrue(state.roll.getIntegral() > 0.0f, "Integrator should have accumulated");

        // Step 2: Cut throttle to idle (< 0.06)
        DrehmFlightController.PilotCommand cmdIdle = new DrehmFlightController.PilotCommand(0.02f, 10f, 0f, 0f, true);
        DrehmFlightController.updateAttitudeControl(cmdIdle, telem, defaultGains, defaultGains, defaultGains, 0.004f, state);

        assertEquals(0.0f, state.roll.getIntegral(), "Integrator must clamp to 0.0 at low throttle");
    }

    @Test
    @DisplayName("[LLR-PID-003] Should saturate integrator to prevent windup")
    void shouldSaturateIntegratorToLimit() {
        DrehmFlightController.PilotCommand cmd = new DrehmFlightController.PilotCommand(0.8f, 30f, 0f, 0f, true);
        DrehmFlightController.Telemetry telem = new DrehmFlightController.Telemetry(0f, 0f, 0f, 0f, 0f, true);

        // Run 500 steps of constant error
        for (int i = 0; i < 500; i++) {
            DrehmFlightController.updateAttitudeControl(cmd, telem, defaultGains, defaultGains, defaultGains, 0.010f, state);
        }

        assertEquals(defaultGains.iLimit(), state.roll.getIntegral(), 1e-4f, "Integrator must be clamped at iLimit");
    }

    @Test
    @DisplayName("[LLR-PID-005] Should clamp moments to [-1.0, 1.0]")
    void shouldClampMomentsToUnitBounds() {
        DrehmFlightController.PidGains highGains = new DrehmFlightController.PidGains(100.0f, 50.0f, 10.0f, 50.0f);
        DrehmFlightController.PilotCommand cmd = new DrehmFlightController.PilotCommand(0.5f, 45f, 45f, 45f, true);
        DrehmFlightController.Telemetry telem = new DrehmFlightController.Telemetry(-45f, -45f, 0f, 0f, 0f, true);

        DrehmFlightController.StepResult result = DrehmFlightController.updateAttitudeControl(
            cmd, telem, highGains, highGains, highGains, 0.004f, state
        );

        assertEquals(1.0f, result.moments().roll());
        assertEquals(1.0f, result.moments().pitch());
    }

    @Test
    @DisplayName("[LLR-MIX-001, LLR-MIX-002] Mixer should obey quad-X geometry and constrain [0.0, 1.0]")
    void shouldCorrectlyMixAndConstrainActuators() {
        // Full throttle, full pitch up
        DrehmFlightController.Moments moments = new DrehmFlightController.Moments(0.0f, 1.0f, 0.0f);
        DrehmFlightController.Actuators actuators = DrehmFlightController.mixActuators(1.0f, moments);

        // M1 = 1 - 0 + 1 + 0 = 2 -> clamped to 1.0
        // M2 = 1 - 0 - 1 - 0 = 0 -> clamped to 0.0
        assertEquals(1.0f, actuators.m1());
        assertEquals(0.0f, actuators.m2());
        assertEquals(0.0f, actuators.m3());
        assertEquals(1.0f, actuators.m4());
    }
}
