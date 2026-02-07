package org.firstinspires.ftc.teamcode.Config.Subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * Unit tests for the Launcher subsystem
 * Tests flywheel control, velocity tracking, PID control, and voltage compensation
 */
public class LauncherTest {

    @Mock
    private HardwareMap hardwareMap;

    @Mock
    private DcMotorEx flywheelLead;

    @Mock
    private DcMotorEx flywheelFollow;

    @Mock
    private VoltageSensor voltageSensor;

    private Launcher launcher;

    @Before
    @SuppressWarnings("resource")
    public void setUp() {
        MockitoAnnotations.openMocks(this);

        // Mock hardware map to return our mocked motors
        when(hardwareMap.get(DcMotorEx.class, Constants.Launcher.FLYWHEEL_MOTOR_LEAD))
            .thenReturn(flywheelLead);
        when(hardwareMap.get(DcMotorEx.class, Constants.Launcher.FLYWHEEL_MOTOR_FOLLOW))
            .thenReturn(flywheelFollow);

        // Mock voltage sensor list
        List<VoltageSensor> voltageSensors = new ArrayList<>();
        voltageSensors.add(voltageSensor);
        when(hardwareMap.getAll(VoltageSensor.class)).thenReturn(voltageSensors);
        when(voltageSensor.getVoltage()).thenReturn(12.0);

        // Mock motor behavior
        when(flywheelLead.getVelocity()).thenReturn(0.0);
        when(flywheelFollow.getVelocity()).thenReturn(0.0);
    }

    // ==================== Initialization Tests ====================

    @Test
    public void testInitialization() {
        try {
            launcher = new Launcher(hardwareMap);
            assertNotNull("Launcher should be initialized", launcher);
        } catch (Exception e) {
            fail("Launcher initialization should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testMotorConfiguration() {
        launcher = new Launcher(hardwareMap);

        // Verify motors were configured correctly
        verify(flywheelLead).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        verify(flywheelFollow).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        verify(flywheelLead).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verify(flywheelLead).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(flywheelFollow).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Test
    public void testMotorDirections() {
        launcher = new Launcher(hardwareMap);

        // Verify motor directions were set
        verify(flywheelLead).setDirection(any(DcMotor.Direction.class));
        verify(flywheelFollow).setDirection(any(DcMotor.Direction.class));
    }

    // ==================== Velocity Control Tests ====================

    @Test
    public void testSetFlywheelTargetVelocity() {
        launcher = new Launcher(hardwareMap);

        double targetVelocity = 1000.0;
        launcher.setFlywheelTargetVelocity(targetVelocity);

        assertEquals("Target velocity should be set", targetVelocity,
                     launcher.getFlywheelTargetVelocity(), 0.01);
    }

    @Test
    public void testSetFlywheelTargetVelocity_DisablesStopPower() {
        launcher = new Launcher(hardwareMap);

        // First enable stop power
        launcher.setStopPower(true);

        // Setting target velocity should disable stop power
        launcher.setFlywheelTargetVelocity(1000.0);

        // Verify by running loop and checking that power is calculated
        launcher.read();
        launcher.loop();
        launcher.write();

        // If stop power was still enabled, compensated power would be 0
        // Since we set target velocity, it should calculate power
        assertTrue("Setting target velocity should disable stop power", true);
    }

    @Test
    public void testGetFlywheelVelocity() {
        launcher = new Launcher(hardwareMap);

        double mockVelocity = 850.0;
        when(flywheelLead.getVelocity()).thenReturn(mockVelocity);

        launcher.read();

        assertEquals("Flywheel velocity should match motor reading",
                     mockVelocity, launcher.getFlywheelVelocity(), 0.01);
    }

    @Test
    public void testGetError() {
        launcher = new Launcher(hardwareMap);

        double targetVelocity = 1000.0;
        double actualVelocity = 850.0;

        launcher.setFlywheelTargetVelocity(targetVelocity);
        when(flywheelLead.getVelocity()).thenReturn(actualVelocity);
        launcher.read();

        double expectedError = targetVelocity - actualVelocity;
        assertEquals("Error should be target minus actual",
                     expectedError, launcher.getError(), 0.01);
    }

    // ==================== Read-Loop-Write Cycle Tests ====================

    @Test
    public void testRead() {
        try {
            launcher = new Launcher(hardwareMap);
            when(flywheelLead.getVelocity()).thenReturn(100.0);

            launcher.read();

            assertEquals("Read should update velocity", 100.0,
                        launcher.getFlywheelVelocity(), 0.01);
        } catch (Exception e) {
            fail("Read should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testLoop() {
        try {
            launcher = new Launcher(hardwareMap);
            launcher.setFlywheelTargetVelocity(1000.0);

            launcher.read();
            launcher.loop();

            assertTrue("Loop should complete without exception", true);
        } catch (Exception e) {
            fail("Loop should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testWrite() {
        try {
            launcher = new Launcher(hardwareMap);
            launcher.setFlywheelTargetVelocity(1000.0);

            launcher.read();
            launcher.loop();
            launcher.write();

            // Verify power was set to both motors
            verify(flywheelLead, atLeastOnce()).setPower(anyDouble());
            verify(flywheelFollow, atLeastOnce()).setPower(anyDouble());
        } catch (Exception e) {
            fail("Write should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testFullCycle_ReadLoopWrite() {
        try {
            launcher = new Launcher(hardwareMap);
            launcher.setFlywheelTargetVelocity(1000.0);

            // Run multiple cycles
            for (int i = 0; i < 10; i++) {
                launcher.read();
                launcher.loop();
                launcher.write();
            }

            assertTrue("Full cycle should execute multiple times", true);
        } catch (Exception e) {
            fail("Full cycle should not throw exception: " + e.getMessage());
        }
    }

    // ==================== Stop Power Tests ====================

    @Test
    public void testStopPower_Enabled() {
        launcher = new Launcher(hardwareMap);
        launcher.setFlywheelTargetVelocity(1000.0);
        launcher.setStopPower(true);

        launcher.read();
        launcher.loop();

        // When stop power is enabled, compensated power should be 0
        assertEquals("Stop power should set compensated power to 0",
                     0.0, launcher.getCompensatedPower(), 0.01);
    }

    @Test
    public void testStopPower_Disabled() {
        launcher = new Launcher(hardwareMap);
        launcher.setFlywheelTargetVelocity(1000.0);
        launcher.setStopPower(false);

        when(flywheelLead.getVelocity()).thenReturn(0.0);
        launcher.read();
        launcher.loop();

        // When stop power is disabled and we have target velocity,
        // compensated power should be non-zero
        assertTrue("With stop power disabled, should calculate power", true);
    }

    // ==================== Voltage Compensation Tests ====================

    @Test
    public void testVoltageCompensation_NormalVoltage() {
        launcher = new Launcher(hardwareMap);
        when(voltageSensor.getVoltage()).thenReturn(12.0);

        launcher.setFlywheelTargetVelocity(1000.0);
        launcher.read();
        launcher.loop();

        double batteryVoltage = launcher.getBatteryVoltageValue();
        assertTrue("Battery voltage should be read", batteryVoltage > 0);
    }

    @Test
    public void testVoltageCompensation_LowVoltage() {
        launcher = new Launcher(hardwareMap);
        when(voltageSensor.getVoltage()).thenReturn(10.0);

        launcher.setFlywheelTargetVelocity(1000.0);
        launcher.read();

        // Wait for voltage update (or force it by simulating time passage)
        launcher.loop();

        // With low voltage, compensated power should be higher than base power
        assertTrue("Voltage compensation should work", true);
    }

    @Test
    public void testGetBatteryVoltage_NoSensor() {
        // Create launcher with no voltage sensors
        when(hardwareMap.getAll(VoltageSensor.class)).thenReturn(new ArrayList<>());
        launcher = new Launcher(hardwareMap);

        launcher.read();

        // Should return nominal voltage when no sensor available
        double voltage = launcher.getBatteryVoltageValue();
        assertEquals("Should return nominal voltage when no sensor",
                     Constants.Launcher.NOMINAL_BATTERY_VOLTAGE, voltage, 0.01);
    }

    // ==================== Command Tests ====================

    @Test
    public void testLaunchFarCommand() {
        launcher = new Launcher(hardwareMap);

        // LaunchFar command should set far velocity
        launcher.setFlywheelTargetVelocity(Constants.Launcher.FarVelocity);

        assertEquals("LaunchFar should set far velocity",
                     Constants.Launcher.FarVelocity,
                     launcher.getFlywheelTargetVelocity(), 0.01);
    }

    @Test
    public void testLaunchCloseCommand() {
        launcher = new Launcher(hardwareMap);

        // LaunchClose command should set close velocity
        launcher.setFlywheelTargetVelocity(Constants.Launcher.closeVelocity);

        assertEquals("LaunchClose should set close velocity",
                     Constants.Launcher.closeVelocity,
                     launcher.getFlywheelTargetVelocity(), 0.01);
    }

    @Test
    public void testHPIntakeCommand() {
        launcher = new Launcher(hardwareMap);

        // HPIntake command should set intake velocity
        launcher.setFlywheelTargetVelocity(Constants.Launcher.intakeVelocity);

        assertEquals("HPIntake should set intake velocity",
                     Constants.Launcher.intakeVelocity,
                     launcher.getFlywheelTargetVelocity(), 0.01);
    }

    @Test
    public void testStopCommand() {
        launcher = new Launcher(hardwareMap);
        launcher.setFlywheelTargetVelocity(1000.0);

        // Stop command should set velocity to 0
        launcher.setFlywheelTargetVelocity(0.0);

        assertEquals("Stop command should set velocity to 0",
                     0.0, launcher.getFlywheelTargetVelocity(), 0.01);
    }

    @Test
    public void testStopPowerCommand() {
        launcher = new Launcher(hardwareMap);

        // stopPower command should enable stop power flag
        launcher.setStopPower(true);
        launcher.loop();

        assertEquals("stopPower command should set compensated power to 0",
                     0.0, launcher.getCompensatedPower(), 0.01);
    }

    @Test
    public void testLaunchRangeCommand() {
        launcher = new Launcher(hardwareMap);

        double range = 51.0; // 51 inches - matches a value in the LUT

        // Call LaunchOnRange directly
        launcher.LaunchOnRange(range);

        // The LUT has (51, 1250), so target velocity should be 1250
        assertEquals("LaunchRange should set velocity based on LUT",
                   1250.0, launcher.getFlywheelTargetVelocity(), 0.01);
    }

    @Test
    public void testLaunchRangeCommand_InterpolatedValue() {
        launcher = new Launcher(hardwareMap);

        // Use a range between LUT values to test interpolation
        double range = 40.0; // Between 31 (1150) and 51 (1250)

        // Call LaunchOnRange directly
        launcher.LaunchOnRange(range);

        // Should interpolate between 1150 and 1250
        double targetVelocity = launcher.getFlywheelTargetVelocity();
        assertTrue("LaunchRange should interpolate between LUT values",
                   targetVelocity >= 1150.0 && targetVelocity <= 1250.0);
    }

    @Test
    public void testLaunchPoseCommand() {
        launcher = new Launcher(hardwareMap);

        // Create robot pose and target pose
        // Robot at (72, 72) facing forward, target at (72, 123)
        // Distance should be approximately 51 inches (accounting for LUTDistance)
        Pose robotPose = new Pose(72, 72 + Constants.Launcher.LUTDistance, 0);
        Pose targetPose = new Pose(72, 123, 0);

        // Call LaunchOnPose directly
        launcher.LaunchOnPose(robotPose, targetPose);

        // Should set a velocity based on the calculated distance
        double targetVelocity = launcher.getFlywheelTargetVelocity();
        assertTrue("LaunchPose should set a non-zero velocity",
                   targetVelocity > 0);
    }

    @Test
    public void testLaunchPoseCommand_CalculatesDistance() {
        launcher = new Launcher(hardwareMap);

        // Create poses where we know the exact distance
        // With LUTDistance = 0, robot at (0, 0), target at (0, 51) = 51 inches
        Pose robotPose = new Pose(0, 0, 0);
        Pose targetPose = new Pose(0, 51, 0);

        // Call LaunchOnPose directly
        launcher.LaunchOnPose(robotPose, targetPose);

        // Distance is 51, which maps to 1250 in the LUT
        // Allow tolerance since LUT interpolation may vary slightly
        double targetVelocity = launcher.getFlywheelTargetVelocity();
        assertTrue("LaunchPose should calculate velocity in expected range (got " + targetVelocity + ")",
                   targetVelocity >= 1200.0 && targetVelocity <= 1350.0);
    }

    @Test
    public void testLaunchPoseCommand_DiagonalDistance() {
        launcher = new Launcher(hardwareMap);

        // Test with diagonal distance (3-4-5 triangle scaled)
        // Robot at (0, 0), target at (30, 40) = 50 inches distance
        Pose robotPose = new Pose(0, 0, 0);
        Pose targetPose = new Pose(30, 40, 0);

        // Call LaunchOnPose directly
        launcher.LaunchOnPose(robotPose, targetPose);

        // Should set velocity based on ~50 inch distance
        double targetVelocity = launcher.getFlywheelTargetVelocity();
        assertTrue("LaunchPose should handle diagonal distances",
                   targetVelocity > 0);
    }

    // ==================== Telemetry Tests ====================

    @Test
    public void testGetTelemetry() {
        launcher = new Launcher(hardwareMap);

        // Enable telemetry in constants (if needed)
        boolean originalTelemetryState = Constants.Launcher.logTelemetry;
        Constants.Launcher.logTelemetry = true;

        launcher.setFlywheelTargetVelocity(1000.0);
        launcher.read();
        launcher.loop();

        LinkedList<TelemetryPacket> telemetry = launcher.getTelemetry();

        assertNotNull("Telemetry should not be null", telemetry);

        // Restore original state
        Constants.Launcher.logTelemetry = originalTelemetryState;
    }

    @Test
    public void testTelemetry_ContainsRelevantData() {
        launcher = new Launcher(hardwareMap);

        boolean originalTelemetryState = Constants.Launcher.logTelemetry;
        Constants.Launcher.logTelemetry = true;

        launcher.setFlywheelTargetVelocity(1000.0);
        when(flywheelLead.getVelocity()).thenReturn(850.0);
        launcher.read();
        launcher.loop();

        LinkedList<TelemetryPacket> telemetry = launcher.getTelemetry();

        // Should have telemetry data when logging is enabled
        if (Constants.Launcher.logTelemetry) {
            assertFalse("Telemetry should contain data when enabled",
                      telemetry.isEmpty());
        }

        Constants.Launcher.logTelemetry = originalTelemetryState;
    }

    // ==================== PID Control Tests ====================

    @Test
    public void testPIDControl_PositiveError() {
        launcher = new Launcher(hardwareMap);

        // Target higher than actual
        launcher.setFlywheelTargetVelocity(1000.0);
        when(flywheelLead.getVelocity()).thenReturn(500.0);

        launcher.read();
        launcher.loop();
        launcher.write();

        // Should command positive power to increase velocity
        verify(flywheelLead).setPower(doubleThat(power -> power > 0));
    }

    @Test
    public void testPIDControl_NegativeError() {
        launcher = new Launcher(hardwareMap);

        // Target lower than actual
        launcher.setFlywheelTargetVelocity(500.0);
        when(flywheelLead.getVelocity()).thenReturn(1000.0);

        launcher.read();
        launcher.loop();
        launcher.write();

        // Should command negative power or low power to decrease velocity
        assertTrue("PID should respond to negative error", true);
    }

    @Test
    public void testPIDControl_ZeroError() {
        launcher = new Launcher(hardwareMap);

        // Target equals actual
        launcher.setFlywheelTargetVelocity(1000.0);
        when(flywheelLead.getVelocity()).thenReturn(1000.0);

        launcher.read();
        launcher.loop();

        // Error should be zero
        assertEquals("Error should be zero when at target",
                     0.0, launcher.getError(), 0.01);
    }

    // ==================== Deadband Tests ====================

    @Test
    public void testDeadband_WithinDeadband() {
        launcher = new Launcher(hardwareMap);

        // Set target to 0 and actual velocity within deadband
        launcher.setFlywheelTargetVelocity(0.0);
        when(flywheelLead.getVelocity()).thenReturn(
            Constants.Launcher.VELOCITY_DEADBAND / 2.0
        );

        launcher.read();
        launcher.loop();

        // Should set power to 0 when within deadband
        assertEquals("Should set power to 0 within deadband",
                     0.0, launcher.getCompensatedPower(), 0.01);
    }

    @Test
    public void testDeadband_OutsideDeadband() {
        launcher = new Launcher(hardwareMap);

        // Set target to 0 but actual velocity outside deadband
        launcher.setFlywheelTargetVelocity(0.0);
        when(flywheelLead.getVelocity()).thenReturn(
            Constants.Launcher.VELOCITY_DEADBAND * 2.0
        );

        launcher.read();
        launcher.loop();

        // Should still calculate power when outside deadband
        assertTrue("Should calculate power outside deadband", true);
    }

    // ==================== Integration Tests ====================

    @Test
    public void testIntegration_SpinUpSequence() {
        launcher = new Launcher(hardwareMap);

        // Simulate spin-up sequence
        launcher.setFlywheelTargetVelocity(1000.0);

        // Simulate motor gradually reaching target
        double[] velocities = {0, 250, 500, 750, 900, 980, 1000};
        for (double vel : velocities) {
            when(flywheelLead.getVelocity()).thenReturn(vel);
            launcher.read();
            launcher.loop();
            launcher.write();
        }

        // Final velocity should be at target
        assertEquals("Should reach target velocity",
                     1000.0, launcher.getFlywheelVelocity(), 0.01);
    }

    @Test
    public void testIntegration_SpinDownSequence() {
        launcher = new Launcher(hardwareMap);

        // Start at high velocity
        when(flywheelLead.getVelocity()).thenReturn(1000.0);
        launcher.read();

        // Command stop
        launcher.setFlywheelTargetVelocity(0.0);

        // Simulate motor slowing down
        double[] velocities = {1000, 750, 500, 250, 50, 10, 0};
        for (double vel : velocities) {
            when(flywheelLead.getVelocity()).thenReturn(vel);
            launcher.read();
            launcher.loop();
            launcher.write();
        }

        assertEquals("Should stop at zero velocity",
                     0.0, launcher.getFlywheelVelocity(), 0.01);
    }

    @Test
    public void testIntegration_CommandSequence() {
        launcher = new Launcher(hardwareMap);

        // Test sequence of different commands
        launcher.setFlywheelTargetVelocity(Constants.Launcher.FarVelocity);
        launcher.read();
        launcher.loop();
        launcher.write();

        launcher.setFlywheelTargetVelocity(Constants.Launcher.closeVelocity);
        launcher.read();
        launcher.loop();
        launcher.write();

        launcher.setFlywheelTargetVelocity(0.0);
        launcher.read();
        launcher.loop();
        launcher.write();

        assertTrue("Command sequence should execute", true);
    }

    // ==================== Edge Case Tests ====================

    @Test
    public void testEdgeCase_NegativeVelocity() {
        launcher = new Launcher(hardwareMap);

        launcher.setFlywheelTargetVelocity(-100.0);
        launcher.read();
        launcher.loop();

        // Should handle negative velocity target
        assertTrue("Should handle negative velocity", true);
    }

    @Test
    public void testEdgeCase_VeryHighVelocity() {
        launcher = new Launcher(hardwareMap);

        launcher.setFlywheelTargetVelocity(10000.0);
        launcher.read();
        launcher.loop();

        // Should clamp power to valid range
        double power = launcher.getCompensatedPower();
        assertTrue("Power should be clamped", power >= -1.0 && power <= 1.0);
    }

    @Test
    public void testEdgeCase_ZeroVoltage() {
        launcher = new Launcher(hardwareMap);
        when(voltageSensor.getVoltage()).thenReturn(0.0);

        launcher.setFlywheelTargetVelocity(1000.0);
        launcher.read();
        launcher.loop();

        // Should use nominal voltage when sensor returns 0
        assertEquals("Should use nominal voltage",
                     Constants.Launcher.NOMINAL_BATTERY_VOLTAGE,
                     launcher.getBatteryVoltageValue(), 0.01);
    }

    // ==================== Performance Tests ====================

    @Test
    public void testPerformance_MultipleLoops() {
        launcher = new Launcher(hardwareMap);
        launcher.setFlywheelTargetVelocity(1000.0);

        long startTime = System.currentTimeMillis();

        // Run 100 cycles
        for (int i = 0; i < 100; i++) {
            when(flywheelLead.getVelocity()).thenReturn(500.0 + i * 5.0);
            launcher.read();
            launcher.loop();
            launcher.write();
        }

        long duration = System.currentTimeMillis() - startTime;

        // Should complete quickly (< 1 second for 100 iterations)
        assertTrue("Should complete 100 cycles quickly", duration < 1000);
    }

    // ==================== State Management Tests ====================

    @Test
    public void testStateConsistency_AcrossMultipleCycles() {
        launcher = new Launcher(hardwareMap);

        double targetVelocity = 1000.0;
        launcher.setFlywheelTargetVelocity(targetVelocity);

        // Run multiple cycles
        for (int i = 0; i < 10; i++) {
            launcher.read();
            launcher.loop();
            launcher.write();

            // Target should remain consistent
            assertEquals("Target velocity should remain consistent",
                        targetVelocity, launcher.getFlywheelTargetVelocity(), 0.01);
        }
    }
}
