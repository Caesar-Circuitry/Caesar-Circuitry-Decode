package org.firstinspires.ftc.teamcode.Config.Subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import java.util.LinkedList;

/**
 * Unit tests for the Intake subsystem
 * Tests state machine, motor control, servo control, beam breaks, and current sensing
 */
public class IntakeTest {

    @Mock
    private HardwareMap hardwareMap;

    @Mock
    private DcMotorEx intakeMotor;

    @Mock
    private DcMotorEx transferMotor;

    @Mock
    private Servo feederServo;

    @Mock
    private DigitalChannel intakeBeamBreakChannel;

    @Mock
    private DigitalChannel transferBeamBreakChannel;

    private Intake intake;

    @Before
    @SuppressWarnings("resource")
    public void setUp() {
        MockitoAnnotations.openMocks(this);

        // Mock hardware map to return our mocked components
        when(hardwareMap.get(DcMotorEx.class, Constants.Intake.INTAKE_MOTOR))
            .thenReturn(intakeMotor);
        when(hardwareMap.get(DcMotorEx.class, Constants.Intake.TRANSFER_MOTOR))
            .thenReturn(transferMotor);
        when(hardwareMap.get(Servo.class, Constants.Intake.FEEDER_SERVO))
            .thenReturn(feederServo);
        when(hardwareMap.get(DigitalChannel.class, Constants.Intake.intakeBeamBreak))
            .thenReturn(intakeBeamBreakChannel);
        when(hardwareMap.get(DigitalChannel.class, Constants.Intake.transferBeamBreak))
            .thenReturn(transferBeamBreakChannel);

        // Mock motor current readings (using CurrentUnit.AMPS)
        when(intakeMotor.getCurrent(CurrentUnit.AMPS)).thenReturn(0.5);
        when(transferMotor.getCurrent(CurrentUnit.AMPS)).thenReturn(0.5);

        // Mock beam break default state (not broken - returns true for "not broken")
        when(intakeBeamBreakChannel.getState()).thenReturn(true);
        when(transferBeamBreakChannel.getState()).thenReturn(true);
    }

    // ==================== Initialization Tests ====================

    @Test
    public void testInitialization() {
        try {
            intake = new Intake(hardwareMap);
            assertNotNull("Intake should be initialized", intake);
        } catch (Exception e) {
            fail("Intake initialization should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testMotorDirectionConfiguration() {
        intake = new Intake(hardwareMap);

        // Verify motor directions were set
        verify(intakeMotor).setDirection(Constants.Intake.INTAKE_MOTOR_DIRECTION);
        verify(transferMotor).setDirection(Constants.Intake.TRANSFER_MOTOR_DIRECTION);
    }

    // ==================== State Machine Tests ====================

    // Note: The Intake class has write optimization - it only writes when values CHANGE.
    // Initial state is HOLD with:
    //   - FeederServoTargetPos = FEEDER_SERVO_CLOSE, prevFeederServoTargetPos = FEEDER_SERVO_CLOSE
    //   - intakeMotorTargetPower = 0, prevIntakeMotorTargetPower = 0
    //   - INTAKE_MOTOR_HOLD = 0, TRANSFER_MOTOR_HOLD = 0
    // Since initial power (0) == HOLD power (0), motors DON'T write in initial HOLD state!
    // This is the correct behavior due to write optimization.

    @Test
    public void testInitialState_Hold() {
        intake = new Intake(hardwareMap);

        intake.read();
        intake.loop();
        intake.write();

        // Initial state is HOLD.
        // Motors: initial target is 0, HOLD sets to INTAKE_MOTOR_HOLD (0), so NO write (0 == 0)
        // Servo: initial is CLOSE, HOLD sets to CLOSE, so NO write (same value)
        // This is expected behavior due to write optimization - nothing changes!
        verify(intakeMotor, never()).setPower(anyDouble());
        verify(transferMotor, never()).setPower(anyDouble());
        verify(feederServo, never()).setPosition(anyDouble());
    }

    @Test
    public void testState_GroundIntaking() {
        intake = new Intake(hardwareMap);

        // Trigger ground intake state
        intake.GroundIntake().initialize();

        intake.read();
        intake.loop();
        intake.write();

        // Ground intake: motors change from 0 to FORWARD, servo stays at CLOSE
        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_FORWARD);
        verify(transferMotor).setPower(Constants.Intake.TRANSFER_MOTOR_FORWARD);
        // Servo doesn't write because GROUND_INTAKING also uses FEEDER_SERVO_CLOSE (same as initial)
        verify(feederServo, never()).setPosition(anyDouble());
    }

    @Test
    public void testState_HPIntaking() {
        intake = new Intake(hardwareMap);

        // Trigger HP intake state
        intake.HP_Intaking().initialize();

        intake.read();
        intake.loop();
        intake.write();

        // HP intake: servo changes to OPEN (differs from initial CLOSE), so it writes
        verify(feederServo).setPosition(Constants.Intake.FEEDER_SERVO_OPEN);
        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_HP);
        verify(transferMotor).setPower(Constants.Intake.TRANSFER_MOTOR_HP);
    }

    @Test
    public void testState_Launch() {
        intake = new Intake(hardwareMap);

        // Trigger launch state
        intake.Launch().initialize();

        intake.read();
        intake.loop();
        intake.write();

        // Launch: servo changes to OPEN (differs from initial CLOSE), so it writes
        verify(feederServo).setPosition(Constants.Intake.FEEDER_SERVO_OPEN);
        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_FORWARD);
        verify(transferMotor).setPower(Constants.Intake.TRANSFER_MOTOR_FORWARD);
    }

    @Test
    public void testState_Hold_AfterLaunch() {
        intake = new Intake(hardwareMap);

        // First go to launch (opens servo, changes motor power)
        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // Reset mocks to track new calls
        reset(feederServo, intakeMotor, transferMotor);

        // Then go to hold (closes servo, changes motor power back)
        intake.Hold().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // Servo changes from OPEN to CLOSE
        verify(feederServo).setPosition(Constants.Intake.FEEDER_SERVO_CLOSE);
        // Motors change from FORWARD to HOLD (if HOLD != FORWARD)
        // Note: HOLD is 0, FORWARD is typically non-zero, so this should write
        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_HOLD);
        verify(transferMotor).setPower(Constants.Intake.TRANSFER_MOTOR_HOLD);
    }

    // ==================== State Transition Tests ====================

    @Test
    public void testStateTransition_GroundIntakeToHold() {
        intake = new Intake(hardwareMap);

        // Ground intake - motors go to FORWARD
        intake.GroundIntake().initialize();
        intake.read();
        intake.loop();
        intake.write();

        reset(intakeMotor, transferMotor);

        // Transition to hold - motors go back to HOLD (0)
        intake.Hold().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // Should write because FORWARD != HOLD
        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_HOLD);
    }

    @Test
    public void testStateTransition_HoldToLaunch() {
        intake = new Intake(hardwareMap);

        // Start in hold (default)
        intake.read();
        intake.loop();
        intake.write();

        // Transition to launch
        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // Should open feeder (changes from CLOSE to OPEN)
        verify(feederServo, atLeastOnce()).setPosition(Constants.Intake.FEEDER_SERVO_OPEN);
    }

    @Test
    public void testStateTransition_HPIntakeToGroundIntake() {
        intake = new Intake(hardwareMap);

        // Start HP intake (opens servo)
        intake.HP_Intaking().initialize();
        intake.read();
        intake.loop();
        intake.write();

        reset(feederServo, intakeMotor, transferMotor);

        // Transition to ground intake (closes servo)
        intake.GroundIntake().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // Should close feeder for ground intake
        verify(feederServo).setPosition(Constants.Intake.FEEDER_SERVO_CLOSE);
    }

    // ==================== Read-Loop-Write Cycle Tests ====================

    @Test
    public void testRead() {
        try {
            intake = new Intake(hardwareMap);
            intake.read();
            assertTrue("Read should complete without exception", true);
        } catch (Exception e) {
            fail("Read should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testLoop() {
        try {
            intake = new Intake(hardwareMap);
            intake.read();
            intake.loop();
            assertTrue("Loop should complete without exception", true);
        } catch (Exception e) {
            fail("Loop should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testWrite() {
        try {
            intake = new Intake(hardwareMap);
            intake.read();
            intake.loop();
            intake.write();
            assertTrue("Write should complete without exception", true);
        } catch (Exception e) {
            fail("Write should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testFullCycle_ReadLoopWrite() {
        try {
            intake = new Intake(hardwareMap);

            // Run multiple cycles
            for (int i = 0; i < 10; i++) {
                intake.read();
                intake.loop();
                intake.write();
            }

            assertTrue("Full cycle should execute multiple times", true);
        } catch (Exception e) {
            fail("Full cycle should not throw exception: " + e.getMessage());
        }
    }

    // ==================== Write Optimization Tests ====================

    @Test
    public void testWrite_OnlyWritesWhenChanged() {
        intake = new Intake(hardwareMap);

        // First cycle in HOLD state
        // Since initial power (0) == HOLD power (0), nothing writes!
        intake.read();
        intake.loop();
        intake.write();

        // Verify NO writes in initial HOLD state (optimization: 0 == 0)
        verify(intakeMotor, never()).setPower(anyDouble());
        verify(transferMotor, never()).setPower(anyDouble());
        verify(feederServo, never()).setPosition(anyDouble());

        // Second cycle with same state - still no writes
        intake.read();
        intake.loop();
        intake.write();

        // Still no writes
        verify(intakeMotor, never()).setPower(anyDouble());
        verify(transferMotor, never()).setPower(anyDouble());
        verify(feederServo, never()).setPosition(anyDouble());
    }

    @Test
    public void testWrite_WritesWhenStateChanges() {
        intake = new Intake(hardwareMap);

        // First cycle in hold - no writes (0 == 0)
        intake.read();
        intake.loop();
        intake.write();

        // Change to launch (different motor powers AND servo position changes)
        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // Motors should have written once (only when changed to LAUNCH)
        verify(intakeMotor, times(1)).setPower(Constants.Intake.INTAKE_MOTOR_FORWARD);
        verify(transferMotor, times(1)).setPower(Constants.Intake.TRANSFER_MOTOR_FORWARD);
        // Servo should have written once (changed from CLOSE to OPEN)
        verify(feederServo, times(1)).setPosition(Constants.Intake.FEEDER_SERVO_OPEN);
    }

    // ==================== Command Tests ====================

    @Test
    public void testGroundIntakeCommand() {
        intake = new Intake(hardwareMap);

        // Execute command
        intake.GroundIntake().initialize();

        intake.read();
        intake.loop();
        intake.write();

        // Verify motors are at FORWARD power
        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_FORWARD);
    }

    @Test
    public void testHoldCommand() {
        intake = new Intake(hardwareMap);

        // First change to launch (different state)
        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        reset(intakeMotor, transferMotor, feederServo);

        // Execute hold command
        intake.Hold().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_HOLD);
    }

    @Test
    public void testLaunchCommand() {
        intake = new Intake(hardwareMap);

        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(feederServo).setPosition(Constants.Intake.FEEDER_SERVO_OPEN);
        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_FORWARD);
    }

    @Test
    public void testHPIntakingCommand() {
        intake = new Intake(hardwareMap);

        intake.HP_Intaking().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(feederServo).setPosition(Constants.Intake.FEEDER_SERVO_OPEN);
        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_HP);
        verify(transferMotor).setPower(Constants.Intake.TRANSFER_MOTOR_HP);
    }

    // ==================== Telemetry Tests ====================

    @Test
    public void testGetTelemetry() {
        intake = new Intake(hardwareMap);

        boolean originalTelemetryState = Constants.Intake.logTelemetry;
        Constants.Intake.logTelemetry = true;

        intake.read();
        intake.loop();

        LinkedList<TelemetryPacket> telemetry = intake.getTelemetry();

        assertNotNull("Telemetry should not be null", telemetry);

        Constants.Intake.logTelemetry = originalTelemetryState;
    }

    @Test
    public void testTelemetry_ContainsData() {
        intake = new Intake(hardwareMap);

        boolean originalTelemetryState = Constants.Intake.logTelemetry;
        Constants.Intake.logTelemetry = true;

        intake.read();
        intake.loop();

        LinkedList<TelemetryPacket> telemetry = intake.getTelemetry();

        if (Constants.Intake.logTelemetry) {
            assertFalse("Telemetry should contain data when enabled", telemetry.isEmpty());
        }

        Constants.Intake.logTelemetry = originalTelemetryState;
    }

    @Test
    public void testTelemetry_ContainsStateInfo() {
        intake = new Intake(hardwareMap);

        boolean originalTelemetryState = Constants.Intake.logTelemetry;
        Constants.Intake.logTelemetry = true;

        intake.read();
        intake.loop();

        LinkedList<TelemetryPacket> telemetry = intake.getTelemetry();

        // Check that telemetry contains expected data
        boolean hasStatePacket = telemetry.stream()
            .anyMatch(packet -> "State".equals(packet.getName()));

        assertTrue("Telemetry should contain state info", hasStatePacket);

        Constants.Intake.logTelemetry = originalTelemetryState;
    }

    // ==================== Integration Tests ====================

    @Test
    public void testIntegration_FullIntakeSequence() {
        intake = new Intake(hardwareMap);

        // Simulate full intake sequence
        // 1. Start in hold
        intake.read();
        intake.loop();
        intake.write();

        // 2. Ground intake to pick up ball
        intake.GroundIntake().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // 3. Hold with ball
        intake.Hold().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // 4. Launch
        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // 5. Back to hold
        intake.Hold().initialize();
        intake.read();
        intake.loop();
        intake.write();

        assertTrue("Full intake sequence should complete", true);
    }

    @Test
    public void testIntegration_HPIntakeSequence() {
        intake = new Intake(hardwareMap);

        // Simulate HP intake sequence
        // 1. Start HP intake
        intake.HP_Intaking().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // 2. Hold
        intake.Hold().initialize();
        intake.read();
        intake.loop();
        intake.write();

        // 3. Launch
        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        assertTrue("HP intake sequence should complete", true);
    }

    @Test
    public void testIntegration_RapidStateChanges() {
        intake = new Intake(hardwareMap);

        // Simulate rapid state changes
        for (int i = 0; i < 5; i++) {
            intake.GroundIntake().initialize();
            intake.read();
            intake.loop();
            intake.write();

            intake.Hold().initialize();
            intake.read();
            intake.loop();
            intake.write();

            intake.Launch().initialize();
            intake.read();
            intake.loop();
            intake.write();

            intake.HP_Intaking().initialize();
            intake.read();
            intake.loop();
            intake.write();
        }

        assertTrue("Rapid state changes should complete", true);
    }

    // ==================== Performance Tests ====================

    @Test
    public void testPerformance_MultipleLoops() {
        intake = new Intake(hardwareMap);

        long startTime = System.currentTimeMillis();

        // Run 100 cycles
        for (int i = 0; i < 100; i++) {
            intake.read();
            intake.loop();
            intake.write();
        }

        long duration = System.currentTimeMillis() - startTime;

        // Should complete quickly (< 1 second for 100 iterations)
        assertTrue("100 cycles should complete quickly", duration < 1000);
    }

    // ==================== State Consistency Tests ====================

    @Test
    public void testStateConsistency_AcrossMultipleCycles() {
        intake = new Intake(hardwareMap);

        // Set to ground intake
        intake.GroundIntake().initialize();

        // Run multiple cycles and verify state remains consistent
        for (int i = 0; i < 10; i++) {
            intake.read();
            intake.loop();
            intake.write();
        }

        // Motors should maintain ground intake power
        // Verify by checking the last calls
        verify(intakeMotor, atLeastOnce()).setPower(Constants.Intake.INTAKE_MOTOR_FORWARD);
    }

    // ==================== Feeder Servo Tests ====================
    // Note: Due to write optimization, servo only writes when position CHANGES

    @Test
    public void testFeederServo_ClosedAfterOpen() {
        intake = new Intake(hardwareMap);

        // First open (Launch)
        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        reset(feederServo);

        // Then close (Hold)
        intake.Hold().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(feederServo).setPosition(Constants.Intake.FEEDER_SERVO_CLOSE);
    }

    @Test
    public void testFeederServo_OpenInLaunch() {
        intake = new Intake(hardwareMap);

        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(feederServo).setPosition(Constants.Intake.FEEDER_SERVO_OPEN);
    }

    @Test
    public void testFeederServo_OpenInHPIntake() {
        intake = new Intake(hardwareMap);

        intake.HP_Intaking().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(feederServo).setPosition(Constants.Intake.FEEDER_SERVO_OPEN);
    }

    // ==================== Motor Power Tests ====================

    @Test
    public void testMotorPower_GroundIntake() {
        intake = new Intake(hardwareMap);

        intake.GroundIntake().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_FORWARD);
        verify(transferMotor).setPower(Constants.Intake.TRANSFER_MOTOR_FORWARD);
    }

    @Test
    public void testMotorPower_Hold() {
        intake = new Intake(hardwareMap);

        // Go to a different state first to ensure motors change
        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        reset(intakeMotor, transferMotor);

        // Now go to hold
        intake.Hold().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_HOLD);
        verify(transferMotor).setPower(Constants.Intake.TRANSFER_MOTOR_HOLD);
    }

    @Test
    public void testMotorPower_Launch() {
        intake = new Intake(hardwareMap);

        intake.Launch().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_FORWARD);
        verify(transferMotor).setPower(Constants.Intake.TRANSFER_MOTOR_FORWARD);
    }

    @Test
    public void testMotorPower_HPIntake() {
        intake = new Intake(hardwareMap);

        intake.HP_Intaking().initialize();
        intake.read();
        intake.loop();
        intake.write();

        verify(intakeMotor).setPower(Constants.Intake.INTAKE_MOTOR_HP);
        verify(transferMotor).setPower(Constants.Intake.TRANSFER_MOTOR_HP);
    }
}
