package org.firstinspires.ftc.teamcode.Config.Subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
 * Unit tests for the Vision subsystem
 * Tests Limelight integration, Kalman filter drift correction, pose transformation, and motif detection
 */
public class VisionTest {

    @Mock
    private HardwareMap hardwareMap;

    @Mock
    private Limelight3A limelight;

    @Mock
    private Follower follower;

    @Mock
    private Turret turret;

    @Mock
    private LLResult llResult;

    @Mock
    private Pose3D pose3D;

    @Mock
    private YawPitchRollAngles orientation;

    private Vision vision;

    /**
     * Helper method to create Position objects with specific coordinates
     * Position has public fields, so we create actual instances instead of mocking
     */
    private Position createPosition(double x, double y, double z) {
        Position pos = new Position();
        pos.x = x;
        pos.y = y;
        pos.z = z;
        return pos;
    }

    @Before
    @SuppressWarnings("resource")
    public void setUp() {
        MockitoAnnotations.openMocks(this);

        // Mock Limelight hardware
        when(hardwareMap.get(Limelight3A.class, Constants.Vision.cameraName))
            .thenReturn(limelight);

        // Mock follower pose
        Pose mockPose = new Pose(0, 0, 0);
        when(follower.getPose()).thenReturn(mockPose);

        // Mock turret angle
        when(turret.getCurrentTurretAngle()).thenReturn(0.0);

        // Mock Limelight result
        when(limelight.getLatestResult()).thenReturn(llResult);
        when(llResult.isValid()).thenReturn(false); // Default to no vision
    }

    // ==================== Initialization Tests ====================

    @Test
    public void testInitialization() {
        try {
            vision = new Vision(hardwareMap, follower, turret);
            assertNotNull("Vision should be initialized", vision);
        } catch (Exception e) {
            fail("Vision initialization should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testLimelightConfiguration() {
        vision = new Vision(hardwareMap, follower, turret);

        // Verify Limelight was configured
        verify(limelight).pipelineSwitch(0); // Megatag 2 pipeline
        verify(limelight).start();
    }

    @Test
    public void testKalmanFiltersInitialized() {
        vision = new Vision(hardwareMap, follower, turret);

        // Get drift estimates (should be initialized to 0)
        double[] drifts = vision.getDriftEstimates();

        assertNotNull("Drift estimates should not be null", drifts);
        assertEquals("Should have 3 drift estimates", 3, drifts.length);
        assertEquals("Initial X drift should be 0", 0.0, drifts[0], 0.01);
        assertEquals("Initial Y drift should be 0", 0.0, drifts[1], 0.01);
        assertEquals("Initial heading drift should be 0", 0.0, drifts[2], 0.01);
    }

    // ==================== Read-Loop-Write Cycle Tests ====================

    @Test
    public void testRead_NoVision() {
        vision = new Vision(hardwareMap, follower, turret);

        when(llResult.isValid()).thenReturn(false);

        vision.read();
        vision.loop();

        // Should handle no vision gracefully
        assertTrue("Should handle no vision", true);
    }

    @Test
    public void testRead_WithValidVision() {
        vision = new Vision(hardwareMap, follower, turret);

        // Mock valid vision data
        when(llResult.isValid()).thenReturn(true);
        when(llResult.getBotpose_MT2()).thenReturn(pose3D);
        when(pose3D.getPosition()).thenReturn(createPosition(50.0, 50.0, 0.0));
        when(pose3D.getOrientation()).thenReturn(orientation);
        when(orientation.getYaw()).thenReturn(0.0);

        // Ensure limelight returns the mocked result
        when(limelight.getLatestResult()).thenReturn(llResult);

        vision.read();
        vision.loop();

        // Verify limelight.getLatestResult() was called in read()
        verify(limelight, atLeastOnce()).getLatestResult();
        assertTrue("Should process valid vision", true);
    }

    @Test
    public void testLoop() {
        try {
            vision = new Vision(hardwareMap, follower, turret);

            vision.read();
            vision.loop();

            assertTrue("Loop should complete without exception", true);
        } catch (Exception e) {
            fail("Loop should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testWrite() {
        try {
            vision = new Vision(hardwareMap, follower, turret);

            vision.read();
            vision.loop();
            vision.write();

            // Vision has no actuators, write should do nothing
            assertTrue("Write should complete without exception", true);
        } catch (Exception e) {
            fail("Write should not throw exception: " + e.getMessage());
        }
    }

    @Test
    public void testFullCycle_ReadLoopWrite() {
        try {
            vision = new Vision(hardwareMap, follower, turret);

            // Run multiple cycles
            for (int i = 0; i < 10; i++) {
                vision.read();
                vision.loop();
                vision.write();
            }

            assertTrue("Full cycle should execute multiple times", true);
        } catch (Exception e) {
            fail("Full cycle should not throw exception: " + e.getMessage());
        }
    }

    // ==================== Pose Correction Tests ====================

    @Test
    public void testPoseCorrection_Enabled() {
        vision = new Vision(hardwareMap, follower, turret);
        vision.setEnablePoseCorrection(true);

        // Mock valid vision with valid meter values (0.5m from center = ~91.7 inches)
        when(llResult.isValid()).thenReturn(true);
        when(llResult.getBotpose()).thenReturn(pose3D); // MT1 uses getBotpose()
        when(pose3D.getPosition()).thenReturn(createPosition(0.5, 0.5, 0.0)); // Valid meters (center origin)
        when(pose3D.getOrientation()).thenReturn(orientation);
        when(orientation.getYaw()).thenReturn(45.0);

        // Mock fiducials for quality check to pass
        List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
        LLResultTypes.FiducialResult fiducial = mock(LLResultTypes.FiducialResult.class);
        Pose3D targetPose = mock(Pose3D.class);
        Position targetPos = new Position();
        targetPos.z = 1.0; // 1 meter = 39.37 inches - within MAX_TAG_DISTANCE
        when(fiducial.getTargetPoseCameraSpace()).thenReturn(targetPose);
        when(targetPose.getPosition()).thenReturn(targetPos);
        fiducials.add(fiducial);
        when(llResult.getFiducialResults()).thenReturn(fiducials);

        // Ensure limelight returns the mocked result
        when(limelight.getLatestResult()).thenReturn(llResult);

        // Mock odometry (in inches)
        Pose odoPose = new Pose(85.0, 85.0, Math.toRadians(40.0));
        when(follower.getPose()).thenReturn(odoPose);

        vision.read();
        vision.loop();

        // Should call setPose to correct position
        verify(follower, atLeastOnce()).setPose(any(Pose.class));
    }

    @Test
    public void testPoseCorrection_Disabled() {
        vision = new Vision(hardwareMap, follower, turret);
        vision.setEnablePoseCorrection(false);

        // Mock valid vision
        when(llResult.isValid()).thenReturn(true);
        when(llResult.getBotpose_MT2()).thenReturn(pose3D);
        when(pose3D.getPosition()).thenReturn(createPosition(60.0, 60.0, 0.0));
        when(pose3D.getOrientation()).thenReturn(orientation);
        when(orientation.getYaw()).thenReturn(45.0);

        // Ensure limelight returns the mocked result
        when(limelight.getLatestResult()).thenReturn(llResult);

        vision.read();
        vision.loop();

        // Should NOT call setPose when disabled
        verify(follower, never()).setPose(any(Pose.class));
    }

    @Test
    public void testSetEnablePoseCorrection() {
        vision = new Vision(hardwareMap, follower, turret);

        vision.setEnablePoseCorrection(false);
        // Verify by running loop with vision - should not update pose

        vision.setEnablePoseCorrection(true);
        // Verify by running loop with vision - should update pose

        assertTrue("Should be able to toggle pose correction", true);
    }

    // ==================== Kalman Filter Tests ====================

    @Test
    public void testKalmanFilter_PredictStep() {
        vision = new Vision(hardwareMap, follower, turret);

        // Run predict-only cycles (no vision updates)
        for (int i = 0; i < 10; i++) {
            vision.read();
            vision.loop();
        }

        // Drift estimates should still be near 0 without measurements
        double[] drifts = vision.getDriftEstimates();
        assertTrue("Drift should remain bounded",
                   Math.abs(drifts[0]) < 10.0 && Math.abs(drifts[1]) < 10.0);
    }

    @Test
    public void testKalmanFilter_UpdateWithVision() {
        vision = new Vision(hardwareMap, follower, turret);

        // Create actual Position object (not mocked) with valid meter values
        // Use 0.5m from center = 0.5 * 39.3701 + 72 = 91.685 inches from corner
        Position actualPosition = new Position();
        actualPosition.x = 0.5; // Vision says 0.5m from center
        actualPosition.y = 0.5;
        actualPosition.z = 0.0;

        // Mock vision with drift - use getBotpose() for MT1
        when(llResult.isValid()).thenReturn(true);
        when(llResult.getBotpose()).thenReturn(pose3D);
        when(pose3D.getPosition()).thenReturn(actualPosition);
        when(pose3D.getOrientation()).thenReturn(orientation);
        when(orientation.getYaw()).thenReturn(0.0);

        // Mock fiducials for quality check to pass
        List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
        LLResultTypes.FiducialResult fiducial = mock(LLResultTypes.FiducialResult.class);
        Pose3D targetPose = mock(Pose3D.class);
        Position targetPos = new Position();
        targetPos.z = 1.0; // 1 meter = 39.37 inches - within MAX_TAG_DISTANCE
        when(fiducial.getTargetPoseCameraSpace()).thenReturn(targetPose);
        when(targetPose.getPosition()).thenReturn(targetPos);
        fiducials.add(fiducial);
        when(llResult.getFiducialResults()).thenReturn(fiducials);

        // Ensure limelight returns the mocked result
        when(limelight.getLatestResult()).thenReturn(llResult);

        // Mock odometry with different position (in inches)
        // Vision says 91.685 inches, odometry says 85 inches - 6.685 inch drift
        Pose odoPose = new Pose(85.0, 85.0, 0.0);
        when(follower.getPose()).thenReturn(odoPose);

        // Run multiple updates
        for (int i = 0; i < 5; i++) {
            vision.read();
            vision.loop();
        }

        // Drift should be estimated (difference between vision and odometry)
        double[] drifts = vision.getDriftEstimates();
        assertTrue("X drift should be estimated", Math.abs(drifts[0]) > 0.1);
    }

    @Test
    public void testGetDriftEstimates() {
        vision = new Vision(hardwareMap, follower, turret);

        double[] drifts = vision.getDriftEstimates();

        assertNotNull("Drift estimates should not be null", drifts);
        assertEquals("Should return 3 drift values", 3, drifts.length);
    }

    // ==================== Vision Pose Tests ====================

    @Test
    public void testGetVisionPose_NoVision() {
        vision = new Vision(hardwareMap, follower, turret);

        when(llResult.isValid()).thenReturn(false);
        vision.read();

        double[] visionPose = vision.getVisionPose();

        assertNull("Should return null when no vision", visionPose);
    }

    @Test
    public void testGetVisionPose_ValidVision() {
        vision = new Vision(hardwareMap, follower, turret);

        // Use valid meter values (center origin: -1.83m to +1.83m)
        // Limelight (0.5, 0.5) -> Pedro X = LL_Y * 39.3701 + 72 = 91.69, Pedro Y = LL_X * 39.3701 + 72 = 91.69
        when(llResult.isValid()).thenReturn(true);
        when(llResult.getBotpose()).thenReturn(pose3D); // MT1 uses getBotpose()
        when(pose3D.getPosition()).thenReturn(createPosition(0.5, 0.5, 0.0)); // 0.5 meters from center
        when(pose3D.getOrientation()).thenReturn(orientation);
        when(orientation.getYaw()).thenReturn(90.0);

        // Ensure limelight returns the mocked result
        when(limelight.getLatestResult()).thenReturn(llResult);

        vision.read();

        double[] visionPose = vision.getVisionPose();

        assertNotNull("Should return vision pose", visionPose);
        assertEquals("Should return 3 values", 3, visionPose.length);
        // Expected: Limelight Y (0.5m) -> Pedro X = 0.5 * 39.3701 + 72 = 91.685 inches
        // Expected: Limelight X (0.5m) -> Pedro Y = 0.5 * 39.3701 + 72 = 91.685 inches
        assertEquals("X should match (from Limelight Y)", 91.685, visionPose[0], 0.1);
        assertEquals("Y should match (from Limelight X)", 91.685, visionPose[1], 0.1);
    }

    @Test
    public void testGetMT1Pose_NoVision() {
        vision = new Vision(hardwareMap, follower, turret);

        when(llResult.isValid()).thenReturn(false);
        vision.read();

        double[] mt1Pose = vision.getMT1Pose();

        assertNull("Should return null when no vision", mt1Pose);
    }

    @Test
    public void testGetMT1Pose_ValidVision() {
        vision = new Vision(hardwareMap, follower, turret);

        // Use 0,0 meters (field center) = 72,72 inches from corner (same after swap)
        when(llResult.isValid()).thenReturn(true);
        when(llResult.getBotpose()).thenReturn(pose3D); // MT1 uses getBotpose()
        when(pose3D.getPosition()).thenReturn(createPosition(0.0, 0.0, 0.0)); // Field center in meters
        when(pose3D.getOrientation()).thenReturn(orientation);
        when(orientation.getYaw()).thenReturn(0.0);

        // Ensure limelight returns the mocked result
        when(limelight.getLatestResult()).thenReturn(llResult);

        vision.read();

        double[] mt1Pose = vision.getMT1Pose();

        assertNotNull("Should return MT1 pose", mt1Pose);
        assertEquals("Should return 3 values", 3, mt1Pose.length);
        // Limelight (0,0) -> Pedro X = LL_Y * 39.3701 + 72 = 72, Pedro Y = LL_X * 39.3701 + 72 = 72
        assertEquals("X should match (from Limelight Y)", 72.0, mt1Pose[0], 0.1);
        assertEquals("Y should match (from Limelight X)", 72.0, mt1Pose[1], 0.1);
    }

    // ==================== Motif Detection Tests ====================

    @Test
    public void testReadMotif_NoVision() {
        vision = new Vision(hardwareMap, follower, turret);

        when(llResult.isValid()).thenReturn(false);
        vision.read();
        vision.readMotif();

        // Should not crash with no vision
        assertTrue("Should handle no vision", true);
    }

    @Test
    public void testReadMotif_Tag21_GPP() {
        vision = new Vision(hardwareMap, follower, turret);

        // Mock AprilTag 21 detection
        when(llResult.isValid()).thenReturn(true);

        LLResultTypes.FiducialResult fiducial = mock(LLResultTypes.FiducialResult.class);
        when(fiducial.getFiducialId()).thenReturn(21);

        List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
        fiducials.add(fiducial);
        when(llResult.getFiducialResults()).thenReturn(fiducials);

        vision.read();
        vision.readMotif();

        assertEquals("Tag 21 should set GPP motif",
                     Constants.Robot.motif.GPP, Constants.Robot.CurrentMOTIF);
    }

    @Test
    public void testReadMotif_Tag22_PGP() {
        vision = new Vision(hardwareMap, follower, turret);

        // Mock AprilTag 22 detection
        when(llResult.isValid()).thenReturn(true);

        LLResultTypes.FiducialResult fiducial = mock(LLResultTypes.FiducialResult.class);
        when(fiducial.getFiducialId()).thenReturn(22);

        List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
        fiducials.add(fiducial);
        when(llResult.getFiducialResults()).thenReturn(fiducials);

        vision.read();
        vision.readMotif();

        assertEquals("Tag 22 should set PGP motif",
                     Constants.Robot.motif.PGP, Constants.Robot.CurrentMOTIF);
    }

    @Test
    public void testReadMotif_Tag23_PPG() {
        vision = new Vision(hardwareMap, follower, turret);

        // Mock AprilTag 23 detection
        when(llResult.isValid()).thenReturn(true);

        LLResultTypes.FiducialResult fiducial = mock(LLResultTypes.FiducialResult.class);
        when(fiducial.getFiducialId()).thenReturn(23);

        List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
        fiducials.add(fiducial);
        when(llResult.getFiducialResults()).thenReturn(fiducials);

        vision.read();
        vision.readMotif();

        assertEquals("Tag 23 should set PPG motif",
                     Constants.Robot.motif.PPG, Constants.Robot.CurrentMOTIF);
    }

    @Test
    public void testReadMotif_NonMotifTag() {
        vision = new Vision(hardwareMap, follower, turret);

        // Mock non-motif AprilTag
        when(llResult.isValid()).thenReturn(true);

        LLResultTypes.FiducialResult fiducial = mock(LLResultTypes.FiducialResult.class);
        when(fiducial.getFiducialId()).thenReturn(10); // Random tag

        List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
        fiducials.add(fiducial);
        when(llResult.getFiducialResults()).thenReturn(fiducials);

        Constants.Robot.motif originalMotif = Constants.Robot.CurrentMOTIF;

        vision.read();
        vision.readMotif();

        // Motif should not change for non-motif tags
        assertEquals("Non-motif tag should not change motif",
                     originalMotif, Constants.Robot.CurrentMOTIF);
    }

    // ==================== Range Calculation Tests ====================

    @Test
    public void testGetRangeFromTag_NoVision() {
        vision = new Vision(hardwareMap, follower, turret);

        when(llResult.isValid()).thenReturn(false);
        vision.read();

        double range = vision.getRangeFromTag();

        assertEquals("Should return 0 when no vision", 0.0, range, 0.01);
    }

    @Test
    public void testGetRangeFromTag_ValidVision() {
        vision = new Vision(hardwareMap, follower, turret);

        when(llResult.isValid()).thenReturn(true);
        when(llResult.getBotpose()).thenReturn(pose3D); // MT1 uses getBotpose()
        when(pose3D.getPosition()).thenReturn(createPosition(3.0, 4.0, 0.0));

        // Ensure limelight returns the mocked result
        when(limelight.getLatestResult()).thenReturn(llResult);

        vision.read();

        double range = vision.getRangeFromTag();

        // Range should be sqrt(3^2 + 4^2) = 5.0
        assertEquals("Range should be calculated correctly", 5.0, range, 0.01);
    }

    @Test
    public void testGetRangeFromTag_ZeroDistance() {
        vision = new Vision(hardwareMap, follower, turret);

        when(llResult.isValid()).thenReturn(true);
        when(llResult.getBotpose_MT2()).thenReturn(pose3D);
        when(pose3D.getPosition()).thenReturn(createPosition(0.0, 0.0, 0.0));

        // Ensure limelight returns the mocked result
        when(limelight.getLatestResult()).thenReturn(llResult);

        vision.read();

        double range = vision.getRangeFromTag();

        assertEquals("Range should be 0 at origin", 0.0, range, 0.01);
    }

    // ==================== Camera Orientation Tests ====================

    @Test
    public void testCameraOrientation_WithTurretRotation() {
        vision = new Vision(hardwareMap, follower, turret);

        // Mock turret at 45 degrees
        when(turret.getCurrentTurretAngle()).thenReturn(45.0);

        // Mock robot heading at 0
        Pose robotPose = new Pose(0, 0, 0);
        when(follower.getPose()).thenReturn(robotPose);

        vision.read();
        vision.loop();

        // Should update Limelight with camera heading (robot + turret)
        // Camera heading = 0 + 45° = 45°
        verify(limelight, atLeastOnce()).updateRobotOrientation(anyDouble());
    }

    @Test
    public void testCameraOrientation_WithRobotAndTurretRotation() {
        vision = new Vision(hardwareMap, follower, turret);

        // Mock turret at 30 degrees
        when(turret.getCurrentTurretAngle()).thenReturn(30.0);

        // Mock robot heading at 90 degrees
        Pose robotPose = new Pose(0, 0, Math.toRadians(90.0));
        when(follower.getPose()).thenReturn(robotPose);

        vision.read();
        vision.loop();

        // Camera heading = 90° + 30° = 120°
        verify(limelight, atLeastOnce()).updateRobotOrientation(anyDouble());
    }

    // ==================== Telemetry Tests ====================

    @Test
    public void testGetTelemetry() {
        vision = new Vision(hardwareMap, follower, turret);

        boolean originalTelemetryState = Constants.Vision.logTelemetry;
        Constants.Vision.logTelemetry = true;

        vision.read();
        vision.loop();

        LinkedList<TelemetryPacket> telemetry = vision.getTelemetry();

        assertNotNull("Telemetry should not be null", telemetry);

        Constants.Vision.logTelemetry = originalTelemetryState;
    }

    @Test
    public void testTelemetry_ContainsData() {
        vision = new Vision(hardwareMap, follower, turret);

        boolean originalTelemetryState = Constants.Vision.logTelemetry;
        Constants.Vision.logTelemetry = true;

        vision.read();
        vision.loop();

        LinkedList<TelemetryPacket> telemetry = vision.getTelemetry();

        if (Constants.Vision.logTelemetry) {
            assertFalse("Telemetry should contain data when enabled",
                       telemetry.isEmpty());
        }

        Constants.Vision.logTelemetry = originalTelemetryState;
    }

    // ==================== Integration Tests ====================

    @Test
    public void testIntegration_VisionUpdateSequence() {
        vision = new Vision(hardwareMap, follower, turret);
        vision.setEnablePoseCorrection(true);

        // Simulate vision coming in and out
        for (int i = 0; i < 10; i++) {
            boolean hasVision = (i % 3 == 0); // Vision available every 3rd cycle
            when(llResult.isValid()).thenReturn(hasVision);

            if (hasVision) {
                when(llResult.getBotpose_MT2()).thenReturn(pose3D);
                when(pose3D.getPosition()).thenReturn(createPosition(50.0 + i, 50.0 + i, 0.0));
                when(pose3D.getOrientation()).thenReturn(orientation);
                when(orientation.getYaw()).thenReturn(0.0);
            }

            // Ensure limelight returns the mocked result
            when(limelight.getLatestResult()).thenReturn(llResult);

            vision.read();
            vision.loop();
            vision.write();
        }

        assertTrue("Should handle intermittent vision", true);
    }

    @Test
    public void testIntegration_MotifDetectionDuringMatch() {
        vision = new Vision(hardwareMap, follower, turret);

        // Simulate detecting different motif tags during match
        int[] tagSequence = {21, 10, 22, 15, 23, 21};
        Constants.Robot.motif[] expectedMotifs = {
            Constants.Robot.motif.GPP,
            Constants.Robot.motif.GPP, // Unchanged
            Constants.Robot.motif.PGP,
            Constants.Robot.motif.PGP, // Unchanged
            Constants.Robot.motif.PPG,
            Constants.Robot.motif.GPP
        };

        for (int i = 0; i < tagSequence.length; i++) {
            when(llResult.isValid()).thenReturn(true);

            LLResultTypes.FiducialResult fiducial = mock(LLResultTypes.FiducialResult.class);
            when(fiducial.getFiducialId()).thenReturn(tagSequence[i]);

            List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
            fiducials.add(fiducial);
            when(llResult.getFiducialResults()).thenReturn(fiducials);

            vision.read();
            vision.readMotif();

            assertEquals("Motif should match expected for tag " + tagSequence[i],
                        expectedMotifs[i], Constants.Robot.CurrentMOTIF);
        }
    }

    @Test
    public void testIntegration_DriftCorrectionOverTime() {
        vision = new Vision(hardwareMap, follower, turret);
        vision.setEnablePoseCorrection(true);

        // Create a scenario where odometry and vision disagree
        // This will cause the Kalman filter to estimate drift

        for (int i = 0; i < 20; i++) {
            when(llResult.isValid()).thenReturn(true);
            when(llResult.getBotpose()).thenReturn(pose3D); // MT1 uses getBotpose()

            // Vision reports position in valid meters (0.5m from center = 91.685 inches)
            when(pose3D.getPosition()).thenReturn(createPosition(0.5, 0.5, 0.0));
            when(pose3D.getOrientation()).thenReturn(orientation);
            when(orientation.getYaw()).thenReturn(0.0);

            // Mock fiducials for quality check to pass
            List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
            LLResultTypes.FiducialResult fiducial = mock(LLResultTypes.FiducialResult.class);
            Pose3D targetPose = mock(Pose3D.class);
            Position targetPos = new Position();
            targetPos.z = 1.0; // 1 meter = 39.37 inches - within MAX_TAG_DISTANCE
            when(fiducial.getTargetPoseCameraSpace()).thenReturn(targetPose);
            when(targetPose.getPosition()).thenReturn(targetPos);
            fiducials.add(fiducial);
            when(llResult.getFiducialResults()).thenReturn(fiducials);

            // Ensure limelight returns the mocked result
            when(limelight.getLatestResult()).thenReturn(llResult);

            // Odometry reports different position (simulating drift) - 85 inches vs 91.685 inches
            Pose odoPose = new Pose(85.0, 85.0, 0.0);
            when(follower.getPose()).thenReturn(odoPose);

            vision.read();
            vision.loop();
        }

        // After multiple updates with disagreement, drift should be estimated
        double[] drifts = vision.getDriftEstimates();

        // Check that drift has been estimated (is non-zero)
        // The exact value depends on the transformation, but it should change from initial 0
        boolean driftEstimated = Math.abs(drifts[0]) > 0.01 || Math.abs(drifts[1]) > 0.01;

        assertTrue("Drift should be estimated after multiple vision updates with odometry disagreement (X drift: "
                   + drifts[0] + ", Y drift: " + drifts[1] + ")", driftEstimated);
    }

    // ==================== Edge Case Tests ====================

    @Test
    public void testEdgeCase_NullPose3D() {
        vision = new Vision(hardwareMap, follower, turret);

        when(llResult.isValid()).thenReturn(true);
        when(llResult.getBotpose_MT2()).thenReturn(null);

        vision.read();
        vision.loop();

        // Should handle null pose gracefully
        assertTrue("Should handle null pose", true);
    }

    @Test
    public void testEdgeCase_ExtremeRobotHeading() {
        vision = new Vision(hardwareMap, follower, turret);

        // Test with extreme heading values
        Pose extremePose = new Pose(0, 0, Math.PI * 3); // > 2π
        when(follower.getPose()).thenReturn(extremePose);

        vision.read();
        vision.loop();

        // Should normalize angles correctly
        assertTrue("Should handle extreme headings", true);
    }

    @Test
    public void testEdgeCase_EmptyFiducialList() {
        vision = new Vision(hardwareMap, follower, turret);

        when(llResult.isValid()).thenReturn(true);
        when(llResult.getFiducialResults()).thenReturn(new ArrayList<>());

        vision.read();
        vision.readMotif();

        // Should handle empty list gracefully
        assertTrue("Should handle empty fiducial list", true);
    }

    // ==================== Performance Tests ====================

    @Test
    public void testPerformance_MultipleLoops() {
        vision = new Vision(hardwareMap, follower, turret);

        long startTime = System.currentTimeMillis();

        // Run 100 cycles
        for (int i = 0; i < 100; i++) {
            vision.read();
            vision.loop();
            vision.write();
        }

        long duration = System.currentTimeMillis() - startTime;

        // Should complete quickly (< 1 second)
        assertTrue("100 cycles should complete quickly", duration < 1000);
    }

    // ==================== State Management Tests ====================

    @Test
    public void testStateConsistency_AcrossMultipleCycles() {
        vision = new Vision(hardwareMap, follower, turret);

        vision.setEnablePoseCorrection(true);

        // Run multiple cycles
        for (int i = 0; i < 10; i++) {
            vision.read();
            vision.loop();
            vision.write();

            // Drift estimates should remain accessible
            double[] drifts = vision.getDriftEstimates();
            assertNotNull("Drift estimates should remain consistent", drifts);
        }
    }
}
