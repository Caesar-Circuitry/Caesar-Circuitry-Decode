package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;

import java.util.LinkedList;

public class Drivetrain extends WSubsystem {
    private Follower follower;
    private boolean holdPoint = false;
    private boolean fieldCentric = true;

    private final LinkedList<TelemetryPacket> telemetryPackets = new LinkedList<>();


    public Drivetrain(HardwareMap hardwareMap, Pose startPose) {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    public Drivetrain(HardwareMap hardwareMap) {
        this(hardwareMap, Constants.Drivetrain.Pose);
    }

    @Override
    public void read() {
        // Pedro Pathing handles sensor reading internally
    }

    @Override
    public void loop() {
        // Update follower state
        follower.update();

        // Telemetry logging
        if (Constants.Drivetrain.logTelemetry) {
            telemetryPackets.clear();

            // Pose information
            Pose currentPose = follower.getPose();
            telemetryPackets.add(new TelemetryPacket("Pose X", currentPose.getX()));
            telemetryPackets.add(new TelemetryPacket("Pose Y", currentPose.getY()));
            telemetryPackets.add(new TelemetryPacket("Heading(rad)", currentPose.getHeading()));
            telemetryPackets.add(new TelemetryPacket("Heading(deg)", Math.toDegrees(currentPose.getHeading())));

            // Path following state
            telemetryPackets.add(new TelemetryPacket("Path T Value", follower.getCurrentTValue()));
            telemetryPackets.add(new TelemetryPacket("Is Busy", follower.isBusy()));

            // Control state
            telemetryPackets.add(new TelemetryPacket("Hold Active", holdPoint));
            telemetryPackets.add(new TelemetryPacket("Field Centric", fieldCentric));
            telemetryPackets.add(new TelemetryPacket("Alliance", Constants.Robot.alliance.name()));
        }
    }

    @Override
    public void write() {
        // Pedro Pathing handles motor writing internally via follower.update()
    }

    // Drivetrain control methods

    /**
     * Start teleop drive mode
     */
    public void startDrive() {
        follower.startTeleopDrive();
    }

    public InstantCommand start() {
        return new InstantCommand(this::startDrive);
    }

    /**
     * Reset drivetrain to corner position based on alliance
     */
    public void resetDrive() {
        if (Constants.Robot.alliance == Constants.Robot.Alliance.BLUE) {
            follower.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
        } else {
            follower.setPose(new Pose(8, 6.25, Math.toRadians(0)));
        }
    }

    public InstantCommand reset() {
        return new InstantCommand(this::resetDrive);
    }

    /**
     * Manual teleop drive with gamepad input
     * @param gamepad Gamepad input device
     */
    public void drive(Gamepad gamepad) {
        if (!holdPoint) {
            if (fieldCentric) {
                follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    false,
                    Constants.Robot.alliance == Constants.Robot.Alliance.BLUE ? Math.toRadians(180) : 0
                );
            } else {
                follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    true
                );
            }
        }
    }

    /**
     * Hold current position
     */
    public void holdCurrent() {
        follower.holdPoint(new BezierPoint(follower.getPose()), follower.getHeading(), true);
        holdPoint = true;
    }

    public InstantCommand hold() {
        return new InstantCommand(this::holdCurrent);
    }

    /**
     * Release hold and return to manual control
     */
    public void releaseHold() {
        holdPoint = false;
    }

    public InstantCommand release() {
        return new InstantCommand(this::releaseHold);
    }

    /**
     * Toggle between field-centric and robot-centric drive
     */
    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    public InstantCommand toggleCentric() {
        return new InstantCommand(this::toggleFieldCentric);
    }

    /**
     * Reset to corner (same as resetDrive)
     */
    public void cornerReset() {
        if (Constants.Robot.alliance == Constants.Robot.Alliance.BLUE) {
            follower.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
        } else {
            follower.setPose(new Pose(8, 6.25, Math.toRadians(0)));
        }
    }

    public InstantCommand corner() {
        return new InstantCommand(this::cornerReset);
    }

    /**
     * Set starting pose
     * @param startPose The starting pose
     */
    public void setStart(Pose startPose) {
        follower.setStartingPose(startPose);
    }

    /**
     * Get current pose
     * @return Current robot pose
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Check if follower is busy (path following)
     * @return true if actively following a path
     */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /**
     * Get current path T value (progress along path)
     * @return Current T value (0.0 to 1.0)
     */
    public double getT() {
        return follower.getCurrentTValue();
    }

    /**
     * Get the follower instance for advanced control
     * @return The Pedro Pathing Follower
     */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Get current alliance
     * @return Current alliance (RED or BLUE)
     */
    public Constants.Robot.Alliance getAlliance() {
        return Constants.Robot.alliance;
    }

    /**
     * Check if hold is active
     * @return true if drivetrain is holding position
     */
    public boolean isHoldActive() {
        return holdPoint;
    }

    /**
     * Check if field centric mode is active
     * @return true if field centric, false if robot centric
     */
    public boolean isFieldCentric() {
        return fieldCentric;
    }

    @Override
    public LinkedList<TelemetryPacket> getTelemetry() {
        return telemetryPackets;
    }
}
