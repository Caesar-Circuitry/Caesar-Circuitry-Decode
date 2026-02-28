package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.paths.RedStartFromClose;
import org.firstinspires.ftc.teamcode.Config.robot;
@Autonomous
@Disabled
public class RedClosePathOnly extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private RedStartFromClose paths;

    @Override
    public void initialize() {
        super.reset();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        paths = new RedStartFromClose(robot.getHardware().getFollower());
//        robot.getHardware().getVision().setEnablePoseCorrection(false);
        robot.getHardware().getTurret().disablePinpointTracking();
        robot.getHardware().getFollower().setStartingPose(RedStartFromClose.START_POSE);
        waitForStart();
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        // ==================== MOVE TO LAUNCH 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch0()),

                        // ==================== INTAKE ARTIFACT 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeArtifact0()),

                        // ==================== MOVE TO LAUNCH 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch1()),

                        // ==================== INTAKE ARTIFACT 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeArtifact1()),

                        // ==================== MOVE TO LAUNCH 2 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch2()),

                        // ==================== MOVE TO RAMP 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp0()),

                        // ==================== INTAKE RAMP 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp0()),

                        // ==================== MOVE TO LAUNCH 3 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch3()),

                        // ==================== MOVE TO RAMP 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp1()),

                        // ==================== INTAKE RAMP 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp1()),

                        // ==================== MOVE TO LAUNCH 4 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch4()),

                        // ==================== MOVE TO RAMP 2 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp2()),

                        // ==================== INTAKE RAMP 2 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp2()),

                        // ==================== MOVE TO LAUNCH 5 (Final) ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch5())
                )
        );
    }

    @Override
    public void run() {
        super.run();
    }

    @Override
    public void end() {
        Constants.Drivetrain.Pose = robot.getHardware().getFollower().getPose();
    }
}
