package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.paths.BlueStartFromClose;
import org.firstinspires.ftc.teamcode.Config.robot;
@Autonomous
public class BlueClosePathOnly extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private BlueStartFromClose paths;

    @Override
    public void initialize() {
        super.reset();
        waitForStart();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        robot.getHardware().getTurret().disablePinpointTracking();
        paths = new BlueStartFromClose(robot.getHardware().getFollower());
        robot.getHardware().getFollower().setStartingPose(BlueStartFromClose.START_POSE);
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        // ==================== MOVE TO LAUNCH 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch0()),

                        //test
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToClose()),
                        new WaitCommand(1000),

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

                        // ==================== MOVE TO LAUNCH 5 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch5()),

                        // ==================== MOVE TO RAMP 3 ===================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp3()),

                        // ==================== INTAKE RAMP 3 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp3()),

                        // ==================== MOVE TO LAUNCH 6 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch6())
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
