package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.paths.ZacPathing;
import org.firstinspires.ftc.teamcode.Config.robot;
@Autonomous
public class ZacPathingStuff extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private ZacPathing paths;

    @Override
    public void initialize() {
        super.reset();
        waitForStart();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        robot.getHardware().getTurret().disablePinpointTracking();
        paths = new ZacPathing(robot.getHardware().getFollower());
        robot.getHardware().getFollower().setStartingPose(ZacPathing.START_POSE);
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        // ==================== MOVE TO LAUNCH 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunch()),

                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeMid()),

                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunchMid()),

                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeTop()),

                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToLaunchTop())
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
