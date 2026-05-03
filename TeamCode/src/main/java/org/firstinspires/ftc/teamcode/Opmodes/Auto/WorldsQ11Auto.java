package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.paths.Blue15Ball;
import org.firstinspires.ftc.teamcode.Config.paths.WorldsQ11;
import org.firstinspires.ftc.teamcode.Config.robot;
@Autonomous
public class WorldsQ11Auto extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private WorldsQ11 paths;


    @Override
    public void initialize() {
        super.reset();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        robot.getHardware().getTurret().enablePinpointTracking();
        paths = new WorldsQ11(robot.getHardware().getFollower());
        robot.getHardware().getFollower().setStartingPose(Blue15Ball.START_POSE);
        robot.getHardware().getIntake().badCloseFeeder();
        waitForStart();
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo1stLaunch(),false)
                )
        );
    }
    @Override
    public void run() {
        super.run();
    }

    @Override
    public void end() {

    }
}
