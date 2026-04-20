package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.paths.Blue15BallGood;
import org.firstinspires.ftc.teamcode.Config.robot;

@Autonomous
public class Blue15BallAutoGood extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private Blue15BallGood paths;

    @Override
    public void initialize() {
        super.reset();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        robot.getHardware().getTurret().enablePinpointTracking();
        paths = new Blue15BallGood(robot.getHardware().getFollower());
        robot.getHardware().getFollower().setStartingPose(Blue15BallGood.START_POSE);
        robot.getHardware().getIntake().badCloseFeeder();
        waitForStart();
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        // ==================== LAUNCH 0 and Intake (6 Artifacts Launched, 6 scored)====================
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.getHardware().getFollower(), paths.chain1IntakeLaunchCycle(), false),
                                new SequentialCommandGroup(
                                        new WaitCommand(600)
                                )),
                        // ==================== Intake from Gate 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.chain2GateThenIntake(), false),
                        // ==================== LAUNCH 1 (3 Artifacts Launched, 9 scored)====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.chain3ReturnToLaunch(), false),
                        // ==================== Intake from Gate 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.chain4GateThenIntake(), false),
                        // ==================== LAUNCH 2 (3 Artifacts Launched, 12 scored) ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.chain5ReturnToLaunch(), false),
                        // ==================== Park (3 Artifacts Launched, 15 scored) ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.chain6ParkRoute(), false)
                )
        );
    }

    @Override
    public void run() {
        super.run();
    }

    @Override
    public void end() {
        //Constants.Drivetrain.Pose = robot.getHardware().getFollower().getPose();
    }
}