package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.Commands.LaunchWhenReady;
import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.paths.BlueStartFromClose;
import org.firstinspires.ftc.teamcode.Config.paths.BlueStartFromFar;
import org.firstinspires.ftc.teamcode.Config.robot;
@Disabled
public class BlueFarAuto extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private BlueStartFromFar paths;
    private long LaunchTime = 250;
    private long waitRampTime = 100;

    @Override
    public void initialize() {
        super.reset();
        waitForStart();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        paths = new BlueStartFromFar(robot.getHardware().getFollower());
        robot.getHardware().getFollower().setStartingPose(BlueStartFromClose.START_POSE);
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        // ==================== LAUNCH 0 (Preload from Start) ====================
                        new InstantCommand(()->robot.getHardware().getLauncher().LaunchPose(BlueStartFromFar.START_POSE, Constants.Robot.Goal)),
//                        robot.getHardware().getTurret().TargetBlueGoal(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch0()),
                        new LaunchWhenReady(robot.getHardware().getIntake(), robot.getHardware().getLauncher()),
                        new WaitCommand(LaunchTime),

                        // ==================== INTAKE ARTIFACT 0 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intake0()),
                        robot.getHardware().getIntake().Hold(),

                        // ==================== LAUNCH 1 ====================
                        new InstantCommand(()->robot.getHardware().getLauncher().LaunchPose(paths.launch1().endPose(), Constants.Robot.Goal)),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch1()),
                        new LaunchWhenReady(robot.getHardware().getIntake(), robot.getHardware().getLauncher()),
                        new WaitCommand(LaunchTime),

                        // ==================== INTAKE ARTIFACT 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intake1()),
                        robot.getHardware().getIntake().Hold(),

                        // ==================== LAUNCH 2 ====================
                        new InstantCommand(()->robot.getHardware().getLauncher().LaunchPose(paths.launch2().endPose(), Constants.Robot.Goal)),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch2()),
                        new LaunchWhenReady(robot.getHardware().getIntake(), robot.getHardware().getLauncher()),
                        new WaitCommand(LaunchTime),

                        // ==================== RAMP CYCLE 0 ====================
                        robot.getHardware().getIntake().Hold(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp0()),
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp0()),
                                new SequentialCommandGroup(new WaitCommand(waitRampTime), robot.getHardware().getIntake().GroundIntake())
                        ),
                        robot.getHardware().getIntake().Hold(),

                        // ==================== LAUNCH 3 ====================
                        new InstantCommand(()->robot.getHardware().getLauncher().LaunchPose(paths.launch3().endPose(), Constants.Robot.Goal)),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch3()),
                        new LaunchWhenReady(robot.getHardware().getIntake(), robot.getHardware().getLauncher()),
                        new WaitCommand(LaunchTime),

                        // ==================== RAMP CYCLE 1 ====================
                        robot.getHardware().getIntake().Hold(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp1()),
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp1()),
                                new SequentialCommandGroup(new WaitCommand(waitRampTime), robot.getHardware().getIntake().GroundIntake())
                        ),
                        robot.getHardware().getIntake().Hold(),

                        // ==================== LAUNCH 4 ====================
                        new InstantCommand(()->robot.getHardware().getLauncher().LaunchPose(paths.launch4().endPose(), Constants.Robot.Goal)),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch4()),
                        new LaunchWhenReady(robot.getHardware().getIntake(), robot.getHardware().getLauncher()),
                        new WaitCommand(LaunchTime),

                        // ==================== RAMP CYCLE 2 ====================
                        robot.getHardware().getIntake().Hold(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp2()),
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp2()),
                                new SequentialCommandGroup(new WaitCommand(waitRampTime), robot.getHardware().getIntake().GroundIntake())
                        ),
                        robot.getHardware().getIntake().Hold(),

                        // ==================== LAUNCH 5 (Final) ====================
                        new InstantCommand(()->robot.getHardware().getLauncher().LaunchPose(paths.launch5().endPose(), Constants.Robot.Goal)),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch5()),
                        new LaunchWhenReady(robot.getHardware().getIntake(), robot.getHardware().getLauncher()),
                        new WaitCommand(LaunchTime)
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
