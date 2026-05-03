package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.google.gson.extras.examples.rawcollections.RawCollectionsExample;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.paths.Red15Ball;
import org.firstinspires.ftc.teamcode.Config.robot;

@Autonomous
public class Red15BallAuto extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private Red15Ball paths;

    @Override
    public void initialize() {
        super.reset();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        robot.getHardware().getTurret().enablePinpointTracking();
        paths = new Red15Ball(robot.getHardware().getFollower());
        robot.getHardware().getFollower().setStartingPose(Red15Ball.START_POSE);
        robot.getHardware().getIntake().badCloseFeeder();
        waitForStart();
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        robot.getHardware().getIntake().Hold(),
                        robot.getHardware().getLauncher().LaunchSpeed(1180),
                        robot.getHardware().getTurret().TargetAngle(-112),
                        // ==================== MOVE TO LAUNCH 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo1stLaunch(),false),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(800),
                        robot.getHardware().getIntake().closeFeeder(),
                        // ==================== INTAKE ARTIFACT 0 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        robot.getHardware().getLauncher().LaunchSpeed(1120),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeMid(),false),
                        new WaitCommand(10),
                        robot.getHardware().getTurret().TargetAngle(-123),
                        // ==================== MOVE TO LAUNCH 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo2ndLaunch(robot.getHardware().getIntake()),false),
                        robot.getHardware().getIntake().Hold(),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(800),
                        robot.getHardware().getIntake().closeFeeder(),
                        // ==================== INTAKE ARTIFACT 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        robot.getHardware().getLauncher().LaunchSpeed(1145),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeTop(),false),
                        new WaitCommand(10),
                        robot.getHardware().getTurret().TargetAngle(-85),
                        // ==================== MOVE TO LAUNCH 2 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo3rdLaunch(), false),
                        robot.getHardware().getIntake().Hold(),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(800),
                        robot.getHardware().getIntake().closeFeeder(),
                        new WaitCommand(2500),
                        // ==================== MOVE TO RAMP 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToGate(),false),
                        new WaitCommand(50),
                        // ==================== INTAKE RAMP 0 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRampIntake(), false),
                        robot.getHardware().getTurret().TargetAngle(-49),
                        new WaitCommand(2000),
                        robot.getHardware().getLauncher().LaunchSpeed(1175),
                        // ==================== MOVE TO LAUNCH 3 ====================
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo4thLaunch(),false),
                                new SequentialCommandGroup(
                                        new WaitCommand(750),
                                        robot.getHardware().getIntake().Hold()
                                        )

                                ),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(2200),
                        robot.getHardware().getIntake().closeFeeder(),
//                        robot.getHardware().getTurret().TargetAngle(-46),
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.park(),true)

                        // ==================== MOVE TO RAMP 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToGate(),false),
                        new WaitCommand(10),
                        // ==================== INTAKE RAMP 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRampIntake(), false),
                        new WaitCommand(500),
                        robot.getHardware().getTurret().TargetAngle(30),
                        robot.getHardware().getLauncher().LaunchSpeed(1050),
                        robot.getHardware().getIntake().Hold(),
                        // ==================== MOVE TO LAUNCH 4 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo5thLaunch(),false),
                        new WaitCommand(500),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(800),
                        robot.getHardware().getIntake().closeFeeder(),
                        robot.getHardware().getLauncher().LaunchSpeed(0),
                        robot.getHardware().getLauncher().stopPower()
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