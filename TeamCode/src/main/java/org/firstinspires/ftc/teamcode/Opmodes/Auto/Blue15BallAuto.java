package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.paths.Blue15Ball;
import org.firstinspires.ftc.teamcode.Config.robot;
@Autonomous
public class Blue15BallAuto extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private Blue15Ball paths;

    @Override
    public void initialize() {
        super.reset();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        robot.getHardware().getTurret().enablePinpointTracking();
        paths = new Blue15Ball(robot.getHardware().getFollower());
        robot.getHardware().getFollower().setStartingPose(Blue15Ball.START_POSE);
        robot.getHardware().getIntake().badCloseFeeder();
        waitForStart();
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        robot.getHardware().getIntake().Hold(),
                        robot.getHardware().getLauncher().LaunchSpeed(1200),
                        robot.getHardware().getTurret().TargetAngle(54),
                        // ==================== MOVE TO LAUNCH 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo1stLaunch(),false),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(800),
                        robot.getHardware().getIntake().closeFeeder(),
                        // ==================== INTAKE ARTIFACT 0 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        robot.getHardware().getLauncher().LaunchSpeed(1125),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeMid(),false),
                        new WaitCommand(10),
                        robot.getHardware().getTurret().TargetAngle(-49),
                        // ==================== MOVE TO LAUNCH 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo2ndLaunch(robot.getHardware().getIntake()),false),
                        robot.getHardware().getIntake().Hold(),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(800),
                        robot.getHardware().getIntake().closeFeeder(),
                        // ==================== INTAKE ARTIFACT 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        robot.getHardware().getLauncher().LaunchSpeed(1170),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeTop(),false),
                        new WaitCommand(10),
                        robot.getHardware().getTurret().TargetAngle(-97),
                        // ==================== MOVE TO LAUNCH 2 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo3rdLaunch(), false),
                        robot.getHardware().getIntake().Hold(),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(800),
                        robot.getHardware().getIntake().closeFeeder(),
//                      // ==================== MOVE TO RAMP 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToGate(),false),
                        new WaitCommand(10),
                        // ==================== INTAKE RAMP 0 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRampIntake(), false),
                        robot.getHardware().getTurret().TargetAngle(-120),
                        new WaitCommand(300),
                        robot.getHardware().getIntake().Hold(),
                        // ==================== MOVE TO LAUNCH 3 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo4thLaunch(),false),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(2200),
                        robot.getHardware().getIntake().closeFeeder(),
                        // ==================== MOVE TO RAMP 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToGate(),false),
                        new WaitCommand(10),
                        // ==================== INTAKE RAMP 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRampIntake(), false),
                        new WaitCommand(300),
                        robot.getHardware().getTurret().TargetAngle(-235),
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