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

import org.firstinspires.ftc.teamcode.Config.Commands.IntakeOff;
import org.firstinspires.ftc.teamcode.Config.Commands.LaunchWhenReady;
import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.paths.ZacPathing;
import org.firstinspires.ftc.teamcode.Config.robot;
@Autonomous
public class Zac15BallAuto extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private ZacPathing paths;

    @Override
    public void initialize() {
        super.reset();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        robot.getHardware().getTurret().enablePinpointTracking();
        paths = new ZacPathing(robot.getHardware().getFollower());
        robot.getHardware().getFollower().setStartingPose(ZacPathing.START_POSE);
        robot.getHardware().getIntake().badCloseFeeder();
        waitForStart();
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        robot.getHardware().getIntake().Hold(),
                        robot.getHardware().getLauncher().LaunchClose(),
                        robot.getHardware().getTurret().TargetAngle(54),
                        // ==================== MOVE TO LAUNCH 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo1stLaunch(),false),
                        robot.getHardware().getIntake().Launch(),
                        new WaitCommand(800),
                        robot.getHardware().getIntake().closeFeeder(),
                        // ==================== INTAKE ARTIFACT 0 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeMid(),false),
                        new WaitCommand(10),
                        robot.getHardware().getTurret().TargetAngle(45),
                        // ==================== MOVE TO LAUNCH 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo2ndLaunch(robot.getHardware().getIntake()),false),
                        robot.getHardware().getIntake().Hold(),
                        new WaitCommand(250),
                        // ==================== INTAKE ARTIFACT 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeTop(),false),
                        new WaitCommand(10),
                        robot.getHardware().getTurret().TargetAngle(90),
                        // ==================== MOVE TO LAUNCH 2 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo3rdLaunch(), false),
                        robot.getHardware().getIntake().Hold(),
                        new WaitCommand(250),
//                      // ==================== MOVE TO RAMP 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToGate(),false),
                        new WaitCommand(50),
                        // ==================== INTAKE RAMP 0 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRampIntake(), false),
                        new WaitCommand(300),
                        robot.getHardware().getIntake().Hold(),
                        // ==================== MOVE TO LAUNCH 3 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo4thLaunch(),false),
                        new WaitCommand(500),
                        // ==================== MOVE TO RAMP 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToGate(),false),
                        new WaitCommand(50),
                        // ==================== INTAKE RAMP 1 ====================
                        robot.getHardware().getIntake().GroundIntake(),
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRampIntake(), false),
                        new WaitCommand(300),
                        robot.getHardware().getIntake().Hold(),
                        // ==================== MOVE TO LAUNCH 4 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo5thLaunch(),false),
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