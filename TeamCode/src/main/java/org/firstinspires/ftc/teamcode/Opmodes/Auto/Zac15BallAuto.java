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
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo1stLaunch()),
                        new WaitCommand(750)
                        // ==================== INTAKE ARTIFACT 0 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeMid()),
//                        new WaitCommand(100),
//                        // ==================== MOVE TO LAUNCH 1 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo2ndLaunch()),
//                        new WaitCommand(750),
//                        // ==================== INTAKE ARTIFACT 1 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToIntakeTop()),
//                        new WaitCommand(100),
//                        // ==================== MOVE TO LAUNCH 2 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo3rdLaunch()),
//                        new WaitCommand(750),
//                        // ==================== MOVE TO RAMP 0 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToGate()),
//                        new WaitCommand(250),
//                        // ==================== INTAKE RAMP 0 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRampIntake()),
//                        new WaitCommand(1000),
//                        // ==================== MOVE TO LAUNCH 3 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo4thLaunch()),
//                        new WaitCommand(750),
//                        // ==================== MOVE TO RAMP 1 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToGate()),
//                        new WaitCommand(250),
//                        // ==================== INTAKE RAMP 1 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRampIntake()),
//                        new WaitCommand(1000),
//                        // ==================== MOVE TO LAUNCH 4 ====================
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveTo5thLaunch()),
//                        new WaitCommand(750)
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
