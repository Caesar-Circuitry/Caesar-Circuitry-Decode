package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.paths.BlueStartFromFar;
import org.firstinspires.ftc.teamcode.Config.robot;

public class BlueFarPathOnly extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private BlueStartFromFar paths;

    @Override
    public void initialize() {
        super.reset();
        waitForStart();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        paths = new BlueStartFromFar(robot.getHardware().getFollower());
        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write),
                new SequentialCommandGroup(
                        // ==================== LAUNCH 0 (Stay in Place) ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch0()),

                        // ==================== INTAKE 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intake0()),

                        // ==================== LAUNCH 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch1()),

                        // ==================== INTAKE 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intake1()),

                        // ==================== LAUNCH 2 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch2()),

                        // ==================== MOVE TO RAMP 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp0()),

                        // ==================== INTAKE RAMP 0 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp0()),

                        // ==================== LAUNCH 3 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch3()),

                        // ==================== MOVE TO RAMP 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp1()),

                        // ==================== INTAKE RAMP 1 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp1()),

                        // ==================== LAUNCH 4 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch4()),

                        // ==================== MOVE TO RAMP 2 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.moveToRamp2()),

                        // ==================== INTAKE RAMP 2 ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.intakeRamp2()),

                        // ==================== LAUNCH 5 (Final) ====================
                        new FollowPathCommand(robot.getHardware().getFollower(), paths.launch5())
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
