package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Config.paths.WorldsQ11;
import org.firstinspires.ftc.teamcode.Config.robot;

@Autonomous
public class WorldsRed8393Auto extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private WorldsRed8393 paths;
    @Override
    public void initialize() {
        super.reset();
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap, Telemetry);
        robot.getHardware().getIntake().badCloseFeeder();
        paths = new WorldsRed8393(robot.getHardware().getFollower());
        waitForStart();
        double start = System.currentTimeMillis();
        robot.getHardware().getFollower().startTeleopDrive();
        while(System.currentTimeMillis() - start < 2000) {
            robot.getHardware().getFollower().setTeleOpDrive(.5, 0, 0);
            robot.getHardware().loop();
        }
        robot.getHardware().getFollower().setTeleOpDrive(0, 0, 0);
//        schedule(
//                new RunCommand(this.robot::read),
//                new RunCommand(this.robot::loop),
//                new RunCommand(this.robot::write),
//                new SequentialCommandGroup(
//                        robot.getHardware().getIntake().Hold(),
//                        robot.getHardware().getTurret().TargetAngle(50),
//                        robot.getHardware().getLauncher().LaunchSpeed(1580),
//                        new WaitCommand(3000),
//
//                        robot.getHardware().getIntake().Launch(),
//                        new WaitCommand(900),
//                        robot.getHardware().getIntake().closeFeeder(),
//
//
//                        new FollowPathCommand(robot.getHardware().getFollower(), paths.red(),false)
//
//                )
//
//        );
    }
}
