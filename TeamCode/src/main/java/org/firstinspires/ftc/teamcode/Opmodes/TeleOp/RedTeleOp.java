package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class RedTeleOp extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;

    @Override
    public void initialize() {
        super.reset();
        waitForStart();
        Constants.Robot.alliance = Constants.Robot.Alliance.RED;
        Constants.Robot.Goal = Constants.Robot.RedGoal;
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot = new robot(hardwareMap,gamepad1,gamepad2,Telemetry);

        // Attempt to set initial pose from vision (MT1)
//        if (robot.getHardware().getVision().setInitialPoseFromVision()) {
//            Telemetry.addLine("Initial pose set from vision");
//        } else {
//            Telemetry.addLine("Vision unavailable - using default pose");
//        }
//        Telemetry.update();

        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write));
    }

    @Override
    public void run(){
        super.run();
    }
}
