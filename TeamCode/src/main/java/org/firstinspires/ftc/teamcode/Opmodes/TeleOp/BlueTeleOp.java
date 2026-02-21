package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.utils.LoopTimer;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.robot;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class BlueTeleOp extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;
    private LoopTimer loopTimer;

    @Override
    public void initialize() {
        super.reset();
        waitForStart();
        loopTimer = new LoopTimer();
        Constants.Robot.alliance = Constants.Robot.Alliance.BLUE;
        Constants.Robot.Goal = Constants.Robot.BlueGoal;
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
        loopTimer.start();
        super.run();
        loopTimer.end();
        Telemetry.addData("LoopTimeHz",loopTimer.getHz());
        Telemetry.addData("LoopTimeMs",loopTimer.getMs());
        Telemetry.update();
    }
}
