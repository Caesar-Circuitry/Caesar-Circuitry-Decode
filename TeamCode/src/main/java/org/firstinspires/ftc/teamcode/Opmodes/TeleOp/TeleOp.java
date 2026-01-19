package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Config.robot;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    private robot robot;
    private JoinedTelemetry Telemetry;

    @Override
    public void initialize() {
        super.reset();
        waitForStart();
        robot = new robot(hardwareMap,gamepad1,gamepad2);
        Telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        schedule(
                new RunCommand(this.robot::read),
                new RunCommand(this.robot::loop),
                new RunCommand(this.robot::write));
    }

    @Override
    public void run(){
        super.run();
        telemetry.addData("heading",Math.toDegrees(this.robot.getHardware().getFollower().getHeading()));
        telemetry.update();
    }
}
